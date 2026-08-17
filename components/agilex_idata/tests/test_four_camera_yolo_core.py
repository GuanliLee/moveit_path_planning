import importlib.util
import json
import sys
import threading
import time
from pathlib import Path

import pytest


CORE_PATH = (
    Path(__file__).parents[1] / "scripts" / "inference" / "four_camera_yolo_core.py"
)


@pytest.fixture
def core():
    spec = importlib.util.spec_from_file_location("four_camera_yolo_core", CORE_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_parse_targets_accepts_commas_newlines_and_stable_deduplication(core):
    assert core.parse_targets(" Wanglaoji, Sprite\nWanglaoji ,, Daily C Grape Juice ") == (
        "Wanglaoji",
        "Sprite",
        "Daily C Grape Juice",
    )


@pytest.mark.parametrize(
    "value, expected", [(0.2, 0.2), ("0.3", 0.3), (1, 1.0), (60, 60.0)]
)
def test_validate_interval_sec_accepts_closed_range(core, value, expected):
    assert core.validate_interval_sec(value) == expected


@pytest.mark.parametrize("value", [None, True, 0.19, 60.01, "nan", "bad"])
def test_validate_interval_sec_rejects_invalid_values(core, value):
    with pytest.raises(ValueError):
        core.validate_interval_sec(value)


def state_with_published_left_result(core, targets="Wanglaoji"):
    state = core.StateStore(default_interval_sec=3.0)
    state.update_frame("left", b"source", stamp_ns=10, received_at=1.0)
    generation = state.apply_config(
        targets=targets, interval_sec=3.0, paused=False, cameras={}
    )
    frame = state.frame_snapshot("left")
    accepted = state.publish_result(
        "left",
        generation,
        frame,
        core.InferenceResult(
            overlay_jpeg=b"overlay",
            count=1,
            class_counts={targets: 1},
            service_latency_ms=10.0,
            round_trip_ms=12.0,
            completed_at=3.0,
        ),
    )
    assert accepted is True
    return state


def test_latest_frame_replaces_previous_without_queue(core):
    state = core.StateStore(default_interval_sec=3.0)
    state.update_frame("left", b"first", stamp_ns=10, received_at=1.0)
    state.update_frame("left", b"second", stamp_ns=20, received_at=2.0)
    frame = state.frame_snapshot("left")
    assert frame.jpeg == b"second"
    assert frame.sequence == 2
    assert frame.stamp_ns == 20


def test_result_from_old_config_generation_is_discarded(core):
    state = core.StateStore(default_interval_sec=3.0)
    state.update_frame("left", b"source", stamp_ns=10, received_at=1.0)
    generation = state.apply_config(
        targets="Wanglaoji", interval_sec=3.0, paused=False, cameras={}
    )
    frame = state.frame_snapshot("left")
    state.apply_config(targets="Sprite", interval_sec=3.0, paused=False, cameras={})
    accepted = state.publish_result(
        "left",
        generation,
        frame,
        core.InferenceResult(
            overlay_jpeg=b"overlay",
            count=1,
            class_counts={"Wanglaoji": 1},
            service_latency_ms=10.0,
            round_trip_ms=12.0,
            completed_at=3.0,
        ),
    )
    assert accepted is False
    assert state.image("left", "overlay") is None


def test_applying_new_prompt_clears_previous_prompt_images(core):
    state = state_with_published_left_result(core, targets="Wanglaoji")
    assert state.image("left", "overlay") is not None
    state.apply_config("Sprite", 3.0, False, {})
    assert state.image("left", "source") is None
    assert state.image("left", "overlay") is None


def test_config_updates_are_validated_atomically(core):
    state = core.StateStore(default_interval_sec=3.0)
    state.apply_config("Wanglaoji", 3.0, False, {"left": {"enabled": False}})
    before = state.status(now=1.0)

    with pytest.raises(ValueError, match="interval_sec must be between"):
        state.apply_config(
            "Sprite",
            60.01,
            True,
            {"left": {"enabled": True}},
        )

    assert state.status(now=1.0) == before


@pytest.mark.parametrize(
    "cameras, message",
    [
        ({"missing": {}}, "unknown camera"),
        ({"left": []}, "camera configuration must be an object"),
        ({"left": {"unknown": True}}, "unknown camera configuration field"),
        ({"left": {"enabled": 1}}, "enabled must be boolean"),
    ],
)
def test_config_rejects_malformed_camera_updates(core, cameras, message):
    state = core.StateStore(default_interval_sec=3.0)
    with pytest.raises(ValueError, match=message):
        state.apply_config("Wanglaoji", 3.0, False, cameras)


def test_config_payload_and_worker_snapshot(core):
    state = core.StateStore(default_interval_sec=3.0)
    generation = state.apply_config_payload(
        {
            "targets": "Wanglaoji, Sprite",
            "interval_sec": "4",
            "paused": True,
            "cameras": {"right": {"enabled": False}},
        }
    )

    assert state.config_snapshot("right") == core.WorkerConfigSnapshot(
        generation=generation,
        targets=("Wanglaoji", "Sprite"),
        paused=True,
        enabled=False,
        interval_sec=4.0,
    )
    assert state.global_config_snapshot() == core.GlobalConfigSnapshot(
        generation=generation,
        targets=("Wanglaoji", "Sprite"),
        paused=True,
        interval_sec=4.0,
    )


@pytest.mark.parametrize(
    "payload, message",
    [
        ([], "configuration must be a JSON object"),
        ({"unknown": True}, "unknown configuration field"),
        ({"cameras": []}, "cameras must be an object"),
    ],
)
def test_config_payload_rejects_malformed_objects(core, payload, message):
    state = core.StateStore(default_interval_sec=3.0)
    with pytest.raises(ValueError, match=message):
        state.apply_config_payload(payload)


def test_errors_are_generation_scoped(core):
    state = core.StateStore(default_interval_sec=3.0)
    old_generation = state.apply_config("Wanglaoji", 3.0, False, {})
    generation = state.apply_config("Wanglaoji", 4.0, False, {})

    assert state.record_error("left", old_generation, "old", when=1.0) is False
    assert state.record_error("left", generation, "current", when=2.0) is True
    assert state.status(now=2.0)["cameras"]["left"]["error"] == "current"


def test_health_and_classes_snapshots_do_not_expose_mutable_state(core):
    state = core.StateStore(default_interval_sec=3.0)
    health = {"model": "yolo"}
    classes = ({"name": "Wanglaoji", "id": 1},)
    state.update_health(True, health, classes, error="", checked_at=1.0)

    health["model"] = "changed"
    first = state.classes_snapshot()
    first[0]["name"] = "changed"

    assert state.classes_snapshot() == [{"name": "Wanglaoji", "id": 1}]
    assert state.status(now=1.0)["yolo"] == {
        "ok": True,
        "error": "",
        "checked_at": 1.0,
        "check_age_sec": 0.0,
    }


def test_status_is_json_safe_and_contains_derived_camera_metrics(core):
    state = state_with_published_left_result(core)
    state.record_attempt("left", started=2.8, completed=3.0)
    status = state.status(now=3.2)
    left = status["cameras"]["left"]

    json.dumps(status)
    assert status["generation"] == 1
    assert status["targets"] == ["Wanglaoji"]
    assert status["interval_sec"] == 3.0
    assert status["config_error"] == ""
    assert left["input_sequence"] == 1
    assert left["result_sequence"] == 1
    assert left["source_frame_sequence"] == 1
    assert left["source_stamp_sec"] == pytest.approx(1e-8)
    assert left["topic_online"] is False
    assert left["effective_rate_hz"] > 0.0
    assert left["service_latency_ms"] == 10.0
    assert left["round_trip_ms"] == 12.0
    assert left["result_age_sec"] == pytest.approx(0.2)
    assert left["result_stale"] is False
    assert left["count"] == 1
    assert left["class_counts"] == {"Wanglaoji": 1}

    left["class_counts"]["Wanglaoji"] = 99
    assert state.status(now=3.2)["cameras"]["left"]["class_counts"] == {
        "Wanglaoji": 1
    }


def test_status_reports_health_and_error_ages(core):
    state = core.StateStore(default_interval_sec=3.0)
    state.update_health(True, {"ok": True}, (), "", checked_at=4.0)
    generation = state.apply_config("Wanglaoji", 3.0, False, {})
    state.record_error("left", generation, "temporary failure", when=3.0)
    status = state.status(now=5.5)
    assert status["yolo"]["check_age_sec"] == pytest.approx(1.5)
    assert status["cameras"]["left"]["error_age_sec"] == pytest.approx(2.5)


def test_unknown_camera_and_image_kind_are_rejected(core):
    state = core.StateStore(default_interval_sec=3.0)
    with pytest.raises(ValueError, match="unknown camera"):
        state.frame_snapshot("missing")
    with pytest.raises(ValueError, match="unknown image kind"):
        state.image("left", "mask")


def inference_result(core, camera, completed_at=None):
    return core.InferenceResult(
        overlay_jpeg=f"overlay-{camera}".encode(),
        count=1,
        class_counts={"Wanglaoji": 1},
        service_latency_ms=8.0,
        round_trip_ms=10.0,
        completed_at=time.monotonic() if completed_at is None else completed_at,
    )


def test_worker_uses_newest_frame_and_publishes_matching_pair(core):
    state = core.StateStore(default_interval_sec=3.0)
    state.apply_config("Wanglaoji", 3.0, False, {})
    state.update_frame("left", b"old", 1, 1.0)
    state.update_frame("left", b"new", 2, 2.0)

    class Client:
        frames = []

        def predict(self, camera, frame, targets):
            self.frames.append(frame.jpeg)
            assert targets == ("Wanglaoji",)
            return inference_result(core, camera)

    client = Client()
    worker = core.InferenceWorker("left", state, lambda: client, time.monotonic)
    assert worker.run_once() is True
    assert client.frames == [b"new"]
    assert state.image("left", "source") == b"new"
    assert state.image("left", "overlay") == b"overlay-left"


def test_worker_discards_response_when_configuration_changes_in_flight(core):
    state = core.StateStore(default_interval_sec=3.0)
    state.apply_config("Wanglaoji", 3.0, False, {})
    state.update_frame("left", b"source", 1, time.monotonic())
    started = threading.Event()
    release = threading.Event()

    class Client:
        def predict(self, camera, frame, targets):
            started.set()
            assert release.wait(1.0)
            return inference_result(core, camera)

    worker = core.InferenceWorker("left", state, Client, time.monotonic)
    thread = threading.Thread(target=worker.run_once)
    thread.start()
    assert started.wait(1.0)
    state.apply_config("Sprite", 4.0, False, {})
    release.set()
    thread.join(1.0)
    assert not thread.is_alive()
    assert state.image("left", "overlay") is None


def test_worker_records_one_camera_error_without_raising(core):
    state = core.StateStore(default_interval_sec=3.0)
    state.apply_config("Wanglaoji", 3.0, False, {})
    state.update_frame("right", b"source", 1, time.monotonic())

    class Client:
        def predict(self, camera, frame, targets):
            raise RuntimeError("forced right failure")

    worker = core.InferenceWorker("right", state, Client, time.monotonic)
    assert worker.run_once() is True
    assert state.status(time.monotonic())["cameras"]["right"]["error"] == (
        "forced right failure"
    )


def test_worker_group_starts_four_requests_together_and_skips_busy_round(core):
    state = core.StateStore(default_interval_sec=3.0)
    state.apply_config("Wanglaoji", 3.0, False, {})
    for index, camera in enumerate(core.CAMERA_KEYS):
        state.update_frame(camera, f"source-{camera}".encode(), index, time.monotonic())

    lock = threading.Lock()
    all_started = threading.Event()
    release = threading.Event()
    calls = []

    class Client:
        def predict(self, camera, frame, targets):
            with lock:
                calls.append(camera)
                if len(calls) == 4:
                    all_started.set()
            assert release.wait(2.0)
            return inference_result(core, camera)

    stop_event = threading.Event()
    group = core.WorkerGroup(state, Client, stop_event, time.monotonic)
    assert group.run_round() == 4
    assert all_started.wait(1.0)
    assert set(calls) == set(core.CAMERA_KEYS)
    assert group.run_round() == 0
    release.set()
    assert group.wait_for_idle(2.0) is True
    group.stop(timeout=2.0)

    for camera in core.CAMERA_KEYS:
        assert state.image(camera, "source") == f"source-{camera}".encode()
        assert state.image(camera, "overlay") == f"overlay-{camera}".encode()


def test_worker_group_wakes_immediately_when_configuration_is_applied(core):
    state = core.StateStore(default_interval_sec=60.0)
    state.update_frame("left", b"source-left", 1, time.monotonic())
    called = threading.Event()

    class Client:
        def predict(self, camera, frame, targets):
            if camera == "left":
                called.set()
            return inference_result(core, camera)

    stop_event = threading.Event()
    group = core.WorkerGroup(state, Client, stop_event, time.monotonic)
    group.start()
    assert group.wait_for_idle(1.0) is True
    state.apply_config("Wanglaoji", 3.0, False, {})
    try:
        assert called.wait(0.5), "new configuration did not wake the scheduler"
    finally:
        group.stop(timeout=2.0)


def test_health_monitor_filters_undeployed_classes_and_keeps_camera_state(core):
    state = core.StateStore(default_interval_sec=3.0)

    class Client:
        def health(self):
            return {"ok": True, "api_version": "v1"}

        def classes(self):
            return {
                "ok": True,
                "classes": [
                    {"name": "Wanglaoji", "supported_by_deployed_model": True},
                    {"name": "Coca-Cola", "supported_by_deployed_model": False},
                ],
            }

    monitor = core.HealthMonitor(state, Client, lambda: 5.0)
    assert monitor.run_once() is True
    assert state.classes_snapshot() == [
        {"name": "Wanglaoji", "supported_by_deployed_model": True}
    ]
    assert state.status(5.0)["yolo"] == {
        "ok": True,
        "error": "",
        "checked_at": 5.0,
        "check_age_sec": 0.0,
    }


def test_health_monitor_reports_offline_without_clearing_camera_state(core):
    state = state_with_published_left_result(core)

    class Client:
        def health(self):
            raise RuntimeError("connection refused")

    monitor = core.HealthMonitor(state, Client, lambda: 5.0)
    assert monitor.run_once() is False
    status = state.status(5.0)
    assert status["yolo"]["ok"] is False
    assert "connection refused" in status["yolo"]["error"]
    assert state.image("left", "overlay") == b"overlay"

from __future__ import annotations

import ast
import importlib.util
import os
import queue
import subprocess
import textwrap
import threading
from pathlib import Path
from types import SimpleNamespace

import pytest


REPO_ROOT = Path(__file__).resolve().parents[1]
WRAPPER = REPO_ROOT / "scripts" / "collection" / "collect_mobile_pipeline_web_grasp_place.sh"
RULES = REPO_ROOT / "scripts" / "collection" / "grasp_place_collection.py"
STAGED = REPO_ROOT / "scripts" / "collection" / "collect_mobile_pipeline_web_staged.sh"


def load_rules_module():
    spec = importlib.util.spec_from_file_location("grasp_place_collection_test", RULES)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def load_staged_bridge_function(name: str):
    text = STAGED.read_text(encoding="utf-8")
    source = text.rsplit("from __future__ import annotations", 1)[1].split("\nPY\n", 1)[0]
    tree = ast.parse("from __future__ import annotations" + source)
    included_names = {name}
    if name == "wait_for_initial_status":
        included_names.add("validated_initial_episode")
    if name == "configure_collection_for_target":
        included_names.update({"collection_config_payload", "wait_for_status"})
    selected = [
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name in included_names
    ]
    assert len(selected) == len(included_names)
    namespace = {
        "os": os,
        "queue": queue,
        "rclpy": SimpleNamespace(ok=lambda: True),
        "time": __import__("time"),
    }
    exec(compile(ast.Module(body=selected, type_ignores=[]), str(STAGED), "exec"), namespace)
    return namespace[name]


def test_grasp_place_wrapper_forwards_args_and_enables_two_phase_defaults(tmp_path: Path):
    staged_stub = tmp_path / "staged_stub.sh"
    output = tmp_path / "wrapper-env.txt"
    staged_stub.write_text(
        textwrap.dedent(
            """\
            #!/usr/bin/env bash
            set -eo pipefail
            {
              printf 'args=%s\\n' "$*"
              printf 'GRASP_PLACE_COLLECTION_ENABLE=%s\\n' "${GRASP_PLACE_COLLECTION_ENABLE-}"
              printf 'GRASP_PLACE_DATA_ROOT=%s\\n' "${GRASP_PLACE_DATA_ROOT-}"
              printf 'GRASP_PLACE_START_TOPIC=%s\\n' "${GRASP_PLACE_START_TOPIC-}"
              printf 'GRASP_PLACE_GRASP_END_TOPIC=%s\\n' "${GRASP_PLACE_GRASP_END_TOPIC-}"
              printf 'STATE_MACHINE_START_TOPIC=%s\\n' "${STATE_MACHINE_START_TOPIC-}"
              printf 'STATE_MACHINE_END_TOPIC=%s\\n' "${STATE_MACHINE_END_TOPIC-}"
              printf 'DATA_COLLECTION_START_TOPIC=%s\\n' "${DATA_COLLECTION_START_TOPIC-}"
              printf 'DATA_COLLECTION_SAVE_SUCCESS_TOPIC=%s\\n' "${DATA_COLLECTION_SAVE_SUCCESS_TOPIC-}"
              printf 'COLLECTION_STAGED_CAPTURE_DEFAULT=%s\\n' "${COLLECTION_STAGED_CAPTURE_DEFAULT-}"
              printf 'STATE_MACHINE_STAGED_DEFAULT=%s\\n' "${STATE_MACHINE_STAGED_DEFAULT-}"
            } > "${GRASP_PLACE_TEST_OUTPUT}"
            """
        ),
        encoding="utf-8",
    )
    staged_stub.chmod(0o755)
    env = os.environ.copy()
    env.update(
        {
            "COLLECT_MOBILE_PIPELINE_WEB_STAGED_SCRIPT": str(staged_stub),
            "GRASP_PLACE_TEST_OUTPUT": str(output),
        }
    )

    subprocess.run(
        [
            "bash",
            str(WRAPPER),
            "/home/agilex/data/stage2_twohand/20260803_scene1",
            "7",
            "--grade",
            "A",
        ],
        cwd=REPO_ROOT,
        env=env,
        check=True,
    )

    assert output.read_text(encoding="utf-8") == textwrap.dedent(
        """\
        args=/home/agilex/data/stage2_twohand/20260803_scene1 7 --grade A
        GRASP_PLACE_COLLECTION_ENABLE=1
        GRASP_PLACE_DATA_ROOT=/home/agilex/data/place
        GRASP_PLACE_START_TOPIC=/state_place/start
        GRASP_PLACE_GRASP_END_TOPIC=/state_machine/grasping/end
        STATE_MACHINE_START_TOPIC=/state_machine/start
        STATE_MACHINE_END_TOPIC=/state_machine/end
        DATA_COLLECTION_START_TOPIC=/data_collection/start
        DATA_COLLECTION_SAVE_SUCCESS_TOPIC=/state_collect/end
        COLLECTION_STAGED_CAPTURE_DEFAULT=0
        STATE_MACHINE_STAGED_DEFAULT=0
        """
    )


def test_place_directory_uses_configuration_name_not_episode_directory():
    rules = load_rules_module()

    assert rules.derive_place_data_dir(
        "/home/agilex/data/stage2_twohand/20260803_scene1/",
        "/home/agilex/data/place",
    ) == "/home/agilex/data/place/20260803_scene1"


def test_two_phase_event_routes_only_the_event_valid_for_current_state():
    rules = load_rules_module()

    assert rules.two_phase_event_action("grasp", "idle", "start") == "start_grasp"
    assert rules.two_phase_event_action("grasp", "recording", "grasp_end") == "save_grasp"
    assert rules.two_phase_event_action("place", "idle", "place_start") == "start_place"
    assert rules.two_phase_event_action("place", "recording", "end") == "save_place"

    assert rules.two_phase_event_action("grasp", "idle", "place_start") == "ignore"
    assert rules.two_phase_event_action("grasp", "recording", "end") == "ignore"
    assert rules.two_phase_event_action("place", "idle", "start") == "ignore"
    assert rules.two_phase_event_action("place", "recording", "grasp_end") == "ignore"
    assert rules.two_phase_event_action("place", "saving", "end") == "ignore"


def test_review_outcome_distinguishes_accepted_discarded_and_incomplete():
    rules = load_rules_module()

    assert rules.review_outcome(
        {
            "capture_running": False,
            "quality_review_pending": False,
            "last_saved_episode": "7",
            "episode": 8,
        },
        7,
    ) == "accepted"
    assert rules.review_outcome(
        {
            "capture_running": False,
            "quality_review_pending": False,
            "last_saved_episode": "",
            "episode": 7,
        },
        7,
    ) == "discarded"
    assert rules.review_outcome(
        {
            "capture_running": False,
            "quality_review_pending": True,
            "last_saved_episode": "7",
            "episode": 7,
        },
        7,
    ) == "incomplete"
    assert rules.review_outcome(
        {
            "capture_running": False,
            "quality_review_pending": False,
            "last_saved_episode": "6",
            "episode": 8,
        },
        7,
    ) == "incomplete"


def test_review_transition_keeps_episode_paired_until_place_is_accepted():
    rules = load_rules_module()

    assert rules.capture_target_after_review("grasp", "accepted", 7) == ("place", 7)
    assert rules.capture_target_after_review("grasp", "discarded", 7) == ("grasp", 7)
    assert rules.capture_target_after_review("place", "discarded", 7) == ("place", 7)
    assert rules.capture_target_after_review("place", "accepted", 7) == ("grasp", 8)


def test_late_review_reconciliation_never_rewinds_pending_episode():
    rules = load_rules_module()
    pending = {
        "capture_running": False,
        "quality_review_pending": True,
        "last_saved_episode": "7",
        "episode": 7,
    }
    accepted_later = {
        "capture_running": False,
        "quality_review_pending": False,
        "last_saved_episode": "7",
        "episode": 8,
    }

    assert rules.review_target_if_complete(pending, "grasp", 7) is None
    assert rules.review_target_if_complete(accepted_later, "grasp", 7) == (
        "accepted",
        "place",
        7,
    )


def test_resolved_review_target_survives_applied_but_unconfirmed_config():
    rules = load_rules_module()
    accepted_grasp = {
        "capture_running": False,
        "quality_review_pending": False,
        "last_saved_episode": "7",
        "episode": 8,
    }
    target = rules.review_transition_or_cached(
        None,
        accepted_grasp,
        "grasp",
        7,
        "/grasp/scene1",
        "/place/scene1",
    )
    assert target == ("accepted", "place", 7, "/place/scene1")

    # /config has already applied place/episode7, but its confirmation request failed.
    # Reconciliation must retry the frozen target instead of reclassifying this status.
    status_after_applied_config = {
        "capture_running": False,
        "quality_review_pending": False,
        "last_saved_episode": "7",
        "episode": 7,
    }
    assert rules.review_transition_or_cached(
        target,
        status_after_applied_config,
        "grasp",
        7,
        "/grasp/scene1",
        "/place/scene1",
    ) == target


def test_applied_but_unconfirmed_target_config_retries_idempotently():
    rules = load_rules_module()
    configure_target = load_staged_bridge_function("configure_collection_for_target")
    accepted_grasp = {
        "data_dir": "/grasp/scene1",
        "capture_running": False,
        "quality_review_pending": False,
        "staged_capture": False,
        "last_saved_episode": "7",
        "episode": 8,
    }
    target = rules.review_transition_or_cached(
        None,
        accepted_grasp,
        "grasp",
        7,
        "/grasp/scene1",
        "/place/scene1",
    )
    assert target is not None

    class AppliedThenUnconfirmedHttp:
        def __init__(self):
            self.current = dict(accepted_grasp)
            self.fail_confirmation = False
            self.config_posts = 0

        def status(self):
            if self.fail_confirmation:
                self.fail_confirmation = False
                raise ConnectionError("confirmation response lost")
            return dict(self.current)

        def post_json(self, path, payload):
            assert path == "/config"
            self.config_posts += 1
            self.current.update(
                data_dir=payload["data_dir"],
                episode=int(payload["episode_index"]),
                staged_capture=payload["staged_capture"],
            )
            self.fail_confirmation = True

    http = AppliedThenUnconfirmedHttp()
    _, _, target_episode, target_dir = target
    with pytest.raises(ConnectionError, match="confirmation response lost"):
        configure_target(http, data_dir=target_dir, episode_index=target_episode)

    applied_status = http.status()
    retried_target = rules.review_transition_or_cached(
        target,
        applied_status,
        "grasp",
        7,
        "/grasp/scene1",
        "/place/scene1",
    )
    assert retried_target == target
    confirmed = configure_target(http, data_dir=target_dir, episode_index=target_episode)

    assert confirmed["data_dir"] == "/place/scene1"
    assert confirmed["episode"] == 7
    assert http.config_posts == 1


def test_event_context_rejects_events_received_in_an_older_state():
    rules = load_rules_module()

    assert rules.event_context_is_current("grasp", "recording", "grasp", "recording") is True
    assert rules.event_context_is_current("grasp", "reviewing", "place", "idle") is False
    assert rules.event_context_is_current("place", "idle", "place", "recording") is False


def test_place_start_latch_is_scoped_to_grasp_review_and_released_only_on_accept():
    rules = load_rules_module()

    assert rules.should_latch_place_start(
        "place_start", "grasp", "grasp", "saving"
    ) is True
    assert rules.should_latch_place_start(
        "place_start", "grasp", "grasp", "reviewing"
    ) is True
    assert rules.should_latch_place_start(
        "place_start", "grasp", "grasp", "recording"
    ) is True
    assert rules.should_latch_place_start(
        "place_start", "grasp", "place", "reviewing"
    ) is False
    assert rules.should_latch_place_start(
        "end", "grasp", "grasp", "reviewing"
    ) is False
    assert rules.should_latch_place_start(
        "place_start", "place", "place", "reviewing"
    ) is False
    assert rules.should_release_latched_place_start(
        True,
        "grasp",
        "accepted",
        "place",
    ) is True
    assert rules.should_release_latched_place_start(
        True,
        "grasp",
        "discarded",
        "grasp",
    ) is False
    assert rules.should_release_latched_place_start(
        True,
        "place",
        "accepted",
        "grasp",
    ) is False
    assert rules.should_release_latched_place_start(
        False,
        "grasp",
        "accepted",
        "place",
    ) is False


def test_initial_status_read_retries_without_falling_back_to_episode_zero():
    wait_for_initial_status = load_staged_bridge_function("wait_for_initial_status")

    class FlakyHttp:
        def __init__(self):
            self.calls = 0

        def status(self):
            self.calls += 1
            if self.calls == 1:
                return {"state": "starting", "capture_running": False}
            if self.calls == 2:
                return {"episode": -1, "capture_running": False}
            if self.calls == 3:
                raise ConnectionError("not ready")
            return {"episode": 17, "capture_running": False}

    http = FlakyHttp()
    status = wait_for_initial_status(http, timeout=0.2, poll_interval=0.001)

    assert status["episode"] == 17
    assert http.calls == 4


def test_transition_event_drain_waits_for_event_loop_and_discards_delayed_event():
    discard_events = load_staged_bridge_function("discard_two_phase_transition_events")

    class FakeNode:
        def __init__(self):
            self.events = queue.SimpleQueue()

    node = FakeNode()
    delayed_event = ("place_start", "grasp", "reviewing")
    timer = threading.Timer(0.005, node.events.put, args=(delayed_event,))
    timer.start()
    try:
        discarded = discard_events(node, quiet_period=0.02)
    finally:
        timer.join()

    assert discarded == [delayed_event]
    assert node.events.empty()


def test_publish_during_ros_shutdown_is_ignored_but_live_errors_propagate():
    ros_publish_error_is_shutdown = load_staged_bridge_function("ros_publish_error_is_shutdown")

    ros_publish_error_is_shutdown.__globals__["rclpy"] = SimpleNamespace(ok=lambda: False)
    assert ros_publish_error_is_shutdown() is True

    ros_publish_error_is_shutdown.__globals__["rclpy"] = SimpleNamespace(ok=lambda: True)
    assert ros_publish_error_is_shutdown() is False

    text = STAGED.read_text(encoding="utf-8")
    bridge_block = text.split("class StateMachineBridge(Node):", 1)[1].split(
        "def wait_for_status",
        1,
    )[0]
    shutdown_guard = (
        "except Exception:\n"
        "                if ros_publish_error_is_shutdown():\n"
        "                    return\n"
        "                raise"
    )
    assert bridge_block.count(shutdown_guard) == 2


def test_collection_config_payload_can_switch_paired_directory_and_episode():
    collection_config_payload = load_staged_bridge_function("collection_config_payload")
    status = {
        "data_dir": "/home/agilex/data/stage2_twohand/20260803_scene1",
        "episode": 8,
        "with_base": True,
        "auto_save_on_preset": False,
        "run_convert": True,
        "run_qc": True,
        "run_lerobot": True,
        "background_processing": True,
        "preset_info_json": "/tmp/preset.json",
        "lerobot_target_dir": "/home/agilex/data/stage2_twohand/20260803_scene1/lerobot",
        "lerobot_dataset_name": "20260803_scene1_aloha_mobile",
    }

    payload = collection_config_payload(
        status,
        False,
        data_dir="/home/agilex/data/place/20260803_scene1",
        episode_index=7,
    )

    assert payload["data_dir"] == "/home/agilex/data/place/20260803_scene1"
    assert payload["episode_index"] == "7"
    assert payload["staged_capture"] is False
    assert payload["lerobot_target_dir"] == "/home/agilex/data/place/20260803_scene1/lerobot"
    assert payload["lerobot_dataset_name"] == "20260803_scene1_aloha_mobile"


def test_collection_config_payload_preserves_custom_lerobot_destination():
    collection_config_payload = load_staged_bridge_function("collection_config_payload")
    status = {
        "data_dir": "/grasp/scene1",
        "episode": 3,
        "lerobot_target_dir": "/datasets/custom-output",
        "lerobot_dataset_name": "custom_repo",
    }

    payload = collection_config_payload(
        status,
        False,
        data_dir="/place/scene1",
        episode_index=2,
    )

    assert payload["lerobot_target_dir"] == "/datasets/custom-output"
    assert payload["lerobot_dataset_name"] == "custom_repo"


def test_staged_bridge_wires_opt_in_grasp_place_topics_and_flow():
    text = STAGED.read_text(encoding="utf-8")

    required = [
        'GRASP_PLACE_COLLECTION_ENABLE="${GRASP_PLACE_COLLECTION_ENABLE:-0}"',
        'GRASP_PLACE_DATA_ROOT="${GRASP_PLACE_DATA_ROOT:-/home/agilex/data/place}"',
        'GRASP_PLACE_START_TOPIC="${GRASP_PLACE_START_TOPIC:-/state_place/start}"',
        'GRASP_PLACE_GRASP_END_TOPIC="${GRASP_PLACE_GRASP_END_TOPIC:-/state_machine/grasping/end}"',
        "derive_place_data_dir",
        "two_phase_event_action",
        "review_outcome",
        "capture_target_after_review",
        "self.create_subscription(Bool, grasp_end_topic, self._on_grasp_end, 10)",
        "self.create_subscription(Bool, place_start_topic, self._on_place_start, 10)",
        'self._put_event("grasp_end")',
        'self._put_event("place_start")',
        "if not two_phase_enabled:",
        "configure_collection_for_target(",
        "wait_for_initial_status(",
        "def begin_two_phase_save(",
        'phase = "reviewing"',
        "pending_review_episode",
        "event_context_is_current",
        "node.set_event_context(",
        "SingleThreadedExecutor",
        "ros_spin_thread",
        "discard_two_phase_transition_events(",
        "review_transition_or_cached",
        'capture_phase = "grasp"',
        'target_dir = grasp_data_dir if capture_phase == "grasp" else place_data_dir',
    ]
    for marker in required:
        assert marker in text

    two_phase_block = text.split("            if two_phase_enabled:", 1)[1].split(
        '            if event == "start":',
        1,
    )[0]
    assert "save_current_episode(" not in two_phase_block


def test_recording_context_is_armed_before_ready_true_is_published():
    text = STAGED.read_text(encoding="utf-8")
    start_recording = text.split("    def start_recording(", 1)[1].split(
        "\n    def mark_stage_boundary",
        1,
    )[0]

    recording_before_ready = (
        'phase = "recording"\n'
        "            sync_event_context()\n"
        "            node.publish_ready(True)"
    )
    final_recording_before_ready = (
        'phase = "recording"\n'
        "        sync_event_context()\n"
        "        node.publish_ready(True)"
    )
    assert recording_before_ready in start_recording
    assert final_recording_before_ready in start_recording


def test_bridge_latches_review_place_start_and_requeues_it_after_accept():
    text = STAGED.read_text(encoding="utf-8")

    assert "pending_place_start = False" in text
    assert "should_latch_place_start(" in text
    assert "should_release_latched_place_start(" in text
    assert (
        "event_name, received_capture_phase, received_runtime_phase = queued_review_event"
        in text
    )
    assert (
        "event_name, received_capture_phase, received_runtime_phase = discarded_event"
        in text
    )
    assert 'node.events.put(("place_start", capture_phase, phase))' in text


def test_staged_bridge_heredoc_python_blocks_compile():
    lines = STAGED.read_text(encoding="utf-8").splitlines()
    blocks: list[tuple[int, str]] = []
    current: list[str] = []
    in_block = False
    start_line = 0
    for lineno, line in enumerate(lines, start=1):
        if not in_block and "<<'PY'" in line:
            in_block = True
            start_line = lineno + 1
            current = []
            continue
        if in_block and line == "PY":
            blocks.append((start_line, "\n".join(current) + "\n"))
            in_block = False
            continue
        if in_block:
            current.append(line)

    assert not in_block
    assert blocks
    for index, (line, code) in enumerate(blocks, start=1):
        compile(code, f"{STAGED.name}:heredoc_{index}_line_{line}", "exec")

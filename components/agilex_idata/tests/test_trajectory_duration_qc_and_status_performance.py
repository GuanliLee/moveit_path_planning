from __future__ import annotations

from pathlib import Path
import importlib.util
import json
import sys
import threading

import pytest


REPO_ROOT = Path(__file__).resolve().parents[1]
PIPELINE_ROOT = REPO_ROOT / "scripts" / "embodied_data_pipeline-main"
PIPELINE_WEB_APP = PIPELINE_ROOT / "scripts" / "pipeline_web_app.py"
RUN_QUALITY_PIPELINE = PIPELINE_ROOT / "scripts" / "run_quality_pipeline.py"
ALOHA_PROFILE = PIPELINE_ROOT / "robot_profiles" / "aloha.yaml"
FOUR_CAMERA_PROFILE = PIPELINE_ROOT / "robot_profiles" / "aloha_four_camera.yaml"

if str(PIPELINE_ROOT) not in sys.path:
    sys.path.insert(0, str(PIPELINE_ROOT))

from quality_pipeline.episode_io import EpisodeData, StateFrame
from quality_pipeline.profiles import load_profile
from quality_pipeline.qc import run_quality_checks


def load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def load_selectable_web_app(monkeypatch, tmp_path: Path, name: str):
    monkeypatch.setenv("PIPELINE_CAMERA_VARIANT_SELECTABLE", "1")
    monkeypatch.setenv("PIPELINE_CAMERA_COUNT", "4")
    monkeypatch.setenv("PIPELINE_HEAD_CAMERA_SOURCE", "front")
    monkeypatch.setenv("PIPELINE_DEFAULT_PROFILE", str(FOUR_CAMERA_PROFILE))
    monkeypatch.setenv("PIPELINE_CAMERA_LAYOUT", "four_camera")
    monkeypatch.setenv("WEB_LOG_ROOT", str(tmp_path / "web_logs"))
    return load_module(PIPELINE_WEB_APP, name)


def make_episode(duration: float) -> EpisodeData:
    return EpisodeData(
        root=Path("/tmp/duration-test"),
        episode_id="episode0",
        meta={},
        state_frames=[
            StateFrame(frame_idx=0, timestamp=0.0, state=[0.0] * 21),
            StateFrame(frame_idx=1, timestamp=duration, state=[0.0] * 21),
        ],
        actions=[[0.0] * 18, [0.0] * 18],
        camera_counts={
            "head_color": 2,
            "hand_left_color": 2,
            "hand_right_color": 2,
        },
    )


@pytest.mark.parametrize("profile_path", [ALOHA_PROFILE, FOUR_CAMERA_PROFILE])
@pytest.mark.parametrize(
    ("duration", "status", "reason"),
    [
        (3.9, "warn", "too_short"),
        (4.0, "pass", ""),
        (10.0, "pass", ""),
        (10.1, "warn", "too_long"),
    ],
)
def test_aloha_duration_range(
    profile_path: Path,
    duration: float,
    status: str,
    reason: str,
) -> None:
    profile = load_profile(profile_path)
    episode = make_episode(duration)
    if profile_path == FOUR_CAMERA_PROFILE:
        episode = EpisodeData(
            root=episode.root,
            episode_id=episode.episode_id,
            meta=episode.meta,
            state_frames=episode.state_frames,
            actions=episode.actions,
            camera_counts={**episode.camera_counts, "head": 2},
        )

    report = run_quality_checks(episode, profile)
    check = next(item for item in report["checks"] if item["name"] == "duration")

    assert check["status"] == status
    assert check["score_delta"] == (-5.0 if reason else 0.0)
    assert check["detail"] == {
        "duration_sec": duration,
        "min_duration_sec": 4.0,
        "max_duration_sec": 10.0,
        "reason": reason,
    }


def test_duration_warning_text_distinguishes_short_and_long() -> None:
    app = load_module(PIPELINE_WEB_APP, "duration_warning_web_app")

    short_text = app.qc_warning_text(
        {
            "name": "duration",
            "status": "warn",
            "detail": {
                "duration_sec": 3.5,
                "min_duration_sec": 4.0,
                "max_duration_sec": 10.0,
                "reason": "too_short",
            },
        }
    )
    long_text = app.qc_warning_text(
        {
            "name": "duration",
            "status": "warn",
            "detail": {
                "duration_sec": 10.5,
                "min_duration_sec": 4.0,
                "max_duration_sec": 10.0,
                "reason": "too_long",
            },
        }
    )

    assert short_text == "轨迹过短: 3.50s < 最短 4.00s"
    assert long_text == "轨迹过长: 10.50s > 最长 10.00s"


def test_markdown_duration_detail_shows_range_and_reason() -> None:
    pipeline = load_module(RUN_QUALITY_PIPELINE, "duration_report_pipeline")

    short_detail = pipeline._check_detail(
        "duration",
        {
            "duration_sec": 3.5,
            "min_duration_sec": 4.0,
            "max_duration_sec": 10.0,
            "reason": "too_short",
        },
    )
    long_detail = pipeline._check_detail(
        "duration",
        {
            "duration_sec": 10.5,
            "min_duration_sec": 4.0,
            "max_duration_sec": 10.0,
            "reason": "too_long",
        },
    )

    assert short_detail == "时长 3.5s；允许范围 4.0-10.0s；轨迹过短"
    assert long_detail == "时长 10.5s；允许范围 4.0-10.0s；轨迹过长"


def test_quality_check_items_use_effective_profile_thresholds() -> None:
    app = load_module(PIPELINE_WEB_APP, "quality_check_items_web_app")
    cfg = {
        "profile": FOUR_CAMERA_PROFILE,
        "aloha_include_base_action": True,
    }

    items = app.quality_check_items(cfg)

    assert [item["name"] for item in items] == [
        "State 维度",
        "Action 维度",
        "有限数值",
        "时间戳连续性",
        "FPS",
        "相机完整性",
        "相机修复帧",
        "轨迹时长",
        "运动稳定性",
        "Action 静止帧",
        "夹爪活动",
    ]
    assert next(item for item in items if item["name"] == "相机完整性")["criterion"] == (
        "4 路必需视角，单路缺帧率不超过 2.0%"
    )
    assert next(item for item in items if item["name"] == "轨迹时长")["criterion"] == (
        "4.0-10.0 秒（超出范围提示）"
    )


def test_qc_page_contains_visible_quality_check_catalog() -> None:
    app = load_module(PIPELINE_WEB_APP, "quality_check_catalog_web_app")

    assert 'id="qualityCheckItems"' in app.HTML
    assert "function renderQualityCheckItems(items)" in app.HTML
    assert "renderQualityCheckItems(data.quality_check_items)" in app.HTML


def test_markdown_fallback_reads_new_trajectory_duration_label(
    tmp_path: Path,
) -> None:
    app = load_module(PIPELINE_WEB_APP, "trajectory_duration_markdown_web_app")
    table_path = tmp_path / "final_report_table.md"
    table_path.write_text(
        "\n".join(
            [
                "## 数据质量检查",
                "",
                "| 检查项 | 是否通过 | 相关细节 |",
                "|---|---|---|",
                "| 轨迹时长 | 警告 | 时长 10.5s；允许范围 4.0-10.0s；轨迹过长 |",
            ]
        ),
        encoding="utf-8",
    )

    record = app.qc_overview_record_from_final_table("episode0", table_path)

    assert record is not None
    assert record["duration_sec"] == 10.5


def test_hdf5_is_not_opened_when_sidecar_has_frame_metadata(
    monkeypatch,
    tmp_path: Path,
) -> None:
    app = load_selectable_web_app(
        monkeypatch,
        tmp_path,
        "metadata_first_hdf5_web_app",
    )
    episode_dir = tmp_path / "episodes" / "episode0"
    h5_path = episode_dir / "states" / "aligned_joints.h5"
    h5_path.parent.mkdir(parents=True)
    h5_path.touch()
    meta_path = episode_dir / "meta" / "episode_meta.json"
    meta_path.parent.mkdir()
    meta_path.write_text(
        json.dumps({"frame_count": 120, "state_fps": 30.0}),
        encoding="utf-8",
    )

    def fail_if_called(path: Path):
        raise AssertionError(f"unexpected HDF5 read: {path}")

    monkeypatch.setattr(app, "estimate_hdf5_frame_info", fail_if_called)

    episodes = app.discover_hdf5_episodes(tmp_path / "episodes")

    assert episodes[0]["frame_count"] == 120
    assert episodes[0]["fps"] == 30.0


@pytest.mark.parametrize(
    "metadata",
    [
        {"frame_count": 0, "state_fps": 30.0},
        {"frame_count": -1, "state_fps": 30.0},
        {"frame_count": 120, "state_fps": 0.0},
        {"frame_count": 120, "state_fps": -1.0},
        {"frame_count": 120, "state_fps": float("inf")},
    ],
)
def test_invalid_sidecar_frame_metadata_falls_back_to_hdf5(
    monkeypatch,
    tmp_path: Path,
    metadata: dict[str, float],
) -> None:
    app = load_selectable_web_app(
        monkeypatch,
        tmp_path,
        f"invalid_metadata_hdf5_web_app_{len(sys.modules)}",
    )
    episode_dir = tmp_path / "episodes" / "episode0"
    h5_path = episode_dir / "states" / "aligned_joints.h5"
    h5_path.parent.mkdir(parents=True)
    h5_path.touch()
    meta_path = episode_dir / "meta" / "episode_meta.json"
    meta_path.parent.mkdir()
    meta_path.write_text(json.dumps(metadata), encoding="utf-8")
    calls: list[Path] = []

    def estimate(path: Path):
        calls.append(path)
        return 240, 60.0

    monkeypatch.setattr(app, "estimate_hdf5_frame_info", estimate)

    episodes = app.discover_hdf5_episodes(tmp_path / "episodes")

    assert calls == [h5_path]
    assert episodes[0]["frame_count"] > 0
    assert episodes[0]["fps"] > 0


def test_one_status_request_reads_each_qc_report_once(
    monkeypatch,
    tmp_path: Path,
) -> None:
    app = load_selectable_web_app(
        monkeypatch,
        tmp_path,
        "single_qc_report_read_web_app",
    )
    hdf5_root = tmp_path / "hdf5" / "demo"
    episode_dir = hdf5_root / "episode0"
    h5_path = episode_dir / "states" / "aligned_joints.h5"
    h5_path.parent.mkdir(parents=True)
    h5_path.touch()
    meta_path = episode_dir / "meta" / "episode_meta.json"
    meta_path.parent.mkdir()
    meta_path.write_text(
        json.dumps({"frame_count": 120, "state_fps": 30.0}),
        encoding="utf-8",
    )

    qc_root = tmp_path / "qc"
    report_dir = qc_root / "demo_20260728_120000"
    episode_report_dir = report_dir / "episode0"
    episode_report_dir.mkdir(parents=True)
    report_path = episode_report_dir / "qc_report.json"
    report_path.write_text(
        json.dumps(
            {
                "accepted": True,
                "quality_score": 100.0,
                "checks": [
                    {
                        "name": "duration",
                        "status": "pass",
                        "score_delta": 0.0,
                        "detail": {
                            "duration_sec": 5.0,
                            "min_duration_sec": 4.0,
                            "max_duration_sec": 10.0,
                            "reason": "",
                        },
                    }
                ],
                "summary": {"frames": 120, "duration_sec": 5.0, "fps": 30.0},
            }
        ),
        encoding="utf-8",
    )
    (report_dir / "batch_summary.json").write_text(
        json.dumps(
            {
                "episodes": [
                    {
                        "episode_id": "episode0",
                        "ok": True,
                        "accepted": True,
                        "fps": 30.0,
                        "delete_status": "成功",
                        "output": str(episode_report_dir),
                    }
                ]
            }
        ),
        encoding="utf-8",
    )

    reads = 0
    original_read_text = Path.read_text

    def counted_read_text(path: Path, *args, **kwargs):
        nonlocal reads
        if path == report_path:
            reads += 1
        return original_read_text(path, *args, **kwargs)

    monkeypatch.setattr(Path, "read_text", counted_read_text)

    status = app.dataset_status(
        {
            "hdf5_root": str(hdf5_root),
            "qc_root": str(qc_root),
            "dataset_name": "demo",
            "camera_count": 4,
            "head_camera_source": "front",
            "robot_type": "aloha",
        }
    )

    assert status["qc_report_dir"] == str(report_dir)
    assert len(status["episodes"]) == 1
    assert reads == 1


def test_status_cache_isolated_by_camera_mode_and_can_be_invalidated(
    monkeypatch,
    tmp_path: Path,
) -> None:
    app = load_selectable_web_app(
        monkeypatch,
        tmp_path,
        "camera_mode_status_cache_web_app",
    )
    calls: list[str] = []

    def discover(root: Path):
        calls.append(str(root))
        return []

    monkeypatch.setattr(app, "discover_hdf5_episodes", discover)
    base_payload = {
        "data_root": str(tmp_path / "data"),
        "dataset_name": "demo",
        "robot_type": "aloha",
    }
    four_payload = {
        **base_payload,
        "camera_count": 4,
        "head_camera_source": "front",
    }
    three_payload = {
        **base_payload,
        "camera_count": 3,
        "head_camera_source": "global",
    }

    first_four = app.dataset_status(four_payload)
    second_four = app.dataset_status(four_payload)
    three = app.dataset_status(three_payload)

    assert second_four is first_four
    assert first_four["config"]["camera_count"] == 4
    assert three["config"]["camera_count"] == 3
    assert len(calls) == 2

    gpu_payload = {**four_payload, "gpu_device": "1"}
    gpu_status = app.dataset_status(gpu_payload)

    assert gpu_status["config"]["gpu_device"] == "1"
    assert len(calls) == 3

    app.invalidate_status_cache()
    refreshed_four = app.dataset_status(four_payload)

    assert refreshed_four is not first_four
    assert len(calls) == 4


def test_invalidation_during_status_build_prevents_stale_cache_write(
    monkeypatch,
    tmp_path: Path,
) -> None:
    app = load_selectable_web_app(
        monkeypatch,
        tmp_path,
        "concurrent_status_cache_invalidation_web_app",
    )
    started = threading.Event()
    release = threading.Event()
    calls = 0
    original_build = app._build_dataset_status

    def blocking_build(cfg, payload, report_dir):
        nonlocal calls
        calls += 1
        if calls == 1:
            started.set()
            assert release.wait(timeout=5)
        return original_build(cfg, payload, report_dir)

    monkeypatch.setattr(app, "_build_dataset_status", blocking_build)
    payload = {
        "data_root": str(tmp_path / "data"),
        "dataset_name": "demo",
        "robot_type": "aloha",
        "camera_count": 4,
        "head_camera_source": "front",
    }
    worker = threading.Thread(target=app.dataset_status, args=(payload,))
    worker.start()
    assert started.wait(timeout=5)

    app.invalidate_status_cache()
    release.set()
    worker.join(timeout=5)
    assert not worker.is_alive()

    app.dataset_status(payload)

    assert calls == 2


def test_run_job_invalidates_status_cache_on_every_terminal_state(
    monkeypatch,
    tmp_path: Path,
) -> None:
    app = load_selectable_web_app(
        monkeypatch,
        tmp_path,
        "job_status_cache_invalidation_web_app",
    )
    invalidations = 0
    current_job: dict[str, object] = {}
    statuses_when_invalidated: list[str] = []

    def invalidate():
        nonlocal invalidations
        invalidations += 1
        statuses_when_invalidated.append(str(current_job["job"]["status"]))

    def fail(_job):
        raise RuntimeError("expected test failure")

    monkeypatch.setattr(app, "invalidate_status_cache", invalidate)
    completed_job = {"status": "queued", "stop_requested": False, "log": []}
    failed_job = {"status": "queued", "stop_requested": False, "log": []}
    stopped_job = {"status": "queued", "stop_requested": True, "log": []}

    current_job["job"] = completed_job
    app.run_job(completed_job, [lambda _job: None])
    current_job["job"] = failed_job
    app.run_job(failed_job, [fail])
    current_job["job"] = stopped_job
    app.run_job(stopped_job, [lambda _job: None])

    assert completed_job["status"] == "completed"
    assert failed_job["status"] == "failed"
    assert stopped_job["status"] == "stopped"
    assert invalidations == 3
    assert statuses_when_invalidated == ["running", "running", "running"]


def test_concurrent_joints_only_profile_writes_use_distinct_temp_files(
    monkeypatch,
    tmp_path: Path,
) -> None:
    app = load_selectable_web_app(
        monkeypatch,
        tmp_path,
        "concurrent_profile_write_web_app",
    )
    monkeypatch.setattr(app, "DATA_ROOT", tmp_path / "generated")
    barrier = threading.Barrier(2)
    original_write_text = Path.write_text

    def synchronized_write(path: Path, *args, **kwargs):
        result = original_write_text(path, *args, **kwargs)
        if path.name.endswith(".tmp"):
            barrier.wait(timeout=5)
        return result

    monkeypatch.setattr(Path, "write_text", synchronized_write)
    results: list[Path] = []
    errors: list[Exception] = []

    def write_profile() -> None:
        try:
            results.append(app.write_aloha_joints_only_profile(ALOHA_PROFILE))
        except Exception as exc:
            errors.append(exc)

    workers = [threading.Thread(target=write_profile) for _ in range(2)]
    for worker in workers:
        worker.start()
    for worker in workers:
        worker.join(timeout=5)

    assert all(not worker.is_alive() for worker in workers)
    assert errors == []
    assert len(results) == 2
    assert all(path.is_file() for path in results)


def test_dataset_scan_discovers_processed_variants_without_mcap(
    monkeypatch,
    tmp_path: Path,
) -> None:
    app = load_selectable_web_app(
        monkeypatch,
        tmp_path,
        "processed_dataset_discovery_web_app",
    )
    scan_root = tmp_path / "data"
    scene_root = scan_root / "stage2_new" / "scene2"
    dataset_name = "20260715_scene2"
    scene_root.mkdir(parents=True)
    (scene_root / "leftover.mcap").touch()
    for namespace, episode_names in (
        ("four_camera", ("episode0", "episode1")),
        ("three_camera_global", ("episode0",)),
    ):
        for episode_name in episode_names:
            h5_path = (
                scene_root
                / namespace
                / "hdf5_episodes"
                / dataset_name
                / episode_name
                / "states"
                / "aligned_joints.h5"
            )
            h5_path.parent.mkdir(parents=True)
            h5_path.touch()
        report_root = scene_root / namespace / "qc_reports"
        report_root.mkdir(parents=True)
        (scene_root / namespace / "lerobot").mkdir()

    choices = app.discover_dataset_choices(scan_root)

    assert len(choices) == 1
    choice = choices[0]
    assert choice["path"] == str(scene_root.resolve())
    assert choice["has_mcap"] is False
    assert choice["episode_count"] == 2
    assert set(choice["processed_variants"]) == {
        "four_camera",
        "three_camera_global",
    }
    four = choice["processed_variants"]["four_camera"]
    assert four == [
        {
            "dataset_name": dataset_name,
            "hdf5_root": str(
                (scene_root / "four_camera" / "hdf5_episodes" / dataset_name).resolve()
            ),
            "qc_root": str((scene_root / "four_camera" / "qc_reports").resolve()),
            "lerobot_root": str((scene_root / "four_camera" / "lerobot").resolve()),
            "episode_count": 2,
        }
    ]


def test_processed_only_source_choice_applies_camera_variant_paths() -> None:
    app = load_module(PIPELINE_WEB_APP, "processed_dataset_browser_web_app")
    apply_source = app.HTML.split(
        "async function applySourceDatasetChoice()",
        1,
    )[1].split("function missingRequiredFields", 1)[0]

    assert "function applyProcessedDatasetVariant(choice)" in app.HTML
    assert 'document.getElementById("mcapPath").value = "";' in app.HTML
    assert 'document.getElementById("datasetName").value = processed.dataset_name || "";' in app.HTML
    assert 'document.getElementById("hdf5Root").value = processed.hdf5_root || "";' in app.HTML
    assert 'document.getElementById("qcRoot").value = processed.qc_root || "";' in app.HTML
    assert 'document.getElementById("lerobotRoot").value = processed.lerobot_root || "";' in app.HTML
    assert "applyProcessedDatasetVariant(selected)" in app.HTML
    assert "applyProcessedDatasetVariant(selectedSourceDatasetChoice())" in app.HTML
    assert 'syncSourceDatasetSelectToPath(inputMcap, inputHdf5);' in app.HTML
    assert apply_source.index("if (applyProcessedDatasetVariant(selected))") < apply_source.index(
        'for (const id of ["datasetName", "hdf5Root", "qcRoot", "lerobotRoot", "repoId"])'
    )
    assert "function clearDatasetStatusViews(message)" in app.HTML
    assert "clearDatasetStatusViews(message);" in app.HTML
    assert 'document.getElementById("episodeRows").innerHTML = `<tr><td colspan="11">暂无 episode</td></tr>`;' in app.HTML
    assert 'document.getElementById("replayFrame").removeAttribute("src");' in app.HTML
    assert 'id="lerobotReplayFrame"' not in app.HTML


def test_camera_switch_ignores_stale_status_responses() -> None:
    app = load_module(PIPELINE_WEB_APP, "stale_status_response_web_app")

    assert "let statusAbortController = null;" in app.HTML
    assert "let latestStatusRequestId = 0;" in app.HTML
    assert "statusAbortController.abort();" in app.HTML
    assert 'postJson("/api/status", payload(), {signal: controller.signal})' in app.HTML
    assert "if (requestId !== latestStatusRequestId) return;" in app.HTML
    assert 'if (err?.name === "AbortError") return;' in app.HTML

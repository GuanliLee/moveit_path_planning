# Trajectory Duration QC and Status Performance Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Detect ALOHA trajectories shorter than 4 seconds or longer than 10 seconds, expose the result throughout QC reports and the web UI, and make three/four-camera status switching substantially faster.

**Architecture:** Keep duration thresholds in the ALOHA profiles and consume them through the existing QC configuration path. Build the page's QC item catalog from the effective profile. Remove unconditional HDF5 reads, share parsed report JSON within a request, cache complete status responses briefly by resolved configuration, and prevent stale frontend responses from winning a camera-mode switch.

**Tech Stack:** Python 3, pytest, h5py, YAML profiles, `ThreadingHTTPServer`, embedded HTML/CSS/JavaScript.

## Global Constraints

- ALOHA duration range is inclusive: `4.0 <= duration_sec <= 10.0`.
- Out-of-range duration is a warning with a `-5.0` score delta, not a hard failure.
- G2 duration behavior remains unchanged.
- Status cache TTL is 5 seconds and holds at most 8 isolated configuration entries.
- Existing user worktree deletions and unrelated changes must remain untouched.

---

### Task 1: Core trajectory duration rule

**Files:**
- Create: `tests/test_trajectory_duration_qc_and_status_performance.py`
- Modify: `scripts/embodied_data_pipeline-main/quality_pipeline/qc.py`
- Modify: `scripts/embodied_data_pipeline-main/robot_profiles/aloha.yaml`
- Modify: `scripts/embodied_data_pipeline-main/robot_profiles/aloha_four_camera.yaml`

**Interfaces:**
- Consumes: `EpisodeData.duration_sec`, `RobotProfile.processing`
- Produces: duration check detail keys `duration_sec`, `min_duration_sec`, `max_duration_sec`, `reason`

- [ ] **Step 1: Write the failing duration tests**

```python
from pathlib import Path

from quality_pipeline.episode_io import EpisodeData, StateFrame
from quality_pipeline.profiles import load_profile
from quality_pipeline.qc import run_quality_checks

ALOHA_PROFILE = PIPELINE_ROOT / "robot_profiles" / "aloha.yaml"


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


@pytest.mark.parametrize(
    ("duration", "status", "reason"),
    [(3.9, "warn", "too_short"), (4.0, "pass", ""), (10.0, "pass", ""), (10.1, "warn", "too_long")],
)
def test_aloha_duration_range(duration, status, reason):
    report = run_quality_checks(make_episode(duration), load_profile(ALOHA_PROFILE))
    check = next(item for item in report["checks"] if item["name"] == "duration")
    assert check["status"] == status
    assert check["detail"]["reason"] == reason
    assert check["detail"]["min_duration_sec"] == 4.0
    assert check["detail"]["max_duration_sec"] == 10.0
```

- [ ] **Step 2: Run the targeted test and confirm it fails because max duration/reason are absent**

Run: `pytest -q tests/test_trajectory_duration_qc_and_status_performance.py -k duration`

- [ ] **Step 3: Add ALOHA profile thresholds and implement range checking**

```python
def _check_duration(episode: EpisodeData, config: dict[str, Any]) -> QualityCheck:
    duration = episode.duration_sec
    minimum = float(config["min_duration_sec"])
    maximum = config.get("max_duration_sec")
    maximum_value = float(maximum) if maximum is not None else None
    reason = "too_short" if duration < minimum else "too_long" if maximum_value is not None and duration > maximum_value else ""
    status = "warn" if reason else "pass"
    return QualityCheck(
        "duration",
        status,
        -5.0 if reason else 0.0,
        {
            "duration_sec": round(duration, 4),
            "min_duration_sec": minimum,
            "max_duration_sec": maximum_value,
            "reason": reason,
        },
    )
```

- [ ] **Step 4: Run the targeted tests and confirm they pass**

Run: `pytest -q tests/test_trajectory_duration_qc_and_status_performance.py -k duration`

### Task 2: Report wording and visible QC catalog

**Files:**
- Modify: `scripts/embodied_data_pipeline-main/scripts/run_quality_pipeline.py`
- Modify: `scripts/embodied_data_pipeline-main/scripts/pipeline_web_app.py`
- Test: `tests/test_trajectory_duration_qc_and_status_performance.py`

**Interfaces:**
- Consumes: duration check detail and effective profile
- Produces: Chinese short/long warning text and `quality_check_items` in `/api/status`

- [ ] **Step 1: Write failing tests for report text and the QC item payload/UI container**

```python
assert app.qc_warning_text(short_check) == "轨迹过短: 3.50s < 最短 4.00s"
assert app.qc_warning_text(long_check) == "轨迹过长: 10.50s > 最长 10.00s"
assert any(item["name"] == "轨迹时长" and "4.0-10.0 秒" in item["criterion"] for item in app.quality_check_items(cfg))
assert 'id="qualityCheckItems"' in app.HTML
assert "renderQualityCheckItems(data.quality_check_items" in app.HTML
```

- [ ] **Step 2: Run the tests and confirm the missing text/catalog failures**

Run: `pytest -q tests/test_trajectory_duration_qc_and_status_performance.py -k 'warning or quality_check'`

- [ ] **Step 3: Implement report wording, backend catalog, and compact UI grid**

The catalog contains state/action dimensions, finite values, timestamp continuity, FPS, camera completeness, camera repair frames, trajectory duration, motion stability, action stationary frames, and gripper activity. Threshold text is populated from the effective profile configuration.

- [ ] **Step 4: Run the report and UI tests**

Run: `pytest -q tests/test_trajectory_duration_qc_and_status_performance.py -k 'warning or quality_check'`

### Task 3: Status loading optimization and cache

**Files:**
- Modify: `scripts/embodied_data_pipeline-main/scripts/pipeline_web_app.py`
- Test: `tests/test_trajectory_duration_qc_and_status_performance.py`

**Interfaces:**
- Produces: `load_qc_report(path, cache)`, `invalidate_status_cache()`, configuration-isolated cached `dataset_status(payload)`

- [ ] **Step 1: Write failing tests**

```python
def test_hdf5_is_not_opened_when_sidecar_has_frame_metadata(monkeypatch, tmp_path):
    episode_dir = tmp_path / "episode0"
    h5_path = episode_dir / "states" / "aligned_joints.h5"
    h5_path.parent.mkdir(parents=True)
    h5_path.touch()
    meta_path = episode_dir / "meta" / "episode_meta.json"
    meta_path.parent.mkdir()
    meta_path.write_text('{"frame_count": 120, "state_fps": 30.0}', encoding="utf-8")

    def fail_if_called(path):
        raise AssertionError(f"unexpected HDF5 read: {path}")

    monkeypatch.setattr(app, "estimate_hdf5_frame_info", fail_if_called)
    assert app.discover_hdf5_episodes(tmp_path)[0]["frame_count"] == 120

def test_report_json_is_loaded_once_per_status_request(monkeypatch, tmp_path):
    report_path = tmp_path / "episode0" / "qc_report.json"
    report_path.parent.mkdir()
    report_path.write_text('{"checks": [], "summary": {}}', encoding="utf-8")
    reads = 0
    original = Path.read_text

    def counted_read(path, *args, **kwargs):
        nonlocal reads
        if path == report_path:
            reads += 1
        return original(path, *args, **kwargs)

    monkeypatch.setattr(Path, "read_text", counted_read)
    report_cache = {}
    app.qc_warning_summary_from_output(report_path.parent, report_cache)
    app.qc_overview_record_from_json("episode0", report_path, report_cache)
    assert reads == 1

def test_status_cache_isolated_by_camera_mode_and_invalidated(monkeypatch):
    calls = []
    monkeypatch.setattr(
        app,
        "_build_dataset_status",
        lambda cfg, payload, report_dir: (
            calls.append(cfg["camera_count"]) or {"config": app.stringify_config(cfg)}
        ),
    )
    four_result = app.dataset_status(four_payload)
    assert app.dataset_status(four_payload) is four_result
    three_result = app.dataset_status(three_payload)
    assert three_result["config"]["camera_count"] == 3
    assert calls == [4, 3]
    app.invalidate_status_cache()
    assert app.dataset_status(four_payload) is not four_result
```

- [ ] **Step 2: Run tests and verify unconditional HDF5/report reads and missing cache fail**

Run: `pytest -q tests/test_trajectory_duration_qc_and_status_performance.py -k 'hdf5 or report_json or status_cache'`

- [ ] **Step 3: Implement metadata-first discovery and request-local report reuse**

Only call `estimate_hdf5_frame_info()` for missing `frame_count` or invalid/missing FPS. Pass one report cache through warning extraction and overview construction.

- [ ] **Step 4: Implement bounded five-second status cache**

Cache by resolved HDF5/QC/LeRobot paths, explicit report, dataset, profile, camera layout/count/head source, and relevant options. Include root/report modification signatures and clear the cache after state-changing HTTP actions and job completion.

- [ ] **Step 5: Run the optimization tests**

Run: `pytest -q tests/test_trajectory_duration_qc_and_status_performance.py -k 'hdf5 or report_json or status_cache'`

### Task 4: Frontend race prevention

**Files:**
- Modify: `scripts/embodied_data_pipeline-main/scripts/pipeline_web_app.py`
- Test: `tests/test_trajectory_duration_qc_and_status_performance.py`

**Interfaces:**
- Produces: one active status `AbortController` and monotonically increasing request sequence

- [ ] **Step 1: Write a failing HTML contract test**

```python
assert "statusAbortController.abort()" in app.HTML
assert "if (requestId !== latestStatusRequestId) return;" in app.HTML
```

- [ ] **Step 2: Run the frontend contract test and confirm failure**

Run: `pytest -q tests/test_trajectory_duration_qc_and_status_performance.py -k stale_status`

- [ ] **Step 3: Add abort and stale-response protection to `refreshStatus()`**

Aborted requests are silently ignored; genuine request errors continue to the existing log handling.

- [ ] **Step 4: Run the frontend contract test**

Run: `pytest -q tests/test_trajectory_duration_qc_and_status_performance.py -k stale_status`

### Task 5: Regression, benchmark, documentation, and restart

**Files:**
- Modify: `scripts/embodied_data_pipeline-main/README.md`

- [ ] **Step 1: Document the duration rule, QC item panel, and status optimization**

- [ ] **Step 2: Run targeted and existing selectable-camera tests**

Run: `pytest -q tests/test_trajectory_duration_qc_and_status_performance.py tests/test_selectable_camera_qc.py`

- [ ] **Step 3: Run Python syntax compilation**

Run: `python -m py_compile scripts/embodied_data_pipeline-main/quality_pipeline/qc.py scripts/embodied_data_pipeline-main/scripts/run_quality_pipeline.py scripts/embodied_data_pipeline-main/scripts/pipeline_web_app.py`

- [ ] **Step 4: Benchmark real 160-episode three/four-camera status requests before restart**

Measure uncached and cached calls and verify episode counts/report directories match.

- [ ] **Step 5: Restart the existing `8001` process with its current command and environment**

Terminate PID 2437 gracefully, start the same Python entry point from `/home/caizj/agilex_idata`, and redirect logs to the existing web log root.

- [ ] **Step 6: Verify the restarted service**

Run health requests against `/`, `/api/defaults`, and real three/four-camera `/api/status`; confirm port `8001` belongs to the new process and both camera modes return the expected configuration.

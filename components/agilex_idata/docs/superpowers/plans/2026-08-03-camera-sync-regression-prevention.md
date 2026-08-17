# Camera Sync Regression Prevention Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prevent future collection from accepting a reverted head-camera timestamp driver or a camera stream with short concentrated frame-loss bursts.

**Architecture:** Keep the corrected zhuzl camera overlay as the publisher, but make collection independent of that deployment assumption by enforcing timestamp-age checks before recording. Extend recorder health with per-topic maximum arrival gaps so a short burst like scene1 episode2 cannot pass on average FPS alone, and make post-conversion QC consume those failures.

**Tech Stack:** ROS 2 Humble/rclpy, Python 3.10, Bash collection entrypoints, recorder `capture_health.json`, pytest.

## Global Constraints

- A collection must not start when any RGB camera header age exceeds 50 ms in absolute value or when the camera-to-camera median header-age spread exceeds 40 ms. This refines the design's provisional 30 ms spread using the fixed driver's measured normal spread of about 36.5 ms.
- A recorded RGB camera stream fails when its maximum recorder-arrival gap exceeds 50 ms.
- Preserve MCAP recorder `useTopicStamp=false`; record time remains an independent audit clock.
- The production `usb_cam_node_exe` path must resolve under `/home/zhuzl/workspace/robot_station/ros_packages/sensors/camera/orbbec/install/usb_cam/`.
- Existing capture-health fields remain readable; bump the schema version when adding gap fields.
- Collection entrypoints fail closed and print the exact topic, observed age/gap, and threshold.

---

### Task 1: Persist Per-Topic Maximum Recorder Gaps

**Files:**
- Modify: `ros2_ws/src/data_tools/scripts/capture_health.py`
- Modify: `ros2_ws/src/data_tools/scripts/record_mcap.py:76-106,190-205,268-280`
- Restore and modify: `tests/test_capture_health.py`

**Interfaces:**
- Consumes: `topic_max_gaps_seconds: Mapping[str, float]` gathered from recorder callback time.
- Produces: capture-health schema version 2 fields `max_gap_seconds`, `max_allowed_gap_seconds`, `gap_failed`, and camera failure latching.

- [ ] **Step 1: Write a failing burst-loss test**

```python
def test_capture_health_rejects_camera_with_short_large_gap():
    module = load_module()
    health = module.build_capture_health(
        topic_counts=camera_counts(head=600),
        topic_min_rates=camera_rates(),
        duration_seconds=20.0,
        rate_failed_topics=[],
        topic_max_gaps_seconds={"/camera_h/color/image_raw": 0.164},
        max_camera_gap_seconds=0.05,
    )
    topic = health["topics"]["/camera_h/color/image_raw"]
    assert topic["gap_failed"] is True
    assert health["camera_ok"] is False
```

- [ ] **Step 2: Run the test and verify failure**

Run: `pytest -q tests/test_capture_health.py`

Expected: FAIL because gap inputs and outputs are absent.

- [ ] **Step 3: Track maximum callback gaps and extend health schema**

Add `max_gap_seconds` to `CountInfo`, update it from the recorder callback clock used for MCAP record time, and pass the mapping to `build_capture_health`. Set `failed = coverage_failed or topic_rate_failed or gap_failed` for RGB camera topics; retain the old behavior for other topics.

- [ ] **Step 4: Run capture-health and collection regression tests**

Run: `pytest -q tests/test_capture_health.py tests/test_four_rgb_collection.py`

Expected: all tests pass, including old schema behavior and new gap rejection.

- [ ] **Step 5: Commit recorder gap detection**

```bash
git add ros2_ws/src/data_tools/scripts/capture_health.py ros2_ws/src/data_tools/scripts/record_mcap.py tests/test_capture_health.py
git commit -m "fix: reject burst camera frame loss"
```

### Task 2: Preflight Camera Timestamp-Age Validation

**Files:**
- Create: `scripts/collection/camera_timestamp_health.py`
- Modify: `scripts/collection/check_mobile_topics.sh`
- Create: `tests/test_camera_timestamp_health.py`
- Modify: `tests/test_four_rgb_collection.py`

**Interfaces:**
- Consumes: a sample list of `(topic, arrival_ns, header_ns)` for the four RGB streams.
- Produces: `evaluate_camera_timestamps(samples, max_age_ms=50.0, max_spread_ms=30.0) -> dict` and exit status 1 when invalid.

- [ ] **Step 1: Write failing healthy, reverted-driver, and cross-camera-spread tests**

```python
def test_rejects_reverted_head_timestamp():
    result = module.evaluate_camera_timestamps(
        samples_with_ages(left=3.0, front=3.0, right=3.0, head=543.0)
    )
    assert result["ok"] is False
    assert result["topics"]["/camera_h/color/image_raw/compressed"]["age_failed"] is True


def test_accepts_fixed_head_exposure_timestamp():
    result = module.evaluate_camera_timestamps(
        samples_with_ages(left=3.0, front=3.0, right=3.0, head=39.0),
        max_age_ms=50.0,
        max_spread_ms=40.0,
    )
    assert result["ok"] is True
```

- [ ] **Step 2: Run timestamp-health tests and verify failure**

Run: `pytest -q tests/test_camera_timestamp_health.py`

Expected: FAIL because the module does not exist.

- [ ] **Step 3: Implement a 3-second, 30-sample minimum ROS probe**

The CLI subscribes to four compressed RGB topics with sensor-data QoS, computes median and P90 header age, observed rate, and cross-camera median-age spread. It emits JSON plus concise Chinese failure lines and returns nonzero on missing samples or threshold violation.

- [ ] **Step 4: Invoke the probe from the existing mobile topic check**

After value-presence checks succeed, run:

```bash
python3 "${SCRIPT_DIR}/camera_timestamp_health.py" \
  --sample-seconds "${CAMERA_TIMESTAMP_SAMPLE_SECONDS:-3}" \
  --max-age-ms "${CAMERA_MAX_HEADER_AGE_MS:-50}" \
  --max-spread-ms "${CAMERA_MAX_HEADER_SPREAD_MS:-40}"
```

Use 40 ms spread because the validated fixed driver has about 36.5 ms head-versus-wrist age spread; the absolute 50 ms age limit still rejects the old bug.

- [ ] **Step 5: Run focused and shell syntax tests**

Run: `pytest -q tests/test_camera_timestamp_health.py tests/test_four_rgb_collection.py && bash -n scripts/collection/check_mobile_topics.sh`

Expected: all tests pass and Bash syntax is valid.

- [ ] **Step 6: Commit preflight timestamp validation**

```bash
git add scripts/collection/camera_timestamp_health.py scripts/collection/check_mobile_topics.sh tests/test_camera_timestamp_health.py tests/test_four_rgb_collection.py
git commit -m "fix: block collection on camera clock skew"
```

### Task 3: Deploy Recorder Changes and Verify Both Collection Entrypoints

**Files:**
- Build output: `/home/caizj/agilex_idata/ros2_ws/install/`
- Runtime evidence only: collection logs and a disposable validation dataset

**Interfaces:**
- Consumes: Tasks 1–2 and the zhuzl overlay camera process.
- Produces: installed recorder code, passing preflight for both named entrypoints, and a new capture-health v2 file.

- [ ] **Step 1: Build the affected ROS package**

Run: `colcon build --packages-select data_tools --symlink-install`

Expected: build exits 0 and installed `capture_health.py`/`record_mcap.py` resolve to the modified sources.

- [ ] **Step 2: Verify the live camera process path**

Run: `ps -eo user,pid,args | rg '[u]sb_cam_node_exe'`

Expected: the process owner is zhuzl and executable path begins `/home/zhuzl/workspace/robot_station/ros_packages/sensors/camera/orbbec/install/usb_cam/`.

- [ ] **Step 3: Run the canonical topic check under ROS domain 99**

Run: `ROS_DOMAIN_ID=99 scripts/collection/check_mobile_topics.sh`

Expected: value checks pass; head median age remains below 50 ms; the age-spread check passes under its 40 ms threshold.

- [ ] **Step 4: Verify both requested entrypoints reference the canonical check**

Run: `bash -n scripts/collection/collect_mobile_pipeline_web_staged.sh scripts/collection/collect_mobile_pipeline_qc_web_four_camera.sh && rg -n 'check_mobile_topics.sh|TOPIC_CHECK_SCRIPT' scripts/collection/collect_mobile_pipeline_web_staged.sh scripts/collection/collect_mobile_pipeline_qc_web_four_camera.sh`

Expected: both scripts are syntactically valid and route preflight through the canonical check.

- [ ] **Step 5: Record one disposable dynamic episode and verify schema v2**

Use the collection UI to record a short episode with arm motion. Confirm `capture_health.json` has version 2, all camera gaps at or below 50 ms, camera content lag within 1 frame, and no HDF5 camera count mismatch.

### Task 4: Dispose of the Three Diagnostic Episodes Safely

**Files:**
- Read-only source: `/home/agilex/data/stage2_twohand/20260803_scene1`
- Create: `/home/agilex/data/stage2_twohand/20260803_scene1/analysis/camera_validation.json`

**Interfaces:**
- Consumes: verified MCAP/HDF5 results from the diagnostic analysis.
- Produces: an explicit non-destructive disposition manifest; it does not move or delete episodes.

- [ ] **Step 1: Write the disposition manifest**

Record episode0 and episode1 as `camera_pass_dataset_qc_failed`, and episode2 as `quarantine_left_camera_startup_gaps`. Include episode2's eight gaps, 158.6 ms converted maximum gap, approximately 12 missing frames, and the independent chassis-length QC failures for all three.

- [ ] **Step 2: Validate that no episode is silently marked trainable**

Run a JSON schema/assertion check requiring every episode to have separate `camera_status` and `dataset_qc_status` fields.

Expected: episode0/1 camera pass but dataset QC fail; episode2 camera quarantine and dataset QC fail.

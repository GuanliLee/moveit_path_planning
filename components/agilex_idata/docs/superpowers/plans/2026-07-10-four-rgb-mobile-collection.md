# Four-RGB Mobile Collection Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Record, synchronize, persist, validate, and visualize `/camera_h/color/image_raw` as the fourth `head` RGB stream in both fixed-stage and staged mobile collection workflows without persisting untrusted head calibration.

**Architecture:** Upgrade the shared `aloha_mobile` data profile to the ordered `left/front/right/head` RGB contract, while keeping `configTopics` limited to the original three trustworthy cameras. Both web entry points select that profile; the shared launch records each camera driver's native compressed stream with exactly one publisher per topic; the existing YAML-driven synchronization and conversion path carries `head` into HDF5; QC and replay are extended to require/display it.

**Tech Stack:** Bash, ROS 2 Humble launch/rclpy, YAML, Python 3, HDF5/h5py, pytest.

## Global Constraints

- The new ROS image topic is exactly `/camera_h/color/image_raw`.
- The persisted camera name is exactly `head` and the HDF5 key is exactly `camera/color/head`.
- Do not subscribe to or persist `/camera_h/color/camera_info` as calibration.
- Do not create `camera/colorIntrinsic/head` or `camera/colorExtrinsic/head` when no trustworthy config exists.
- Preserve existing `left`, `front`, and `right` keys and calibration behavior.
- Do not duplicate or pad frames to hide synchronization loss.
- New episodes must fail QC when any of the four synchronized RGB streams is missing or has a mismatched frame count.
- New episodes must fail QC when a camera covers less than 80% of the full capture duration at its configured minimum rate, even if synchronization truncates all output streams to equal lengths.
- New mobile episodes without `capture_health.json` fail closed by default; only explicit `ALLOW_MISSING_CAPTURE_HEALTH=1` permits legacy conversion without it.

---

### Task 1: Define and launch the four-RGB capture contract

**Files:**
- Create: `tests/test_four_rgb_collection.py`
- Modify: `ros2_ws/src/data_tools/config/aloha_mobile_data_params.yaml`
- Modify: `ros2_ws/src/data_tools/launch/run_aloha_mobile_data_capture_to_mcap.launch.py`
- Modify: `scripts/collection/check_mobile_topics.sh`
- Modify: `scripts/collection/collect_mobile_pipeline_web_staged.sh`
- Modify: `scripts/collection/collect_mobile_pipeline_web_fixed_stage.sh`
- Modify: `tests/test_fixed_stage_collection_script.py`

**Interfaces:**
- Consumes: `/camera_h/color/image_raw` (`sensor_msgs/msg/Image`).
- Consumes: the driver's `/camera_h/color/image_raw/compressed` (`sensor_msgs/msg/CompressedImage`) and produces an MCAP channel named `/camera_h/color/image_raw`.
- Produces: the ordered YAML camera names `['left', 'front', 'right', 'head']` and topics ending in `/camera_h/color/image_raw`.

- [ ] **Step 1: Write failing capture-contract tests**

Add tests that load the canonical YAML and assert the exact four names/topics, assert that `configTopics` contains only the existing three camera-info topics, assert the launch does not contain redundant compressor nodes, assert the topic checker probes all four raw and native-compressed images, and assert both entry points select/export the canonical mobile YAML. Extend the fixed-stage stub to print `MOBILE_YAML` and assert it ends in `ros2_ws/src/data_tools/config/aloha_mobile_data_params.yaml`.

```python
def test_mobile_profile_declares_four_rgb_streams_without_fake_head_calibration():
    color = load_mobile_color_profile()
    assert color["names"] == ["left", "front", "right", "head"]
    assert color["topics"] == [
        "/camera_l/color/image_raw",
        "/camera_f/color/image_raw",
        "/camera_r/color/image_raw",
        "/camera_h/color/image_raw",
    ]
    assert color["configTopics"] == [
        "/camera_l/color/camera_info",
        "/camera_f/color/camera_info",
        "/camera_r/color/camera_info",
    ]
    assert "/camera_h/color/camera_info" not in color["configTopics"]
```

- [ ] **Step 2: Run focused tests and verify RED**

Run: `pytest -q tests/test_four_rgb_collection.py tests/test_fixed_stage_collection_script.py`

Expected: failures report missing `head`, missing `/camera_h/color/image_raw`, redundant compressor nodes, and missing `MOBILE_YAML` export.

- [ ] **Step 3: Implement the capture contract**

Update the canonical YAML camera block to:

```yaml
camera:
  color:
    checkFrameRate: 25
    names: ['left', 'front', 'right', 'head']
    parentFrames: ['camera_l_link', 'camera_f_link', 'camera_r_link', 'camera_h_link']
    topics: ['/camera_l/color/image_raw', '/camera_f/color/image_raw', '/camera_r/color/image_raw', '/camera_h/color/image_raw']
    configTopics: ['/camera_l/color/camera_info', '/camera_f/color/camera_info', '/camera_r/color/camera_info']
```

Remove the redundant camera compressor launch nodes. The deployed left/front/right/head drivers already provide reliable `/color/image_raw/compressed` publishers, and `record_mcap.py` already subscribes to those topics. A single publisher per compressed topic avoids duplicate frames.

Add this required topic probe:

```python
TopicCheck("/camera_h/color/image_raw", Image, "sensor_msgs/msg/Image", "图像数据", check_image),
```

Also require non-empty `/camera_{l,f,r,h}/color/image_raw/compressed` messages, because the recorder consumes the native compressed streams.

In both entry points, default and export `MOBILE_YAML` to `${REPO_ROOT}/ros2_ws/src/data_tools/config/aloha_mobile_data_params.yaml`; in the staged entry point pass `MOBILE_YAML` through `start_collection`'s `env` command and document the four-RGB contract in `usage`. The fixed wrapper continues delegating to the staged script.

- [ ] **Step 4: Run focused tests and verify GREEN**

Run: `pytest -q tests/test_four_rgb_collection.py tests/test_fixed_stage_collection_script.py`

Expected: all capture-contract tests pass.

- [ ] **Step 5: Commit Task 1**

```bash
git add tests/test_four_rgb_collection.py tests/test_fixed_stage_collection_script.py \
  ros2_ws/src/data_tools/config/aloha_mobile_data_params.yaml \
  ros2_ws/src/data_tools/launch/run_aloha_mobile_data_capture_to_mcap.launch.py \
  scripts/collection/check_mobile_topics.sh \
  scripts/collection/collect_mobile_pipeline_web_staged.sh \
  scripts/collection/collect_mobile_pipeline_web_fixed_stage.sh
git commit -m "feat: capture fourth mobile rgb stream"
```

### Task 2: Persist head RGB without fabricated calibration and enforce synchronization QC

**Files:**
- Modify: `tests/test_four_rgb_collection.py`
- Modify: `ros2_ws/src/data_tools/scripts/data_to_hdf5.py`
- Modify: `ros2_ws/src/data_tools/launch/run_data_sync.launch.py`
- Modify: `scripts/collection/verify_mobile_hdf5.py`
- Modify: `scripts/collection/collect_mobile_episode_web.sh`
- Create: `ros2_ws/src/data_tools/scripts/capture_health.py`
- Modify: `ros2_ws/src/data_tools/scripts/record_mcap.py`

**Interfaces:**
- Consumes: synchronized ALOHA paths in `camera/color/{left,front,right,head}/sync.txt`.
- Produces: `camera/color/head` in HDF5 with the same synchronized frame count as robot state/action.
- Produces calibration datasets only for cameras whose `config.json` exists.
- Produces QC failures for a missing/mismatched/unreadable fourth RGB stream or effective synchronized FPS below the configured threshold. Preflight also requires non-empty raw and native-compressed images for all four cameras.
- Produces `capture_health.json` with full-duration coverage and latched rate failures; HDF5 QC consumes it and enforces `MIN_FRAMES`.

- [ ] **Step 1: Write failing HDF5-format and QC tests**

Create a minimal ALOHA episode with `left` and `head` RGB sync files, a real config only for `left`, and run `data_to_hdf5.Operator.process()`. Assert:

```python
assert "camera/color/head" in root
assert "camera/colorIntrinsic/left" in root
assert "camera/colorExtrinsic/left" in root
assert "camera/colorIntrinsic/head" not in root
assert "camera/colorExtrinsic/head" not in root
```

Create a synthetic mobile HDF5 fixture with all numeric state/action keys and 25-FPS timestamps. Assert `verify_file` rejects a three-camera fixture with `missing camera/color/head` and accepts a four-camera fixture with equal frame counts.

- [ ] **Step 2: Run focused tests and verify RED**

Run: `pytest -q tests/test_four_rgb_collection.py -k 'hdf5 or qc'`

Expected: the current converter creates empty head calibration datasets and current QC accepts a file without `camera/color/head`.

- [ ] **Step 3: Implement truthful calibration persistence and four-camera QC**

In `data_to_hdf5.Operator.process`, initialize only `camera/color/<name>` before reading files. Create `camera/colorIntrinsic/<name>` and `camera/colorExtrinsic/<name>` only inside the existing `if os.path.exists(config_path)` branch after valid config is parsed.

Add a `paramsFile` launch argument to `run_data_sync.launch.py` and pass `paramsFile:="${ALOHA_YAML}"` from `collect_mobile_episode_web.sh`, so capture, MCAP conversion, synchronization, and HDF5 conversion use the same four-camera profile even before the ROS workspace is rebuilt.

Extend both mobile QC camera lists to:

```python
CAMERA_KEYS = [
    "camera/color/left",
    "camera/color/front",
    "camera/color/right",
    "camera/color/head",
]
```

The standalone verifier already checks equality with the state/action frame count, sample readability, strictly increasing timestamps, and effective FPS. Add `camera/color/head` to the embedded non-mobile fallback list in `collect_mobile_episode_web.sh` so both QC paths reflect the new format.

- [ ] **Step 4: Run focused tests and verify GREEN**

Run: `pytest -q tests/test_four_rgb_collection.py -k 'hdf5 or qc'`

Expected: the HDF5 format and four-camera QC tests pass.

- [ ] **Step 5: Commit Task 2**

```bash
git add tests/test_four_rgb_collection.py \
  ros2_ws/src/data_tools/scripts/data_to_hdf5.py \
  scripts/collection/verify_mobile_hdf5.py \
  scripts/collection/collect_mobile_episode_web.sh
git commit -m "feat: persist and validate head rgb frames"
```

### Task 3: Display the fourth stream and verify the complete workflow

**Files:**
- Modify: `tests/test_four_rgb_collection.py`
- Modify: `scripts/replay/visualize_dataset_web.py`
- Modify: `scripts/replay/visualize_lerobot.py`

**Interfaces:**
- Consumes: HDF5 `camera/color/head` and LeRobot `observation.images.head`.
- Produces: preferred camera display order `left`, `front`, `right`, `head`.

- [ ] **Step 1: Write a failing visualization test**

Load `visualize_dataset_web.py`, create a minimal HDF5 file containing four camera datasets, and assert:

```python
assert module.CAMERA_ORDER == ["left", "front", "right", "head"]
assert list(module.hdf5_camera_paths(path)) == ["left", "front", "right", "head"]
```

Also assert the LeRobot visualizer's preferred order includes `head` after `right`.

- [ ] **Step 2: Run the visualization test and verify RED**

Run: `pytest -q tests/test_four_rgb_collection.py -k visualizer`

Expected: failure shows that `head` is absent from the preferred camera order.

- [ ] **Step 3: Implement four-camera visualization ordering**

Change both preferred order lists to:

```python
["left", "front", "right", "head"]
```

Do not hard-code away dynamically discovered cameras; they remain ordered after the preferred four.

- [ ] **Step 4: Run focused tests and verify GREEN**

Run: `pytest -q tests/test_four_rgb_collection.py -k visualizer`

Expected: all visualization tests pass.

- [ ] **Step 5: Run full verification**

Run:

```bash
pytest -q tests/test_four_rgb_collection.py tests/test_fixed_stage_collection_script.py tests/test_collection_episode_metadata.py tests/test_collection_failure_ui.py
bash -n scripts/collection/collect_mobile_pipeline_web_fixed_stage.sh
bash -n scripts/collection/collect_mobile_pipeline_web_staged.sh
bash -n scripts/collection/collect_mobile_episode_web.sh
bash -n scripts/collection/check_mobile_topics.sh
python3 -m py_compile ros2_ws/src/data_tools/launch/run_aloha_mobile_data_capture_to_mcap.launch.py ros2_ws/src/data_tools/scripts/compress_camera.py ros2_ws/src/data_tools/scripts/data_to_hdf5.py scripts/collection/verify_mobile_hdf5.py scripts/replay/visualize_dataset_web.py scripts/replay/visualize_lerobot.py
source /opt/ros/humble/setup.bash && cd ros2_ws && colcon build --packages-select data_tools --symlink-install
```

Expected: pytest reports zero failures, all Bash/Python syntax checks exit 0, and the ROS package build exits 0. If host-only hardware topics are unavailable, report that limitation explicitly; do not claim a live-camera test.

- [ ] **Step 6: Commit Task 3**

```bash
git add tests/test_four_rgb_collection.py scripts/replay/visualize_dataset_web.py scripts/replay/visualize_lerobot.py
git commit -m "feat: visualize fourth mobile rgb stream"
```

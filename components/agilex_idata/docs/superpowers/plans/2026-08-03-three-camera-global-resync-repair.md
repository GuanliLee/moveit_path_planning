# Three-Camera Global Resync Repair Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Rebuild the 1113 selected scene10–scene16 `three_camera_global` episodes with `/camera_h/color/image_raw` timed by MCAP record time, validate content sync, and generate publishable LeRobot datasets without overwriting the originals.

**Architecture:** Add an explicit per-topic timestamp-source override at the MCAP extraction boundary and pass it through every conversion wrapper. A resumable repair driver inventories only episodes already present in the authoritative HDF5 set, writes a manifest, converts into sibling `three_camera_global_resynced` directories, restores authoritative metadata, applies stationary trimming, and classifies every episode before LeRobot export.

**Tech Stack:** Python 3.10, ROS 2 Humble `rosbag2_py`, HDF5/h5py, OpenCV, ffmpeg, pytest, existing MCAP→ICRA and LeRobot conversion scripts.

## Global Constraints

- Process only `/home/agilex/data/stage2_new/scene10` through `scene16` and only `three_camera_global`.
- Never overwrite, delete, or rename the existing `three_camera_global` directories or raw MCAP files.
- Use MCAP record time only for `/camera_h/color/image_raw`; retain message-header time for every other topic.
- Use a 30 ms synchronization tolerance and 30 FPS output.
- Reapply stationary trimming with 15 retained stationary frames.
- Classify raw head coverage `>=95%` as publish candidate, `>=80% and <95%` as quarantine, and `<80%` as reject.
- Require verified publish episodes to have effective head–wrist content lag no greater than 2 frames.
- Regenerate LeRobot only from verified publish HDF5 outputs.
- All episode writes use a temporary directory followed by atomic rename.

---

### Task 1: Explicit Per-Topic Record-Time Extraction

**Files:**
- Modify: `scripts/embodied_data_pipeline-main/mcap_conversion/scripts/mcap_to_aloha_data.py:113-114,318-470,517-555`
- Create: `tests/test_mcap_record_time_topics.py`

**Interfaces:**
- Consumes: ROS messages with `header.stamp`, MCAP `reader.read_next()` record timestamp in nanoseconds, and a `set[str]` of overridden topics.
- Produces: `message_timestamp_sec_str(topic: str, message: object, record_timestamp_ns: int, record_time_topics: set[str]) -> str` and CLI `--recordTimeTopic TOPIC` with repeatable append semantics.

- [ ] **Step 1: Write failing timestamp-selection tests**

```python
def test_head_topic_uses_record_time():
    module = load_module()
    msg = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=0)))
    assert module.message_timestamp_sec_str(
        "/camera_h/color/image_raw", msg, 12_500_000_000, {"/camera_h/color/image_raw"}
    ) == "12.500000"


def test_unlisted_topic_keeps_header_time():
    module = load_module()
    msg = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=250_000_000)))
    assert module.message_timestamp_sec_str(
        "/camera_l/color/image_raw", msg, 12_500_000_000, {"/camera_h/color/image_raw"}
    ) == "10.250000"
```

- [ ] **Step 2: Run the focused test and verify the missing-helper failure**

Run: `pytest -q tests/test_mcap_record_time_topics.py`

Expected: FAIL because `message_timestamp_sec_str` and `--recordTimeTopic` do not exist.

- [ ] **Step 3: Implement the minimal selector and wire camera filenames to it**

```python
def message_timestamp_sec_str(topic, message, record_timestamp_ns, record_time_topics):
    if topic in record_time_topics:
        return ns_to_sec_str(record_timestamp_ns)
    if not hasattr(message, "header"):
        return ns_to_sec_str(record_timestamp_ns)
    return ros_time_to_sec_str(message.header.stamp)
```

Add `record_time_topics=None` to `process_file`, normalize it with `set(record_time_topics or ())`, use the helper for camera color/depth filenames, and add:

```python
parser.add_argument(
    "--recordTimeTopic",
    action="append",
    default=[],
    help="Use MCAP record time instead of message header time for this exact topic; repeatable.",
)
```

- [ ] **Step 4: Run the focused test and module compile check**

Run: `pytest -q tests/test_mcap_record_time_topics.py && python3 -m py_compile scripts/embodied_data_pipeline-main/mcap_conversion/scripts/mcap_to_aloha_data.py`

Expected: all tests pass and `py_compile` exits 0.

- [ ] **Step 5: Commit the extraction boundary**

```bash
git add tests/test_mcap_record_time_topics.py scripts/embodied_data_pipeline-main/mcap_conversion/scripts/mcap_to_aloha_data.py
git commit -m "fix: allow record-time camera extraction"
```

### Task 2: Wrapper Propagation and Timestamp Provenance

**Files:**
- Modify: `scripts/embodied_data_pipeline-main/mcap_conversion/scripts/mcap_to_hdf5.py:27-105,242-255`
- Modify: `scripts/embodied_data_pipeline-main/mcap_conversion/scripts/mcap_to_icra_episode.py:75-215,1138-1182,1060-1105`
- Modify: `scripts/embodied_data_pipeline-main/scripts/convert_mcap_dataset.py:25-95,168-230`
- Extend: `tests/test_mcap_record_time_topics.py`

**Interfaces:**
- Consumes: repeatable `args.record_time_topic: list[str]` from each CLI layer.
- Produces: repeated `--recordTimeTopic <topic>` arguments in the next command and `timestamp_source_overrides` in `meta/episode_meta.json`.

- [ ] **Step 1: Add failing command-propagation tests**

```python
def test_batch_command_propagates_record_time_topic(tmp_path):
    module = load_batch_module()
    args = module.parse_args(["--dataset-name", "x", "--record-time-topic", "/camera_h/color/image_raw"])
    command = module.build_episode_command(args, tmp_path / "episode0", tmp_path / "out")
    assert command[-2:] == ["--recordTimeTopic", "/camera_h/color/image_raw"]
```

Add equivalent assertions for `mcap_to_icra_episode.py`→`mcap_to_hdf5.py` and `mcap_to_hdf5.py`→`mcap_to_aloha_data.py` by extracting command-builder helpers rather than invoking subprocesses.

- [ ] **Step 2: Run the propagation tests and verify failure**

Run: `pytest -q tests/test_mcap_record_time_topics.py`

Expected: FAIL because the wrapper arguments and command fragments are missing.

- [ ] **Step 3: Add CLI options and repeated command fragments**

Use kebab-case at the batch layer and the existing camel-case convention below it:

```python
parser.add_argument("--record-time-topic", action="append", default=[])
```

```python
for topic in args.record_time_topic:
    command.extend(["--recordTimeTopic", topic])
```

At the ICRA and HDF5 layers use `dest="record_time_topics"`, repeat the lower-layer `--recordTimeTopic` option, and write the sorted unique list into episode metadata:

```python
meta["timestamp_source_overrides"] = {
    topic: "mcap_record_time" for topic in sorted(set(args.record_time_topics))
}
```

- [ ] **Step 4: Run propagation, parser, and dry-run tests**

Run: `pytest -q tests/test_mcap_record_time_topics.py tests/test_four_camera_qc_pipeline.py`

Expected: all tests pass.

- [ ] **Step 5: Commit wrapper propagation**

```bash
git add tests/test_mcap_record_time_topics.py scripts/embodied_data_pipeline-main/mcap_conversion/scripts/mcap_to_hdf5.py scripts/embodied_data_pipeline-main/mcap_conversion/scripts/mcap_to_icra_episode.py scripts/embodied_data_pipeline-main/scripts/convert_mcap_dataset.py
git commit -m "feat: propagate camera timestamp source"
```

### Task 3: Resumable Repair Manifest and Classification Driver

**Files:**
- Create: `scripts/embodied_data_pipeline-main/scripts/repair_three_camera_global.py`
- Create: `tests/test_repair_three_camera_global.py`

**Interfaces:**
- Consumes: `--data-root Path`, repeated `--scene int`, `--source-name`, `--output-name`, `--jobs`, `--stage`, and `--canary`; reads original `meta/episode_meta.json` and each referenced MCAP.
- Produces: `<scene>/<output-name>/repair_reports/<run_id>/manifest.jsonl`, per-episode `publish/quarantine/reject/failed` status, and atomic output directories.

- [ ] **Step 1: Write failing inventory and threshold tests**

```python
def test_classify_coverage_boundaries():
    module = load_module()
    assert module.classify_coverage(0.95) == "publish_candidate"
    assert module.classify_coverage(0.80) == "quarantine"
    assert module.classify_coverage(0.7999) == "reject"


def test_inventory_uses_existing_hdf5_as_authority(tmp_path):
    original = make_original_episode(tmp_path, scene=10, episode=40, grade="A")
    rows = module.inventory_scenes(tmp_path, [10], "three_camera_global")
    assert [(row.scene, row.episode_name, row.source_mcap) for row in rows] == [
        (10, "episode40", original.source_mcap)
    ]
```

- [ ] **Step 2: Run the repair-driver tests and verify failure**

Run: `pytest -q tests/test_repair_three_camera_global.py`

Expected: FAIL because the driver does not exist.

- [ ] **Step 3: Implement typed manifest rows and inventory**

```python
@dataclass(frozen=True)
class RepairEpisode:
    scene: int
    batch: str
    episode_name: str
    source_mcap: Path
    source_hdf5: Path
    output_hdf5: Path
    quality_grade: str
    raw_head_coverage: float
    original_content_lag_frames: int
    head_record_minus_header_median_ms: float
```

Inventory only `sceneN/three_camera_global/hdf5_episodes/*/episode*/meta/episode_meta.json`, require exactly one existing source MCAP, derive the sibling output path, compute head coverage from MCAP frame counts, and refuse duplicate `(scene, episode_name)` keys.

- [ ] **Step 4: Implement atomic conversion command construction**

For each publish candidate or quarantine row, build a command equivalent to:

```bash
python3 mcap_to_icra_episode.py \
  --mcapPath SOURCE_MCAP \
  --output TEMP_EPISODE_DIR \
  --episodeName EPISODE_NAME \
  --cameraLayout three_camera_global \
  --profile robot_profiles/aloha_four_camera.yaml \
  --alohaYaml mcap_conversion/topic_configs/aloha_four_camera_data_params.yaml \
  --timeDiffLimit 0.03 --fps 30 --overwrite --cleanupIntermediate \
  --recordTimeTopic /camera_h/color/image_raw
```

After conversion, copy only authoritative business metadata fields from the old sidecar, retain newly generated sync/provenance fields, inject the per-topic source plus `record_minus_header_median_ms` audit into the new sidecar, run stationary trim with `--keep-stationary-frames 15 --target-fps 30`, validate, and atomically rename the temp directory.

- [ ] **Step 5: Implement resumable manifest writes**

Write each JSON line to `manifest.jsonl.tmp`, `flush()` and `os.fsync()`, then replace `manifest.jsonl`. On restart, skip only rows whose output passes validation and whose source MCAP path and timestamp override match the manifest.

- [ ] **Step 6: Run unit tests and a dry-run inventory against real data**

Run: `pytest -q tests/test_repair_three_camera_global.py`

Run: `python3 scripts/embodied_data_pipeline-main/scripts/repair_three_camera_global.py --data-root /home/agilex/data/stage2_new --scene 10 --scene 11 --scene 12 --scene 13 --scene 14 --scene 15 --scene 16 --stage inventory --dry-run`

Expected: unit tests pass and inventory reports exactly 1113 selected episodes with 1002 publish candidates, 96 quarantine, and 15 reject based on the pre-conversion coverage estimate.

- [ ] **Step 7: Commit the repair driver**

```bash
git add tests/test_repair_three_camera_global.py scripts/embodied_data_pipeline-main/scripts/repair_three_camera_global.py
git commit -m "feat: add resumable camera resync repair"
```

### Task 4: Canary Conversion and Content Validation

**Files:**
- Modify as defects require: files introduced in Tasks 1–3 only
- Output: `/home/agilex/data/stage2_new/scene{10,12,15,16}/three_camera_global_resynced/`

**Interfaces:**
- Consumes: canary set `scene10/episode40`, `scene12/episode40`, `scene15/episode70`, `scene16/episode80`, and reject control `scene10/episode45`.
- Produces: repaired HDF5 episodes, manifest rows, before/after content lag, and visual montage artifacts under `repair_reports`.

- [ ] **Step 1: Run canary conversion**

Run: `python3 scripts/embodied_data_pipeline-main/scripts/repair_three_camera_global.py --data-root /home/agilex/data/stage2_new --scene 10 --scene 12 --scene 15 --scene 16 --canary --stage convert --jobs 1`

Expected: four healthy canaries convert; scene10 episode45 is classified reject and not published.

- [ ] **Step 2: Run canary validation**

Run: `python3 scripts/embodied_data_pipeline-main/scripts/repair_three_camera_global.py --data-root /home/agilex/data/stage2_new --scene 10 --scene 12 --scene 15 --scene 16 --canary --stage validate`

Expected: all four healthy canaries have head–wrist lag within 2 frames, no long head fill run, matching HDF5/video/state frame counts, and correct timestamp provenance.

- [ ] **Step 3: Inspect generated montages and compare frame counts**

Open each `repair_reports/<run_id>/canary_*_montage.mp4`; compare original and repaired `episode_meta.json` task, grade, item, and stationary-trim fields.

Expected: visible actions align across cameras and authoritative metadata is unchanged.

- [ ] **Step 4: Run focused regression tests after any canary correction**

Run: `pytest -q tests/test_mcap_record_time_topics.py tests/test_repair_three_camera_global.py`

Expected: all tests pass.

- [ ] **Step 5: Commit canary-driven corrections**

```bash
git add scripts/embodied_data_pipeline-main tests
git commit -m "fix: validate scene resync canaries"
```

### Task 5: Full Batch, LeRobot Export, and Final Audit

**Files:**
- Data output only: `/home/agilex/data/stage2_new/scene10` through `scene16` sibling `three_camera_global_resynced` directories
- Report output: each scene's `three_camera_global_resynced/repair_reports/<run_id>/`

**Interfaces:**
- Consumes: validated repair driver and original authoritative quality grades.
- Produces: publish HDF5, quarantined HDF5, rejected manifest rows, grade-separated LeRobot datasets, and final audit report.

- [ ] **Step 1: Run full resumable conversion**

Run: `python3 scripts/embodied_data_pipeline-main/scripts/repair_three_camera_global.py --data-root /home/agilex/data/stage2_new --scene 10 --scene 11 --scene 12 --scene 13 --scene 14 --scene 15 --scene 16 --stage convert --jobs 2`

Expected: all 1113 manifest rows reach converted, quarantine, reject, or failed without silent omission.

- [ ] **Step 2: Run full validation and classification**

Run: `python3 scripts/embodied_data_pipeline-main/scripts/repair_three_camera_global.py --data-root /home/agilex/data/stage2_new --scene 10 --scene 11 --scene 12 --scene 13 --scene 14 --scene 15 --scene 16 --stage validate --jobs 2`

Expected: publish outputs meet lag and frame-integrity requirements; failures remain outside publish roots.

- [ ] **Step 3: Regenerate grade-separated LeRobot datasets**

Run: `python3 scripts/embodied_data_pipeline-main/scripts/repair_three_camera_global.py --data-root /home/agilex/data/stage2_new --scene 10 --scene 11 --scene 12 --scene 13 --scene 14 --scene 15 --scene 16 --stage lerobot --jobs 1`

Expected: only publish HDF5 episodes appear in LeRobot, grouped by their original A/B grades, with exact episode/frame mappings recorded.

- [ ] **Step 4: Run final audit**

Run: `python3 scripts/embodied_data_pipeline-main/scripts/repair_three_camera_global.py --data-root /home/agilex/data/stage2_new --scene 10 --scene 11 --scene 12 --scene 13 --scene 14 --scene 15 --scene 16 --stage audit`

Expected: 1113 accounted episodes; original-tree checksums unchanged; every publish episode has lag `<=2`; manifest and LeRobot mappings are internally consistent.

- [ ] **Step 5: Save and review the final reports**

Review per-scene CSV/JSON summaries and at least one repaired severe-lag montage per scene. Do not promote the sibling directory over the original unless the user explicitly authorizes promotion.

# Four-Camera Mobile QC Pipeline Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build an isolated port-8002 four-camera MCAP conversion and QC service that preserves `head` through videos, QC, replay, repair, and LeRobot without changing port-8001 three-camera defaults.

**Architecture:** Add a thin four-camera launcher and thin converter entry points backed by parameterized shared conversion code. Select a separate topic YAML, camera layout, profile, and output namespace through explicit variant settings; all omitted settings retain the current three-camera behavior.

**Tech Stack:** Bash, Python 3, ROS 2 `rosbag2_py`, YAML, HDF5/h5py, OpenCV/FFmpeg, pytest, LeRobot.

## Global Constraints

- Existing `scripts/collection/collect_mobile_pipeline_qc_web.sh` remains on port `8001` and defaults to exactly three cameras.
- New `scripts/collection/collect_mobile_pipeline_qc_web_four_camera.sh` defaults to port `8002`.
- Camera mapping is `left -> hand_left_color`, `front -> head_color`, `right -> hand_right_color`, `head -> head`.
- Fourth LeRobot key is exactly `observation.images.head`.
- `/camera_h/color/camera_info` is never required or consumed.
- No `camera/colorIntrinsic/head` or `camera/colorExtrinsic/head` dataset is created.
- Existing motion, action, duration, vision, and grading thresholds do not change.
- Default four-camera outputs live below a `four_camera` namespace; explicit output paths still win.
- Shared core code is parameterized; the full pipeline source tree is not copied.

---

### Task 1: Lock the three-camera contract and add isolated four-camera resources

**Files:**
- Create: `scripts/collection/collect_mobile_pipeline_qc_web_four_camera.sh`
- Create: `scripts/embodied_data_pipeline-main/mcap_conversion/topic_configs/aloha_four_camera_data_params.yaml`
- Create: `scripts/embodied_data_pipeline-main/robot_profiles/aloha_four_camera.yaml`
- Modify: `scripts/collection/collect_mobile_pipeline_qc_web.sh`
- Create: `tests/test_four_camera_qc_pipeline.py`

**Interfaces:**
- Consumes: existing 8001 launcher environment forwarding and `aloha.yaml` schema.
- Produces: fixed variant environment variables `PIPELINE_CAMERA_LAYOUT=four_camera`, `PIPELINE_DEFAULT_PROFILE`, `PIPELINE_ALOHA_YAML`, and `PIPELINE_OUTPUT_NAMESPACE=four_camera`.

- [ ] **Step 1: Write failing launcher and configuration tests**

```python
def test_three_camera_qc_entry_keeps_8001_defaults():
    text = THREE_CAMERA_ENTRY.read_text(encoding="utf-8")
    assert 'WEB_PORT="${WEB_PORT:-8001}"' in text


def test_four_camera_qc_entry_is_isolated_on_8002():
    text = FOUR_CAMERA_ENTRY.read_text(encoding="utf-8")
    assert 'WEB_PORT="${WEB_PORT:-8002}"' in text
    assert "aloha_four_camera.yaml" in text
    assert "aloha_four_camera_data_params.yaml" in text
    assert "PIPELINE_CAMERA_LAYOUT" in text
    assert "PIPELINE_OUTPUT_NAMESPACE" in text


def test_four_camera_topic_yaml_has_four_images_and_three_calibrations():
    color = load_color_config(FOUR_CAMERA_TOPIC_YAML)
    assert color["names"] == ["left", "front", "right", "head"]
    assert color["topics"][-1] == "/camera_h/color/image_raw"
    assert len(color["configTopics"]) == 3
    assert all("camera_h" not in topic for topic in color["configTopics"])


def test_four_camera_profile_appends_required_head():
    profile = yaml.safe_load(FOUR_CAMERA_PROFILE.read_text(encoding="utf-8"))
    cameras = {camera["raw_key"]: camera for camera in profile["cameras"]}
    assert cameras["head"]["lerobot_key"] == "observation.images.head"
    assert profile["quality_checks"]["required_cameras"] == [
        "head_color", "hand_left_color", "hand_right_color", "head"
    ]
```

- [ ] **Step 2: Run tests and verify the four-camera resources are missing**

Run: `python3 -m pytest -q tests/test_four_camera_qc_pipeline.py`

Expected: FAIL because the new launcher, topic YAML, and profile do not exist.

- [ ] **Step 3: Add the thin launcher and resource files**

The new launcher sets variant defaults and executes the existing launcher:

```bash
#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

export WEB_PORT="${WEB_PORT:-8002}"
export PIPELINE_DEFAULT_PROFILE="${PIPELINE_DEFAULT_PROFILE:-${REPO_ROOT}/scripts/embodied_data_pipeline-main/robot_profiles/aloha_four_camera.yaml}"
export PIPELINE_ALOHA_YAML="${PIPELINE_ALOHA_YAML:-${REPO_ROOT}/scripts/embodied_data_pipeline-main/mcap_conversion/topic_configs/aloha_four_camera_data_params.yaml}"
export PIPELINE_CAMERA_LAYOUT="${PIPELINE_CAMERA_LAYOUT:-four_camera}"
export PIPELINE_OUTPUT_NAMESPACE="${PIPELINE_OUTPUT_NAMESPACE:-four_camera}"
exec bash "${SCRIPT_DIR}/collect_mobile_pipeline_qc_web.sh" "$@"
```

Copy `aloha.yaml` to `aloha_four_camera.yaml`, append this camera, and append
`head` to `quality_checks.required_cameras` while leaving all other thresholds
identical:

```yaml
  - raw_key: head
    lerobot_key: observation.images.head
    role: head_auxiliary
    required: true
```

Copy the three-camera topic YAML and append `head` to `names`,
`camera_h_link` to `parentFrames`, and `/camera_h/color/image_raw` to `topics`.
Keep `configTopics` at the original three entries.

Update the existing launcher only to forward the four optional variant variables
to `pipeline_web_app.py`; empty defaults preserve 8001 behavior.

- [ ] **Step 4: Run focused tests and shell syntax checks**

Run:

```bash
python3 -m pytest -q tests/test_pipeline_qc_web_script.py tests/test_four_camera_qc_pipeline.py
bash -n scripts/collection/collect_mobile_pipeline_qc_web.sh
bash -n scripts/collection/collect_mobile_pipeline_qc_web_four_camera.sh
```

Expected: all tests pass and both `bash -n` commands exit 0.

- [ ] **Step 5: Commit isolated resources**

```bash
git add scripts/collection/collect_mobile_pipeline_qc_web.sh \
  scripts/collection/collect_mobile_pipeline_qc_web_four_camera.sh \
  scripts/embodied_data_pipeline-main/mcap_conversion/topic_configs/aloha_four_camera_data_params.yaml \
  scripts/embodied_data_pipeline-main/robot_profiles/aloha_four_camera.yaml \
  tests/test_four_camera_qc_pipeline.py
git commit -m "feat: add isolated four-camera qc resources"
```

---

### Task 2: Parameterize camera layout and add thin four-camera converters

**Files:**
- Create: `scripts/embodied_data_pipeline-main/mcap_conversion/scripts/camera_layouts.py`
- Create: `scripts/embodied_data_pipeline-main/mcap_conversion/scripts/mcap_to_icra_episode_four_camera.py`
- Create: `scripts/embodied_data_pipeline-main/scripts/convert_mcap_dataset_four_camera.py`
- Modify: `scripts/embodied_data_pipeline-main/mcap_conversion/scripts/mcap_to_icra_episode.py`
- Modify: `scripts/embodied_data_pipeline-main/scripts/convert_mcap_dataset.py`
- Test: `tests/test_four_camera_qc_pipeline.py`

**Interfaces:**
- Produces: `CameraLayout`, `get_camera_layout(name: str) -> CameraLayout`,
  `mcap_to_icra_episode.parse_args(argv: list[str] | None = None)`,
  `mcap_to_icra_episode.main(argv: list[str] | None = None) -> int`,
  `convert_mcap_dataset.parse_args(argv: list[str] | None = None)`, and
  `convert_mcap_dataset.build_episode_command(args, mcap_input: Path, out_dir: Path) -> list[str]`.
- Consumes: Task 1 four-camera YAML/profile paths.

- [ ] **Step 1: Write failing layout and command-plumbing tests**

```python
def test_camera_layouts_are_explicit_and_three_camera_default_is_unchanged():
    three = camera_layouts.get_camera_layout("three_camera")
    four = camera_layouts.get_camera_layout("four_camera")
    assert list(three.video_sources) == [
        "head_color.mp4", "hand_left_color.mp4", "hand_right_color.mp4"
    ]
    assert four.video_sources["head.mp4"] == Path("camera/color/head/sync.txt")
    assert four.required_video_names == tuple(four.video_sources)


def test_batch_converter_passes_four_camera_options(monkeypatch, tmp_path):
    args = parse_convert_args([
        "--dataset-name", "sample",
        "--camera-layout", "four_camera",
        "--aloha-yaml", str(FOUR_CAMERA_TOPIC_YAML),
    ])
    command = build_episode_command(args, tmp_path / "episode1", tmp_path / "out")
    assert command[command.index("--cameraLayout") + 1] == "four_camera"
    assert command[command.index("--alohaYaml") + 1] == str(FOUR_CAMERA_TOPIC_YAML)
```

- [ ] **Step 2: Run the focused test and verify it fails**

Run: `python3 -m pytest -q tests/test_four_camera_qc_pipeline.py -k 'layout or batch_converter'`

Expected: FAIL because `camera_layouts.py` and the new CLI options are absent.

- [ ] **Step 3: Implement the layout module**

```python
@dataclass(frozen=True)
class CameraLayout:
    name: str
    video_sources: dict[str, Path]
    reference_video_names: tuple[str, ...]
    required_video_names: tuple[str, ...]


THREE_CAMERA = CameraLayout(
    name="three_camera",
    video_sources={
        "head_color.mp4": Path("camera/color/front/sync.txt"),
        "hand_left_color.mp4": Path("camera/color/left/sync.txt"),
        "hand_right_color.mp4": Path("camera/color/right/sync.txt"),
    },
    reference_video_names=(
        "hand_left_color.mp4", "hand_right_color.mp4", "head_color.mp4", "head_depth.mp4"
    ),
    required_video_names=(),
)

FOUR_CAMERA = CameraLayout(
    name="four_camera",
    video_sources={**THREE_CAMERA.video_sources, "head.mp4": Path("camera/color/head/sync.txt")},
    reference_video_names=(*THREE_CAMERA.reference_video_names, "head.mp4"),
    required_video_names=(
        "head_color.mp4", "hand_left_color.mp4", "hand_right_color.mp4", "head.mp4"
    ),
)
```

`get_camera_layout` accepts `three_camera` and `four_camera` and raises
`ValueError` for any other value.

- [ ] **Step 4: Refactor conversion functions to consume a selected layout**

Add `argv=None` to both converters' `parse_args` and `main`, extract the current
per-episode list construction into
`build_episode_command(args, mcap_input: Path, out_dir: Path) -> list[str]`, add
`--cameraLayout/--camera-layout`, and pass the selected `CameraLayout` into:

```python
add_sync_stats_to_video_infos(video_infos, sync_summary, video_sources)
write_available_videos(..., camera_layout)
```

For `four_camera`, raise `FileNotFoundError` listing any missing member of
`required_video_names`. For the default `three_camera`, preserve the current
missing-video behavior.

Add thin entry points that inject four-camera defaults only when the caller did
not explicitly supply them, then call the shared `main(argv)`.

- [ ] **Step 5: Run converter tests and characterize old defaults**

Run:

```bash
python3 -m pytest -q tests/test_four_camera_qc_pipeline.py tests/test_pipeline_failure_paths.py
python3 scripts/embodied_data_pipeline-main/mcap_conversion/scripts/mcap_to_icra_episode.py --help >/tmp/mcap_three_help.txt
python3 scripts/embodied_data_pipeline-main/mcap_conversion/scripts/mcap_to_icra_episode_four_camera.py --help >/tmp/mcap_four_help.txt
```

Expected: new focused tests pass; any pre-existing `test_pipeline_failure_paths.py`
baseline failure is recorded separately and no new failure is introduced.

- [ ] **Step 6: Commit layout parameterization**

```bash
git add scripts/embodied_data_pipeline-main/mcap_conversion/scripts/camera_layouts.py \
  scripts/embodied_data_pipeline-main/mcap_conversion/scripts/mcap_to_icra_episode.py \
  scripts/embodied_data_pipeline-main/mcap_conversion/scripts/mcap_to_icra_episode_four_camera.py \
  scripts/embodied_data_pipeline-main/scripts/convert_mcap_dataset.py \
  scripts/embodied_data_pipeline-main/scripts/convert_mcap_dataset_four_camera.py \
  tests/test_four_camera_qc_pipeline.py
git commit -m "feat: parameterize qc camera conversion layout"
```

---

### Task 3: Preserve the fourth stream through HDF5 without fake calibration

**Files:**
- Modify: `scripts/embodied_data_pipeline-main/mcap_conversion/scripts/data_to_hdf5.py`
- Modify: `scripts/embodied_data_pipeline-main/mcap_conversion/scripts/ario_hdf5_to_aligned_joints.py`
- Test: `tests/test_four_camera_qc_pipeline.py`

**Interfaces:**
- Consumes: intermediate `camera/color/head` produced by Task 2.
- Produces: mobile HDF5 `camera/color/head` and aligned HDF5 `timestamp/camera/head`.

- [ ] **Step 1: Add a synthetic four-camera HDF5 regression fixture and failing assertions**

```python
def test_aligned_adapter_writes_four_camera_timestamps(tmp_path):
    source = make_mobile_hdf5(tmp_path / "mobile.hdf5", cameras=("left", "front", "right", "head"))
    output = tmp_path / "aligned.h5"
    aligned.convert(source, output, robot="aloha")
    with h5py.File(output, "r") as root:
        assert "0/timestamp/camera/head" in root


def test_mobile_hdf5_omits_uncalibrated_head_datasets(tmp_path):
    output = build_mobile_hdf5_fixture(tmp_path, calibrated=("left", "front", "right"))
    with h5py.File(output, "r") as root:
        assert "camera/color/head" in root
        assert "camera/colorIntrinsic/head" not in root
        assert "camera/colorExtrinsic/head" not in root
```

- [ ] **Step 2: Run tests and verify the missing head timestamp/calibration failure**

Run: `python3 -m pytest -q tests/test_four_camera_qc_pipeline.py -k 'aligned or calibration'`

Expected: FAIL because the aligned adapter reads only three cameras and the
pipeline converter currently pre-creates calibration datasets for every camera.

- [ ] **Step 3: Make calibration datasets conditional**

In `data_to_hdf5.py`, create intrinsic/extrinsic keys only for camera directories
that contain a valid `config.json`:

```python
data_dict[f"camera/color/{camera_name}"] = []
config_path = os.path.join(self.cameraColorDirs[index], "config.json")
if os.path.isfile(config_path):
    data_dict[f"camera/colorIntrinsic/{camera_name}"] = []
    data_dict[f"camera/colorExtrinsic/{camera_name}"] = []
```

Guard later calibration assignment with the same key-presence check.

- [ ] **Step 4: Add optional head input and aligned timestamp**

In `load_aloha_source`, add:

```python
"camera_head": src.get("camera/color/head"),
```

In `write_aloha_frame`, append:

```python
if src_data["camera_head"] is not None:
    camera_ts["head"] = camera_timestamp(src_data, "camera_head", frame_idx, main_ts_ns)
```

Old three-camera sources therefore retain exactly their original timestamp keys.

- [ ] **Step 5: Run HDF5 tests and compile the scripts**

Run:

```bash
python3 -m pytest -q tests/test_four_camera_qc_pipeline.py -k 'aligned or calibration'
python3 -m py_compile \
  scripts/embodied_data_pipeline-main/mcap_conversion/scripts/data_to_hdf5.py \
  scripts/embodied_data_pipeline-main/mcap_conversion/scripts/ario_hdf5_to_aligned_joints.py
```

Expected: all selected tests pass and compilation exits 0.

- [ ] **Step 6: Commit HDF5 preservation**

```bash
git add scripts/embodied_data_pipeline-main/mcap_conversion/scripts/data_to_hdf5.py \
  scripts/embodied_data_pipeline-main/mcap_conversion/scripts/ario_hdf5_to_aligned_joints.py \
  tests/test_four_camera_qc_pipeline.py
git commit -m "feat: preserve fourth camera in qc hdf5"
```

---

### Task 4: Route the Web service to isolated variant settings and outputs

**Files:**
- Modify: `scripts/embodied_data_pipeline-main/scripts/pipeline_web_app.py`
- Modify: `scripts/collection/collect_mobile_pipeline_qc_web.sh`
- Test: `tests/test_four_camera_qc_pipeline.py`
- Test: `tests/test_pipeline_qc_web_script.py`

**Interfaces:**
- Consumes: `PIPELINE_DEFAULT_PROFILE`, `PIPELINE_ALOHA_YAML`, `PIPELINE_CAMERA_LAYOUT`, `PIPELINE_OUTPUT_NAMESPACE`.
- Produces: config keys `aloha_yaml`, `camera_layout`, `output_namespace`; host/Docker conversion commands with matching values.

- [ ] **Step 1: Write failing Web configuration and command tests**

```python
def test_four_camera_environment_selects_profile_layout_and_namespace(monkeypatch):
    monkeypatch.setenv("PIPELINE_DEFAULT_PROFILE", str(FOUR_CAMERA_PROFILE))
    monkeypatch.setenv("PIPELINE_ALOHA_YAML", str(FOUR_CAMERA_TOPIC_YAML))
    monkeypatch.setenv("PIPELINE_CAMERA_LAYOUT", "four_camera")
    monkeypatch.setenv("PIPELINE_OUTPUT_NAMESPACE", "four_camera")
    cfg = app.derive_paths({"robot_type": "aloha", "data_root": "/tmp/data", "dataset_name": "demo"})
    assert cfg["profile"] == FOUR_CAMERA_PROFILE
    assert cfg["hdf5_root"] == Path("/tmp/data/four_camera/hdf5_episodes/demo")
    assert cfg["qc_root"] == Path("/tmp/data/four_camera/qc_reports")
    assert cfg["lerobot_root"] == Path("/tmp/data/four_camera/lerobot")


def test_four_camera_host_conversion_command_passes_variant(monkeypatch, tmp_path):
    cfg = four_camera_cfg(tmp_path)
    cmd, _, _ = app.convert_command(cfg)
    assert cmd[cmd.index("--camera-layout") + 1] == "four_camera"
    assert cmd[cmd.index("--aloha-yaml") + 1] == str(FOUR_CAMERA_TOPIC_YAML)
```

Also assert that a clean environment still derives the original profile and
non-namespaced output paths.

- [ ] **Step 2: Run tests and verify the config keys are missing**

Run: `python3 -m pytest -q tests/test_four_camera_qc_pipeline.py -k 'environment or command or namespace'`

Expected: FAIL because the Web app does not yet read the variant environment.

- [ ] **Step 3: Add variant configuration helpers**

```python
def env_path(name: str) -> Path | None:
    value = str(os.environ.get(name) or "").strip()
    return Path(value).expanduser().resolve() if value else None


def output_base(data_root: Path) -> Path:
    namespace = str(os.environ.get("PIPELINE_OUTPUT_NAMESPACE") or "").strip().strip("/")
    return data_root / namespace if namespace else data_root
```

Use the environment profile only when the payload has no explicit `profile`.
Add the YAML/layout/namespace values to the derived config. Use the namespaced
base only for default output paths, never for explicit HDF5/QC/LeRobot paths.

- [ ] **Step 4: Pass variant arguments through host and Docker commands**

Host batch conversion receives:

```python
"--aloha-yaml", str(cfg["aloha_yaml"]),
"--camera-layout", cfg["camera_layout"],
```

The Docker inner `mcap_to_icra_episode.py` command receives the equivalent
`--alohaYaml` and `--cameraLayout` values, and mounts the selected YAML when it
is outside the existing topic-config mount.

- [ ] **Step 5: Run Web command tests and old pipeline regression tests**

Run:

```bash
python3 -m pytest -q \
  tests/test_four_camera_qc_pipeline.py \
  tests/test_pipeline_qc_web_script.py \
  tests/test_pipeline_task_consistency.py
```

Expected: all selected tests pass.

- [ ] **Step 6: Commit Web variant routing**

```bash
git add scripts/embodied_data_pipeline-main/scripts/pipeline_web_app.py \
  scripts/collection/collect_mobile_pipeline_qc_web.sh \
  tests/test_four_camera_qc_pipeline.py tests/test_pipeline_qc_web_script.py
git commit -m "feat: route isolated four-camera qc service"
```

---

### Task 5: Make QC, repair, replay, and LeRobot retain `head`

**Files:**
- Modify: `scripts/embodied_data_pipeline-main/quality_pipeline/qc.py`
- Modify: `scripts/embodied_data_pipeline-main/mcap_conversion/scripts/repair_need_repair_episodes.py`
- Modify: `scripts/embodied_data_pipeline-main/mcap_conversion/scripts/replay_icra_episode.py`
- Modify: `scripts/embodied_data_pipeline-main/scripts/pipeline_web_app.py`
- Test: `tests/test_four_camera_qc_pipeline.py`

**Interfaces:**
- Consumes: aligned `timestamp/camera/head`, `videos/head.mp4`, and four-camera profile.
- Produces: four-camera QC completeness details, repair mapping, replay descriptor, and dynamic Web ordering.

- [ ] **Step 1: Write failing downstream-consumer tests**

```python
def test_four_camera_qc_rejects_episode_without_head(tmp_path):
    episode = make_icra_episode(tmp_path, videos=("head_color", "hand_left_color", "hand_right_color"))
    report = run_qc(episode, FOUR_CAMERA_PROFILE)
    camera_check = check(report, "camera_completeness")
    assert camera_check["status"] == "fail"
    assert camera_check["details"]["counts"]["head"] == 0


def test_repair_and_replay_discover_head_video(tmp_path):
    episode = make_icra_episode(tmp_path, videos=("head_color", "hand_left_color", "hand_right_color", "head"))
    assert repair.video_camera_name(episode / "videos/head.mp4") == "head"
    assert [item["key"] for item in replay.available_video_infos(episode)][-1] == "head"
```

Add an assertion that the four-camera profile produces a LeRobot camera spec
whose raw key is `head` and output key is `observation.images.head`.

- [ ] **Step 2: Run downstream tests and verify failures**

Run: `python3 -m pytest -q tests/test_four_camera_qc_pipeline.py -k 'qc_rejects or repair or replay or lerobot'`

Expected: repair/replay mappings do not yet include `head`; QC fixture exposes
any missing timestamp/video plumbing.

- [ ] **Step 3: Extend dynamic camera mappings**

Append optional mappings without removing existing entries:

```python
# quality_pipeline/qc.py fallback
"head": "head",

# repair_need_repair_episodes.py
"timestamp/camera/head": "head",
"head.mp4": "head",

# replay_icra_episode.py
("head", "Head Auxiliary", "head.mp4"),

# pipeline_web_app.py browser ordering
head: 3,
```

Prefer episode `available_videos` metadata and profile cameras wherever the
current function already receives them; keep these mappings as compatibility
fallbacks.

- [ ] **Step 4: Run QC/replay/repair tests and LeRobot feature tests**

Run:

```bash
python3 -m pytest -q tests/test_four_camera_qc_pipeline.py
python3 -m pytest -q tests/test_lerobot_conversion_stats.py
```

Expected: four-camera tests pass. If the system Python lacks LeRobot, rerun the
LeRobot tests with `/home/agilex/openpi/.venv/bin/python -m pytest` and record
the system-environment dependency failure separately.

- [ ] **Step 5: Commit downstream camera support**

```bash
git add scripts/embodied_data_pipeline-main/quality_pipeline/qc.py \
  scripts/embodied_data_pipeline-main/mcap_conversion/scripts/repair_need_repair_episodes.py \
  scripts/embodied_data_pipeline-main/mcap_conversion/scripts/replay_icra_episode.py \
  scripts/embodied_data_pipeline-main/scripts/pipeline_web_app.py \
  tests/test_four_camera_qc_pipeline.py
git commit -m "feat: validate and expose fourth qc camera"
```

---

### Task 6: End-to-end four-camera conversion and dual-service verification

**Files:**
- Modify: `tests/test_four_camera_qc_pipeline.py`
- Modify: `scripts/embodied_data_pipeline-main/README.md`
- Modify: `scripts/collection/collect_mobile_pipeline_qc_web_four_camera.sh`

**Interfaces:**
- Consumes: all Tasks 1-5.
- Produces: verified four-camera HDF5, MP4, QC report, LeRobot dataset, and concurrent 8001/8002 Web services.

- [ ] **Step 1: Add an end-to-end fixture test**

Create a controlled episode fixture with numeric frame groups, state/action
datasets, four timestamp keys, four short H.264 videos, and metadata. Assert:

```python
assert (episode / "videos/head.mp4").is_file()
assert qc_report["accepted"] is True
assert set(camera_counts) == {"head_color", "hand_left_color", "hand_right_color", "head"}
assert "observation.images.head" in lerobot_info["features"]
```

Create the same fixture without `head` and assert four-camera QC fails while
three-camera QC still accepts the camera contract.

- [ ] **Step 2: Run end-to-end automated tests**

Run:

```bash
python3 -m pytest -q tests/test_four_camera_qc_pipeline.py
python3 -m pytest -q \
  tests/test_pipeline_qc_web_script.py \
  tests/test_pipeline_task_consistency.py \
  tests/test_four_rgb_collection.py \
  tests/test_capture_health.py
```

Expected: all selected tests pass.

- [ ] **Step 3: Run the real converter on a controlled four-camera MCAP**

Use a recent four-camera capture if available. Otherwise record a controlled
ROS 2 fixture containing the full required ALOHA state/action topics and four
RGB topics. Run:

```bash
/home/agilex/openpi/.venv/bin/python \
  scripts/embodied_data_pipeline-main/scripts/convert_mcap_dataset_four_camera.py \
  --dataset-name four_camera_e2e \
  --input-root /tmp/four_camera_e2e/raw_mcap \
  --output-root /tmp/four_camera_e2e/hdf5_episodes \
  --text "four camera integration test" \
  --overwrite
```

Expected: one completed episode with `states/aligned_joints.h5`, four MP4 files,
and `meta/episode_meta.json` listing four available videos.

- [ ] **Step 4: Run QC and LeRobot export on the converted episode**

```bash
/home/agilex/openpi/.venv/bin/python \
  scripts/embodied_data_pipeline-main/scripts/run_quality_pipeline.py \
  --profile scripts/embodied_data_pipeline-main/robot_profiles/aloha_four_camera.yaml \
  --input /tmp/four_camera_e2e/hdf5_episodes \
  --output /tmp/four_camera_e2e/qc

/home/agilex/openpi/.venv/bin/python \
  scripts/embodied_data_pipeline-main/lerobot_conversion/scripts/convert_hdf5_to_lerobot_v2.py \
  --profile scripts/embodied_data_pipeline-main/robot_profiles/aloha_four_camera.yaml \
  --data-dir /tmp/four_camera_e2e/hdf5_episodes \
  --output-root /tmp/four_camera_e2e/lerobot \
  --repo-id four_camera_e2e
```

Expected: QC camera completeness passes for all four keys and LeRobot metadata
contains `observation.images.head`.

- [ ] **Step 5: Start both Web services concurrently**

Start 8001 and 8002 with separate temporary logs, then run:

```bash
curl --fail --silent http://127.0.0.1:8001/ -o /tmp/qc_8001.html
curl --fail --silent http://127.0.0.1:8002/ -o /tmp/qc_8002.html
```

Expected: both requests exit 0; process inspection shows distinct ports; the
8002 status/config endpoint reports the four-camera profile and namespace.
Stop only the two test processes after verification.

- [ ] **Step 6: Document operation and run full regression**

Document the two entry commands, port distinction, output namespace, four
camera keys, and absence of `camera_h` calibration. Then run:

```bash
bash -n scripts/collection/collect_mobile_pipeline_qc_web.sh
bash -n scripts/collection/collect_mobile_pipeline_qc_web_four_camera.sh
python3 -m py_compile \
  scripts/embodied_data_pipeline-main/scripts/*.py \
  scripts/embodied_data_pipeline-main/mcap_conversion/scripts/*.py \
  scripts/embodied_data_pipeline-main/quality_pipeline/*.py
python3 -m pytest -q
git diff --check
```

Expected: all feature tests pass. Compare any full-suite failures against the
known baseline and fix every newly introduced failure.

- [ ] **Step 7: Commit documentation and integration verification**

```bash
git add scripts/embodied_data_pipeline-main/README.md \
  scripts/collection/collect_mobile_pipeline_qc_web_four_camera.sh \
  tests/test_four_camera_qc_pipeline.py
git commit -m "test: verify four-camera qc pipeline end to end"
```

---

### Task 7: Final review and delivery gate

**Files:**
- Review: all files changed since `3aad0c7`

**Interfaces:**
- Consumes: completed implementation and verification evidence.
- Produces: clean reviewed branch ready to merge into the original workspace.

- [ ] **Step 1: Review the complete diff against the design**

Run:

```bash
git diff --check 3aad0c7..HEAD
git diff --stat 3aad0c7..HEAD
git status --short
```

Expected: no whitespace errors and no uncommitted implementation changes.

- [ ] **Step 2: Run the final focused gate from a clean process environment**

```bash
python3 -m pytest -q \
  tests/test_four_camera_qc_pipeline.py \
  tests/test_pipeline_qc_web_script.py \
  tests/test_pipeline_task_consistency.py \
  tests/test_four_rgb_collection.py \
  tests/test_capture_health.py
```

Expected: 0 failures.

- [ ] **Step 3: Perform an independent code review**

Review for three-camera behavior changes, missing Docker argument propagation,
unsafe output overlap, fake head calibration, camera count truncation, and
LeRobot feature loss. Fix all critical and important findings and rerun the
focused gate.

- [ ] **Step 4: Merge the implementation branch and verify the original path**

After the isolated implementation worktree passes review, merge it into the
original repository branch. From `/home/caizj/agilex_idata`, rerun shell syntax,
the focused test gate, and dual-port startup before reporting delivery.

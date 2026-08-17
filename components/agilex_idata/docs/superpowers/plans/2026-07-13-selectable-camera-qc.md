# Selectable Three/Four-Camera QC Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the specialized four-camera QC console select and correctly export either three or four camera streams, with selectable three-camera head input and default port `8012`.

**Architecture:** Resolve the operator's camera count/head-source selection into strict named camera layouts and deterministic LeRobot profiles. Keep the generic legacy layout untouched, pass the resolved configuration through the existing Web pipeline, and isolate each variant's default output paths.

**Tech Stack:** Bash, Python 3, standard-library HTTP/JavaScript UI, YAML robot profiles, pytest.

## Global Constraints

- `/camera_f` is the normal head input and `/camera_h` is the wide-angle/global input.
- Four-camera output keys are exactly `observation.images.hand_left_color`, `observation.images.hand_right_color`, `observation.images.hand_head_color`, and `observation.images.global_color`.
- Three-camera output keys are exactly the two hand keys plus `observation.images.hand_head_color` from the selected head input.
- The specialized launcher defaults to four cameras, `front` head source, and port `8012`.
- Existing generic `three_camera` conversion behavior remains backward compatible.
- Existing user working-tree changes must not be overwritten or committed.

---

### Task 1: Launcher contract

**Files:**
- Modify: `scripts/collection/collect_mobile_pipeline_qc_web.sh`
- Modify: `scripts/collection/collect_mobile_pipeline_qc_web_four_camera.sh`
- Test: `tests/test_selectable_camera_qc.py`

**Interfaces:**
- Consumes: environment defaults `PIPELINE_CAMERA_COUNT`, `PIPELINE_HEAD_CAMERA_SOURCE`, and `PIPELINE_CAMERA_VARIANT_SELECTABLE`.
- Produces: `--camera-count 3|4`, `--head-camera front|global`, validated environment passed to `pipeline_web_app.py`, and specialized defaults `8012`/four/front.

- [ ] **Step 1: Write failing launcher tests**

Create tests that read both shell scripts and run `--help`, asserting port `8012`, the two new options, specialized selection enablement, and default values.

- [ ] **Step 2: Verify the launcher tests fail**

Run: `pytest -q tests/test_selectable_camera_qc.py -k launcher`

Expected: failures showing port `8002` and missing camera-selection arguments.

- [ ] **Step 3: Implement and validate launcher arguments**

Add value capture, usage text, both `--name value` and `--name=value` parsing, exact-value validation, environment forwarding, and specialized defaults. Remove the specialized launcher's fixed output namespace so the Web app can isolate variants dynamically.

- [ ] **Step 4: Verify the launcher tests pass**

Run: `pytest -q tests/test_selectable_camera_qc.py -k launcher`

Expected: all selected tests pass.

- [ ] **Step 5: Commit the launcher change**

Run: `git add scripts/collection/collect_mobile_pipeline_qc_web.sh scripts/collection/collect_mobile_pipeline_qc_web_four_camera.sh tests/test_selectable_camera_qc.py && git commit -m "feat: configure selectable camera QC launcher"`

### Task 2: Strict camera layouts and export names

**Files:**
- Modify: `scripts/embodied_data_pipeline-main/mcap_conversion/scripts/camera_layouts.py`
- Modify: `scripts/embodied_data_pipeline-main/mcap_conversion/scripts/mcap_to_icra_episode.py`
- Modify: `scripts/embodied_data_pipeline-main/scripts/convert_mcap_dataset.py`
- Modify: `scripts/embodied_data_pipeline-main/robot_profiles/aloha_four_camera.yaml`
- Test: `tests/test_selectable_camera_qc.py`

**Interfaces:**
- Produces: layouts `three_camera_front`, `three_camera_global`, and `four_camera`; all conversion parsers accept them; the canonical four-camera profile exposes the exact four LeRobot keys.

- [ ] **Step 1: Write failing layout/profile tests**

Assert front/global video sources and required timestamp keys, legacy layout compatibility, parser choices, and exact `raw_key` to `lerobot_key` mappings in the four-camera profile.

- [ ] **Step 2: Verify the layout/profile tests fail**

Run: `pytest -q tests/test_selectable_camera_qc.py -k 'layout or profile_mapping or parser'`

Expected: missing layout names and old `head_color`/`head` LeRobot keys.

- [ ] **Step 3: Add the strict layouts and canonical naming**

Define the two strict three-camera `CameraLayout` values, register them, use `CAMERA_LAYOUTS` for argparse choices, and change profile mappings to `hand_head_color` and `global_color`.

- [ ] **Step 4: Verify layout/profile tests pass**

Run: `pytest -q tests/test_selectable_camera_qc.py -k 'layout or profile_mapping or parser'`

Expected: all selected tests pass.

- [ ] **Step 5: Commit layout and profile changes**

Run: `git add scripts/embodied_data_pipeline-main/mcap_conversion/scripts scripts/embodied_data_pipeline-main/scripts/convert_mcap_dataset.py scripts/embodied_data_pipeline-main/robot_profiles/aloha_four_camera.yaml tests/test_selectable_camera_qc.py && git commit -m "feat: define selectable camera export contracts"`

### Task 3: Web configuration and derived profiles

**Files:**
- Modify: `scripts/embodied_data_pipeline-main/scripts/pipeline_web_app.py`
- Test: `tests/test_selectable_camera_qc.py`

**Interfaces:**
- Produces: `resolve_camera_variant(payload)`, `write_camera_variant_profile(base_profile, camera_count, head_source)`, and `derive_paths(payload)` fields `camera_variant_selectable`, `camera_count`, `head_camera_source`, `camera_layout`, and `camera_feature_keys`.
- Consumes: Task 2 layout names and canonical profile.

- [ ] **Step 1: Write failing configuration tests**

Load `pipeline_web_app.py` with temporary environment roots and assert default four mode, both three-camera modes, isolated namespaces, exact derived camera mappings/required cameras, and rejection of invalid values.

- [ ] **Step 2: Verify configuration tests fail**

Run: `pytest -q tests/test_selectable_camera_qc.py -k 'derive or variant'`

Expected: missing configuration fields/functions or unsupported layout errors.

- [ ] **Step 3: Implement resolution and derived profile generation**

Resolve validated payload/environment values, choose the layout and namespace, atomically write deterministic JSON-compatible profiles under `WEB_LOG_ROOT/camera_profiles`, filter cameras, remap the selected head, and update annotation camera references.

- [ ] **Step 4: Add a failing Docker command test**

Assert that a generated profile outside `robot_profiles/` is mounted read-only and referenced by its container path.

- [ ] **Step 5: Implement Docker profile mounting and run tests**

Add a helper that reuses `/workspace/robot_profiles/...` for stock profiles and creates a dedicated bind mount for generated profiles. Run: `pytest -q tests/test_selectable_camera_qc.py -k 'derive or variant or docker'`.

Expected: all selected tests pass.

- [ ] **Step 6: Commit Web configuration changes**

Run: `git add scripts/embodied_data_pipeline-main/scripts/pipeline_web_app.py tests/test_selectable_camera_qc.py && git commit -m "feat: derive selectable camera pipeline profiles"`

### Task 4: Web selectors and end-to-end regression

**Files:**
- Modify: `scripts/embodied_data_pipeline-main/scripts/pipeline_web_app.py`
- Test: `tests/test_selectable_camera_qc.py`

**Interfaces:**
- Consumes: Task 3 status payload fields.
- Produces: specialized-only camera-count and head-source selectors whose values are included in all operation payloads.

- [ ] **Step 1: Write failing UI contract tests**

Assert the HTML contains both selectors, starts them hidden, sends values only after the server enables camera selection, disables head selection for four cameras, and reports the resolved image keys.

- [ ] **Step 2: Verify UI tests fail**

Run: `pytest -q tests/test_selectable_camera_qc.py -k ui`

Expected: missing selector element IDs and payload keys.

- [ ] **Step 3: Implement the Web controls**

Add the hidden control row, synchronize it from `/api/status`, update its enabled state on changes, clear a stale profile override, and refresh derived paths before starting work.

- [ ] **Step 4: Run focused and existing camera tests**

Run: `pytest -q tests/test_selectable_camera_qc.py tests/test_four_camera_qc_pipeline.py`

Expected: all tests pass after updating existing assertions to the new requested contract.

- [ ] **Step 5: Run syntax and regression verification**

Run: `bash -n scripts/collection/collect_mobile_pipeline_qc_web.sh scripts/collection/collect_mobile_pipeline_qc_web_four_camera.sh`

Run: `python3 -m py_compile scripts/embodied_data_pipeline-main/scripts/pipeline_web_app.py scripts/embodied_data_pipeline-main/mcap_conversion/scripts/camera_layouts.py scripts/embodied_data_pipeline-main/mcap_conversion/scripts/mcap_to_icra_episode.py scripts/embodied_data_pipeline-main/scripts/convert_mcap_dataset.py`

Run: `pytest -q tests/test_selectable_camera_qc.py tests/test_four_camera_qc_pipeline.py tests/test_pipeline_qc_web_script.py`

Expected: shell/Python syntax succeeds and all selected tests pass.

- [ ] **Step 6: Commit UI and regression changes**

Run: `git add scripts/embodied_data_pipeline-main/scripts/pipeline_web_app.py tests/test_selectable_camera_qc.py tests/test_four_camera_qc_pipeline.py && git commit -m "feat: add camera mode controls to QC web"`

### Task 5: Review and integration

**Files:**
- Review all files changed by Tasks 1-4.

**Interfaces:**
- Produces: reviewed, verified feature branch integrated without losing pre-existing user deletions.

- [ ] **Step 1: Review the diff against the design**

Run: `git diff main...HEAD --check && git diff --stat main...HEAD` and inspect all functional changes for exact key names, validation, path isolation, and generic-launcher compatibility.

- [ ] **Step 2: Run the final verification suite**

Repeat Task 4 syntax checks and focused pytest suite, plus any test named by review feedback.

- [ ] **Step 3: Integrate safely**

Preserve the main worktree's existing deleted test files, fast-forward or cherry-pick only the reviewed feature commits, and confirm `git status --short` still shows those same user-owned deletions plus no unintended files.

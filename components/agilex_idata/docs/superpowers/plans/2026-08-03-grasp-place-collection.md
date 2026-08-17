# Grasp and Place Two-Phase Collection Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an opt-in launcher that records paired grasp and place captures into separate configuration directories under the same episode number.

**Architecture:** A new wrapper enables a two-phase mode in the existing staged web launcher. Pure directory, event-routing, and review-result rules live in a small Python module, while the existing ROS bridge performs recording, saving, review waits, and safe `/config` switches. Legacy launchers never enable the mode and retain their existing path.

**Tech Stack:** Bash, Python 3, ROS 2 `rclpy`, `std_msgs/msg/Bool`, pytest/unittest.

## Global Constraints

- The grasp directory is the exact dataset directory supplied by the operator, such as `/home/agilex/data/stage2_twohand/20260803_scene1`.
- The paired place directory is `/home/agilex/data/place/<grasp-directory-basename>` unless `GRASP_PLACE_DATA_ROOT` overrides the root.
- Grasp and place captures use the same episode number.
- Both captures require an independent A/B/F review.
- Discard retries only the discarded phase and does not advance the logical episode.
- `/data_collection/start=true` is published only after recording status is confirmed.
- Existing fixed-stage and staged launchers must keep their current behavior.

---

## File Structure

- Create `scripts/collection/collect_mobile_pipeline_web_grasp_place.sh`: user-facing launcher and mode defaults.
- Create `scripts/collection/grasp_place_collection.py`: pure directory, event, and review-state decisions.
- Modify `scripts/collection/collect_mobile_pipeline_web_staged.sh`: opt-in configuration, ROS subscriptions, HTTP directory switching, status/UI text, and two-phase runtime flow.
- Create `tests/test_grasp_place_collection.py`: wrapper, pure-state, embedded-Python, and compatibility tests.

### Task 1: New launcher contract

**Files:**
- Create: `tests/test_grasp_place_collection.py`
- Create: `scripts/collection/collect_mobile_pipeline_web_grasp_place.sh`

**Interfaces:**
- Consumes: the existing `collect_mobile_pipeline_web_staged.sh` CLI.
- Produces: environment variables `GRASP_PLACE_COLLECTION_ENABLE=1`, `GRASP_PLACE_DATA_ROOT`, `GRASP_PLACE_START_TOPIC`, `GRASP_PLACE_GRASP_END_TOPIC`, `STATE_MACHINE_START_TOPIC`, `STATE_MACHINE_END_TOPIC`, and `DATA_COLLECTION_START_TOPIC`.

- [ ] **Step 1: Write the failing wrapper test**

Create a temporary staged-script stub that records forwarded arguments and the produced environment. Invoke the new wrapper with:

```python
["/home/agilex/data/stage2_twohand/20260803_scene1", "7", "--grade", "A"]
```

Assert exact forwarding plus these defaults:

```text
GRASP_PLACE_COLLECTION_ENABLE=1
GRASP_PLACE_DATA_ROOT=/home/agilex/data/place
GRASP_PLACE_START_TOPIC=/state_place/start
GRASP_PLACE_GRASP_END_TOPIC=/state_machine/grasping/end
STATE_MACHINE_START_TOPIC=/state_machine/start
STATE_MACHINE_END_TOPIC=/state_machine/end
DATA_COLLECTION_START_TOPIC=/data_collection/start
COLLECTION_STAGED_CAPTURE_DEFAULT=0
STATE_MACHINE_STAGED_DEFAULT=0
```

- [ ] **Step 2: Run the test and verify RED**

Run `pytest -q tests/test_grasp_place_collection.py -k wrapper`.

Expected: failure because `collect_mobile_pipeline_web_grasp_place.sh` does not exist.

- [ ] **Step 3: Implement the minimal wrapper**

Follow the fixed-stage wrapper pattern: parse only `-h/--help`, collect all other arguments unchanged, validate the staged script exists, export the two-phase defaults without overwriting explicit environment overrides, and `exec bash` the staged script.

```bash
#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STAGED_SCRIPT="${COLLECT_MOBILE_PIPELINE_WEB_STAGED_SCRIPT:-${SCRIPT_DIR}/collect_mobile_pipeline_web_staged.sh}"
FORWARD_ARGS=()

while [ "$#" -gt 0 ]; do
    case "$1" in
        -h|--help)
            exec bash "${STAGED_SCRIPT}" --help
            ;;
        *)
            FORWARD_ARGS+=("$1")
            shift
            ;;
    esac
done

[ -f "${STAGED_SCRIPT}" ] || {
    echo "[错误] 找不到全阶段采集脚本: ${STAGED_SCRIPT}" >&2
    exit 1
}

export GRASP_PLACE_COLLECTION_ENABLE=1
export GRASP_PLACE_DATA_ROOT="${GRASP_PLACE_DATA_ROOT:-/home/agilex/data/place}"
export GRASP_PLACE_START_TOPIC="${GRASP_PLACE_START_TOPIC:-/state_place/start}"
export GRASP_PLACE_GRASP_END_TOPIC="${GRASP_PLACE_GRASP_END_TOPIC:-/state_machine/grasping/end}"
export COLLECTION_STAGED_CAPTURE_DEFAULT=0
export STATE_MACHINE_STAGED_DEFAULT=0
export STATE_MACHINE_BRIDGE_ENABLE=1
export STATE_MACHINE_AUTO_ACTIVE_DEFAULT=1
export STATE_MACHINE_START_TOPIC="${STATE_MACHINE_START_TOPIC:-/state_machine/start}"
export STATE_MACHINE_END_TOPIC="${STATE_MACHINE_END_TOPIC:-/state_machine/end}"
export DATA_COLLECTION_START_TOPIC="${DATA_COLLECTION_START_TOPIC:-/data_collection/start}"

exec bash "${STAGED_SCRIPT}" "${FORWARD_ARGS[@]}"
```

- [ ] **Step 4: Verify GREEN and shell syntax**

Run:

```bash
pytest -q tests/test_grasp_place_collection.py -k wrapper
bash -n scripts/collection/collect_mobile_pipeline_web_grasp_place.sh
```

Expected: wrapper test passes and `bash -n` exits 0.

- [ ] **Step 5: Commit the launcher slice**

```bash
git add tests/test_grasp_place_collection.py scripts/collection/collect_mobile_pipeline_web_grasp_place.sh
git commit -m "feat: add grasp place collection launcher"
```

### Task 2: Pure two-phase rules

**Files:**
- Modify: `tests/test_grasp_place_collection.py`
- Create: `scripts/collection/grasp_place_collection.py`

**Interfaces:**
- Produces: `derive_place_data_dir(grasp_data_dir: str, place_root: str) -> str`.
- Produces: `two_phase_event_action(capture_phase: str, runtime_phase: str, event: str) -> str` returning `start_grasp`, `save_grasp`, `start_place`, `save_place`, or `ignore`.
- Produces: `review_outcome(status: dict, saved_episode: int | str) -> str` returning `accepted`, `discarded`, or `incomplete`.

- [ ] **Step 1: Write failing pure-function tests**

Cover this directory mapping:

```python
assert derive_place_data_dir(
    "/home/agilex/data/stage2_twohand/20260803_scene1/",
    "/home/agilex/data/place",
) == "/home/agilex/data/place/20260803_scene1"
```

Cover all four valid event/state combinations and representative out-of-order events. Cover accepted review (`last_saved_episode=N`, `episode=N+1`), discarded review (empty `last_saved_episode`, `episode=N`), and still-pending review.

- [ ] **Step 2: Run tests and verify RED**

Run `pytest -q tests/test_grasp_place_collection.py -k 'directory or event or review'`.

Expected: import failure because `grasp_place_collection.py` does not exist.

- [ ] **Step 3: Implement the pure rules**

Use normalized paths and reject empty/root configuration names. Implement event routing from an explicit mapping table. Treat a review as accepted only when capture and review are both inactive, the saved episode matches, and the current episode advanced; treat it as discarded only when both are inactive, the saved marker is empty, and the current episode did not advance.

```python
from __future__ import annotations

import os


def derive_place_data_dir(grasp_data_dir: str, place_root: str) -> str:
    grasp = os.path.normpath(os.path.expanduser(str(grasp_data_dir).strip()))
    root = os.path.normpath(os.path.expanduser(str(place_root).strip()))
    config_name = os.path.basename(grasp)
    if not config_name or config_name in {os.path.sep, ".", ".."}:
        raise ValueError(f"invalid grasp configuration directory: {grasp_data_dir!r}")
    if not root or root == ".":
        raise ValueError(f"invalid place data root: {place_root!r}")
    return os.path.join(root, config_name)


def two_phase_event_action(capture_phase: str, runtime_phase: str, event: str) -> str:
    return {
        ("grasp", "idle", "start"): "start_grasp",
        ("grasp", "recording", "grasp_end"): "save_grasp",
        ("place", "idle", "place_start"): "start_place",
        ("place", "recording", "end"): "save_place",
    }.get((capture_phase, runtime_phase, event), "ignore")


def review_outcome(status: dict, saved_episode: int | str) -> str:
    if status.get("capture_running") or status.get("quality_review_pending"):
        return "incomplete"
    saved = str(saved_episode)
    marker = str(status.get("last_saved_episode", "")).strip()
    current = str(status.get("episode", "")).strip()
    if marker == saved and current.isdigit() and saved.isdigit() and int(current) > int(saved):
        return "accepted"
    if not marker and current == saved:
        return "discarded"
    return "incomplete"
```

- [ ] **Step 4: Verify GREEN**

Run `pytest -q tests/test_grasp_place_collection.py -k 'directory or event or review'`.

Expected: all selected tests pass.

- [ ] **Step 5: Commit the rules slice**

```bash
git add tests/test_grasp_place_collection.py scripts/collection/grasp_place_collection.py
git commit -m "feat: model grasp place collection states"
```

### Task 3: Opt-in staged-bridge integration

**Files:**
- Modify: `tests/test_grasp_place_collection.py`
- Modify: `scripts/collection/collect_mobile_pipeline_web_staged.sh`

**Interfaces:**
- Consumes: Task 2 pure helpers.
- Produces: ROS subscriptions for `/state_machine/start`, `/state_machine/grasping/end`, `/state_place/start`, and `/state_machine/end` when two-phase mode is enabled.
- Produces: paired HTTP collection configuration switches using the current phase directory and logical episode.

- [ ] **Step 1: Write failing bridge integration tests**

Assert that the staged launcher:

- declares and logs the opt-in environment variables;
- derives the place directory with `derive_place_data_dir` after resolving `DATA_DIR`;
- passes two-phase settings into the embedded bridge;
- subscribes to grasp-end and place-start only in two-phase mode;
- does not install ordinary staged-boundary subscriptions in two-phase mode;
- uses `two_phase_event_action` to route the four control events;
- configures the place directory at episode `N` after accepted grasp review;
- configures the grasp directory at `N+1` after accepted place review;
- reconfigures the same phase and episode after a discard;
- keeps the existing fixed-stage wrapper defaults unchanged;
- has valid Bash syntax and compilable Python heredocs.

- [ ] **Step 2: Run focused tests and verify RED**

Run `pytest -q tests/test_grasp_place_collection.py`.

Expected: failures for missing staged-bridge wiring.

- [ ] **Step 3: Add opt-in configuration and status fields**

Resolve `GRASP_PLACE_COLLECTION_ENABLE`, the two special topics, grasp directory, place root, and derived place directory after normal CLI parsing. Add them to help/config output and the automation control JSON. Expose `two_phase_enabled`, `capture_phase`, `grasp_end_topic`, and `place_start_topic` through `/automation/status`; disable the staged-mode toggle while two-phase mode is active.

- [ ] **Step 4: Add explicit target reconfiguration**

Extend `collection_config_payload` with optional explicit `data_dir` and `episode_index` values while preserving its current two-argument behavior. When switching between default dataset roots, update default LeRobot target paths/names to the paired directory; preserve explicitly customized LeRobot destinations. Add `configure_collection_for_target` that refuses to switch while recording or review is pending and waits until `/status` confirms both directory and episode.

```python
def configure_collection_for_target(
    http: CollectionHttp,
    *,
    data_dir: str,
    episode_index: int,
) -> dict:
    status = http.status()
    if status.get("capture_running") or status.get("quality_review_pending"):
        raise RuntimeError("cannot switch collection target while capture or review is active")
    http.post_json(
        "/config",
        collection_config_payload(
            status,
            False,
            data_dir=data_dir,
            episode_index=episode_index,
        ),
    )
    return wait_for_status(
        http,
        lambda item: (
            not item.get("capture_running")
            and not item.get("quality_review_pending")
            and os.path.normpath(str(item.get("data_dir", ""))) == os.path.normpath(data_dir)
            and str(item.get("episode")) == str(episode_index)
        ),
        8.0,
        f"collection target {data_dir}/episode{episode_index}",
    )
```

- [ ] **Step 5: Add ROS subscriptions and two-phase event handling**

In two-phase mode, subscribe to grasp-end and place-start and skip ordinary staged-boundary subscriptions. At runtime:

```text
start_grasp -> configure grasp/N -> start non-staged capture -> publish ready
save_grasp accepted -> save/review -> configure place/N -> wait place start
save_grasp discarded -> configure grasp/N -> wait grasp start
start_place -> configure place/N -> start non-staged capture -> publish ready
save_place accepted -> save/review -> configure grasp/N+1 -> wait grasp start
save_place discarded -> configure place/N -> wait place start
```

All invalid events update diagnostic status without changing phase. Both save paths publish ready false before waiting for review. Keep the legacy event handlers under the disabled two-phase branch.

The event loop uses the pure action and review result as its only transition decisions:

```python
action = two_phase_event_action(capture_phase, phase, event)
if action in {"start_grasp", "start_place"}:
    target_dir = grasp_data_dir if capture_phase == "grasp" else place_data_dir
    configure_collection_for_target(http, data_dir=target_dir, episode_index=logical_episode)
    start_recording(f"收到双阶段开始事件 {event}", event, False)
elif action in {"save_grasp", "save_place"}:
    saved_episode = logical_episode
    saved_status = save_current_episode(event)
    outcome = review_outcome(saved_status, saved_episode)
    if outcome == "accepted" and capture_phase == "grasp":
        capture_phase = "place"
    elif outcome == "accepted":
        capture_phase = "grasp"
        logical_episode += 1
    target_dir = grasp_data_dir if capture_phase == "grasp" else place_data_dir
    configure_collection_for_target(http, data_dir=target_dir, episode_index=logical_episode)
    phase = "idle"
```

- [ ] **Step 6: Verify focused and legacy tests**

Run:

```bash
pytest -q \
  tests/test_grasp_place_collection.py \
  tests/test_fixed_stage_collection_script.py \
  tests/test_collection_failure_ui.py \
  tests/test_collection_target_dropdowns.py \
  tests/test_collection_auto_grade.py
bash -n scripts/collection/collect_mobile_pipeline_web_staged.sh
```

Expected: all tests pass and shell syntax exits 0.

- [ ] **Step 7: Commit the integration slice**

```bash
git add tests/test_grasp_place_collection.py scripts/collection/collect_mobile_pipeline_web_staged.sh
git commit -m "feat: record paired grasp and place episodes"
```

### Task 4: Final regression and operator handoff

**Files:**
- Verify: all files from Tasks 1-3.

**Interfaces:**
- Produces: a verified launcher command and a documented hardware smoke-test boundary.

- [ ] **Step 1: Run final focused regression**

```bash
pytest -q \
  tests/test_grasp_place_collection.py \
  tests/test_fixed_stage_collection_script.py \
  tests/test_collection_failure_ui.py \
  tests/test_collection_target_dropdowns.py \
  tests/test_collection_auto_grade.py \
  tests/test_four_rgb_collection.py
bash -n scripts/collection/collect_mobile_pipeline_web_grasp_place.sh
bash -n scripts/collection/collect_mobile_pipeline_web_staged.sh
git diff --check HEAD~3..HEAD
```

Expected: zero failed tests, both shell checks exit 0, and no whitespace errors.

- [ ] **Step 2: Inspect the final diff and working tree**

```bash
git diff --stat HEAD~3..HEAD
git status --short
```

Confirm that only the two scripts, one helper module, and one test file changed during implementation.

- [ ] **Step 3: Record the production smoke-test command**

Use a disposable configuration directory and start index before production, for example:

```bash
bash scripts/collection/collect_mobile_pipeline_web_grasp_place.sh \
  /home/agilex/data/stage2_twohand/20260803_scene1 \
  0
```

Do not publish ROS start/end topics automatically during verification because that would operate the live collection system.

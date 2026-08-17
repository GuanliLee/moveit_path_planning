# Early Place-Start Latch Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Preserve one `/state_place/start=true` received during grasp save/review and consume it once after an accepted grasp review so place recording starts without a second publication.

**Architecture:** Add two pure decision helpers to the existing grasp/place rules module, then keep one `pending_place_start` boolean in the embedded bridge coordinator. Review-window events set the boolean; an accepted grasp transition queues one fresh `place/idle` event after target configuration, while a discarded grasp transition clears it.

**Tech Stack:** Bash launcher with embedded Python 3, ROS2 Humble `rclpy`, `std_msgs/msg/Bool`, pytest.

## Global Constraints

- Only a `place_start` associated with the current grasp review may be latched.
- An accepted grasp review may consume the latch only after the place directory and same episode are confirmed.
- A discarded grasp review must clear the latch and retry grasp for the same episode.
- Existing post-review `/state_place/start`, legacy single-stage behavior, event-context checks, and `/data_collection/start` timing must remain unchanged.

---

### Task 1: Pure latch decisions

**Files:**
- Modify: `scripts/collection/grasp_place_collection.py`
- Test: `tests/test_grasp_place_collection.py`

**Interfaces:**
- Produces: `should_latch_place_start(event: str, pending_review_phase: str) -> bool`
- Produces: `should_release_latched_place_start(latched: bool, saved_phase: str, outcome: str, next_phase: str) -> bool`

- [ ] **Step 1: Write failing rule tests**

```python
def test_place_start_latch_is_scoped_to_grasp_review_and_released_only_on_accept():
    rules = load_rules_module()
    assert rules.should_latch_place_start("place_start", "grasp") is True
    assert rules.should_latch_place_start("end", "grasp") is False
    assert rules.should_latch_place_start("place_start", "place") is False
    assert rules.should_release_latched_place_start(True, "grasp", "accepted", "place") is True
    assert rules.should_release_latched_place_start(True, "grasp", "discarded", "grasp") is False
    assert rules.should_release_latched_place_start(True, "place", "accepted", "grasp") is False
```

- [ ] **Step 2: Run the focused test and confirm RED**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/agilex/openpi/.venv/bin/python -m pytest -q tests/test_grasp_place_collection.py -k place_start_latch`

Expected: FAIL because the two rule functions do not exist.

- [ ] **Step 3: Implement the two decisions**

```python
def should_latch_place_start(event: str, pending_review_phase: str) -> bool:
    return event == "place_start" and pending_review_phase == "grasp"


def should_release_latched_place_start(
    latched: bool,
    saved_phase: str,
    outcome: str,
    next_phase: str,
) -> bool:
    return bool(
        latched
        and saved_phase == "grasp"
        and outcome == "accepted"
        and next_phase == "place"
    )
```

- [ ] **Step 4: Run the focused test and confirm GREEN**

Run the Step 2 command.

Expected: PASS.

- [ ] **Step 5: Commit the pure rules**

```bash
git add scripts/collection/grasp_place_collection.py tests/test_grasp_place_collection.py
git commit -m "feat: model early place start latch"
```

### Task 2: Bridge latch and one-shot delivery

**Files:**
- Modify: `scripts/collection/collect_mobile_pipeline_web_staged.sh`
- Test: `tests/test_grasp_place_collection.py`

**Interfaces:**
- Consumes: the two Task 1 decision helpers.
- Produces: bridge-local `pending_place_start: bool` and one synthetic `("place_start", "place", "idle")` queue event after accepted grasp review.

- [ ] **Step 1: Add failing bridge wiring assertions**

```python
def test_bridge_latches_review_place_start_and_requeues_it_after_accept():
    text = STAGED.read_text(encoding="utf-8")
    assert "pending_place_start = False" in text
    assert "should_latch_place_start(" in text
    assert "should_release_latched_place_start(" in text
    assert 'node.events.put(("place_start", capture_phase, phase))' in text
```

- [ ] **Step 2: Run the bridge test and confirm RED**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/agilex/openpi/.venv/bin/python -m pytest -q tests/test_grasp_place_collection.py -k bridge_latches_review_place_start`

Expected: FAIL because the bridge has no latch state or requeue path.

- [ ] **Step 3: Wire the latch into review handling**

Implement these exact behaviors in the embedded bridge:

```python
pending_place_start = False

# At the start of every grasp save/review cycle.
if capture_phase == "grasp":
    pending_place_start = False

# For queued review events and events drained during the target switch.
if should_latch_place_start(event_name, pending_review_phase):
    pending_place_start = True

# After target configuration and transition to the new idle state.
release_place_start = should_release_latched_place_start(
    pending_place_start,
    saved_phase,
    outcome,
    next_capture_phase,
)
pending_place_start = False
if release_place_start:
    node.events.put(("place_start", capture_phase, phase))
```

The synthetic event must be queued only after `capture_phase`, `logical_episode`, `phase`, and the node event context have all changed to `place/idle`.

- [ ] **Step 4: Run bridge and focused regression tests**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/agilex/openpi/.venv/bin/python -m pytest -q \
  tests/test_grasp_place_collection.py \
  tests/test_fixed_stage_collection_script.py \
  tests/test_collection_failure_ui.py \
  tests/test_collection_target_dropdowns.py \
  tests/test_collection_auto_grade.py \
  tests/test_four_rgb_collection.py
bash -n scripts/collection/collect_mobile_pipeline_web_staged.sh
```

Expected: all focused tests pass and shell syntax exits 0.

- [ ] **Step 5: Commit the bridge behavior**

```bash
git add scripts/collection/collect_mobile_pipeline_web_staged.sh tests/test_grasp_place_collection.py
git commit -m "fix: latch early place start through grasp review"
```

### Task 3: End-to-end verification and integration

**Files:**
- Verify: `scripts/collection/collect_mobile_pipeline_web_grasp_place.sh`
- Verify: `scripts/collection/collect_mobile_pipeline_web_staged.sh`

**Interfaces:**
- Consumes: the completed latch rules and bridge behavior.
- Produces: verified local main commit with no active verification processes.

- [ ] **Step 1: Run isolated ROS2 happy-path verification**

Use an isolated ROS domain and fake HTTP collection service. Publish `place_start` once while grasp review is pending, accept the grasp, do not publish it again, and assert place capture reaches `recording` and publishes ready.

- [ ] **Step 2: Run isolated ROS2 discard verification**

Publish `place_start` during grasp review, discard the grasp, and assert the bridge returns to `grasp/idle` without calling place start. Retry grasp, accept it without a new early place signal, and assert it remains `place/idle` until a fresh place signal arrives.

- [ ] **Step 3: Run final checks**

```bash
bash -n scripts/collection/collect_mobile_pipeline_web_staged.sh
bash -n scripts/collection/collect_mobile_pipeline_web_grasp_place.sh
/home/agilex/openpi/.venv/bin/python -m py_compile \
  scripts/collection/grasp_place_collection.py \
  tests/test_grasp_place_collection.py
git diff --check
```

Expected: every command exits 0.

- [ ] **Step 4: Request code review, address Critical/Important findings, and merge locally**

Review the complete feature range against `docs/superpowers/specs/2026-08-03-latch-place-start-during-review-design.md`, then fast-forward the approved branch into local `main`. Preserve all pre-existing dirty main-worktree files.

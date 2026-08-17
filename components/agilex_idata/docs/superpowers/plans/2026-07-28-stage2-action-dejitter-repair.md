# Stage2 Action Periodic-Jitter Repair Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build, verify, and run a reversible repair that removes only high-confidence periodic action source-mixing from all scene1–11 `three_camera_global` HDF5 data and synchronizes existing LeRobot derivatives.

**Architecture:** A pure numerical detector creates an immutable repair plan from HDF5 action/state arrays. A transactional persistence layer backs up every affected file, patches HDF5 and mapped LeRobot Parquet through verified temporary files and atomic replacement, and can verify or roll back the run. A thin CLI exposes audit, plan, apply, verify, and rollback without putting repair logic in the Web application.

**Tech Stack:** Python 3.11, NumPy, h5py, PyArrow, pytest, SHA-256 manifests, HDF5 frame-group schema, LeRobot v2.1 Parquet.

## Global Constraints

- Scope is every path matching `scene1` through `scene11` recursively under `three_camera_global/hdf5_episodes/*/episode*/states/aligned_joints.h5`; include `scene8/stage2_new2`; exclude four-camera, three-camera-front, and scene12.
- Continuous all-zero action is valid. Absolute zero, near-zero, or stationarity alone must never select a frame.
- Detection constants are: 60-frame window, 11-frame median baseline, residual RMS at least `0.005 rad`, baseline RMS velocity at most `0.01 rad/frame`, dominant frequency `8–12 Hz`, spectral concentration at least `0.75`, at least six same-phase reversible excursions, per-joint contribution at least `0.001 rad` on two joints, endpoint velocity at most `0.11 rad/frame`, return ratio at most `0.25`, and state-distance ratio at most `0.35`.
- Modify only planned elements in `action/joint/position` and the matching action effector mirror. State, timestamps, frame count, videos, task metadata, quality grades, and episode numbering are immutable.
- Runs without two trusted anchors, runs of three or more frames, boundary runs, conflicting source direction, and excessive motion remain unchanged and are reported unresolved.
- Every apply requires verified physical backups, pre-apply digests, same-directory temporary files, `fsync`, atomic `os.replace`, post-apply verification, and complete rollback on failure.
- Existing mapped LeRobot data must be synchronized by changing only action dimensions 0–13 and per-episode action statistics; videos and all unrelated Parquet columns remain unchanged.
- Production apply is forbidden until scene10 proposes zero changes and known continuous-zero episodes propose zero changes.
- Use `/home/agilex/openpi/.venv/bin/python` for tests and production commands because it provides NumPy, h5py, PyArrow, and pytest together.
- Implement Tasks 1–4 only in an isolated Git worktree created from the plan
  commit. Require an initially clean worktree index; before every commit run
  `git diff --cached --name-only` and fail if it contains a path outside that
  task's declared file list. Never stage or commit the user's existing main
  worktree changes.
- At the beginning of every implementation task, require
  `git status --porcelain` to be empty. At its end, require every uncommitted
  path to be in that task's declared file list before staging.
- Reject a scoped file if it or any path component below the resolved root is a
  symlink, or if its resolved path is not relative to the resolved root.
- A reviewed plan is not trusted input by itself. Immediately before backups,
  `apply` must rebuild detection and mapping results from every source using the
  policy embedded in the plan and require canonical semantic equality with the
  supplied plan.

## Exact Detector Semantics

For side offset `o ∈ {0, 7}`, let `x[t] = action[t, o:o+6]` contain the six
revolute joints. The seventh value is the gripper: it participates in source
classification and is repaired with an accepted arm event, but is excluded
from geometric norms and FFT power because its scale and mechanics differ.

- Pad each joint sequence with five copies of each endpoint and set
  `b[t,j] = median(x[t-5:t+6,j])`. Set residual `r=x-b`.
- A raw excursion sample satisfies
  `RMS_j(r[t,j]) >= 0.005` and at least two revolute joints have
  `abs(r[t,j]) >= 0.001`. Group contiguous raw samples into runs.
- A run `[s,e]` has trusted anchors only when `s>0`, `e+1<n`, neither anchor is
  a raw excursion, and the endpoint baseline speed
  `RMS_j(x[e+1,j]-x[s-1,j])/(e-s+2) <= 0.11`. Runs at boundaries are
  `boundary`; runs longer than two frames are `run_too_long`.
- For a one- or two-frame run, the proposed value at `t` is the exact linear
  interpolation
  `p[t] = x[s-1] + (t-s+1)/(e-s+2) * (x[e+1]-x[s-1])`, extended to all seven
  side dimensions. The run departure is the maximum six-joint RMS of
  `x[t]-p[t]`. Its return ratio is
  `RMS_j(x[e+1]-x[s-1]) / max(departure, float64_eps)` and must be at most
  `0.25`.
- `source_kind=default_zero` only when every observed 7D run value is within
  `1e-12` of zero and zero samples occupy at most `0.35` of its qualifying
  60-frame window. `source_kind=state_like` only when
  `RMS_7(x[t]-state[t]) <= 0.35 * max(RMS_7(p[t]-state[t]), float64_eps)` for
  every run sample. Anything else is `ambiguous_source_direction` and cannot
  be repaired.
- Candidate event phase is `s % 3`. Within a 60-frame window, accept only a
  chain of at least six same-side, same-phase candidates whose consecutive
  starts differ by exactly 3 or 6 frames.
- For every 60-frame window, demean `r`, multiply by `np.hanning(60)`, run
  `np.fft.rfft` per revolute joint, and sum squared magnitudes across the six
  joints. Residual RMS is `sqrt(mean(r**2))`. Baseline velocity RMS is
  `sqrt(mean(diff(b, axis=0)**2))`. Select the maximum-power bin in 8–12 Hz;
  concentration is power within ±0.5 Hz of that bin divided by total power in
  3–14.5 Hz. All stated thresholds are inclusive. Zero denominator yields
  concentration zero.
- Slide all possible 60-frame windows in ascending start order. An event is
  accepted if any containing window passes all gates; record the earliest
  passing window. Episodes shorter than 60 frames cannot be repaired. Emit
  `short_episode` only when they contain a raw eligible source excursion.
- Merge duplicate discoveries by `(side, frame_indices)` and sort output by
  first frame then side. A frame may belong to at most one event. Conflicting
  source kinds or overlapping hypotheses are unresolved.

These formulas are the implementation contract. Threshold equality, one- and
two-frame interpolation, gripper-scale exclusion, empty/no-frame input,
non-finite input, and 59/60-frame boundary behavior require explicit tests.

## Durable Multi-file Transaction Protocol

The manifest is the write-ahead log and is atomically rewritten plus file and
parent-directory `fsync` at every transition:

```text
prepared → backing_up → backed_up → installing → committed
                                      ↘ rolling_back → rolled_back
```

1. `prepared` contains the canonical plan digest, root inventory, all physical
   targets, source digests, legal masks, deterministic backup paths, and
   affected batch roots before any file copy.
2. During `backing_up`, copy and hash one physical target at a time and persist
   its verified backup digest. No production file may change until all backups
   exist, all digests match, and state `backed_up` is durable.
3. Group logical updates by physical file. In particular, aggregate all
   episode rows sharing one Parquet or `episodes_stats.jsonl` file into one
   candidate and install that physical file exactly once.
4. Before each install, create and fully verify the same-directory candidate,
   persist its expected installed digest and `current_target` in state
   `installing`, recheck the current production digest against its source
   digest, then `os.replace` and directory-`fsync`. Rehash it and persist it in
   `installed_targets` before advancing.
5. A process restart never resumes mutation automatically. `apply` refuses an
   existing nonterminal run. `verify` diagnoses it. `rollback` first reconciles
   each target: source digest means not installed, expected installed digest
   means installed even if the crash preceded the manifest checkpoint, and any
   third digest is a conflict.
6. On any exception, persist `rolling_back` before restoration. Before changing
   anything, rollback performs a full compare-and-swap preflight: every target
   must equal either its original digest or this run's installed digest.
   A third digest aborts rollback without changing any file, preventing later
   external edits from being overwritten. There is no force flag. Restore
   installed files in reverse order through verified temporary copies, persist
   progress after each, verify all originals, then persist `rolled_back`.
7. `committed` is written only after every HDF5, Parquet, stats row, unrelated
   value, mapping, and backup check succeeds. A committed run remains
   roll-backable subject to the compare-and-swap rule.

---

### Task 0: Safely commit the plan and create an isolated implementation worktree

**Files:**
- Commit only: `docs/superpowers/plans/2026-07-28-stage2-action-dejitter-repair.md`
- Create ignored worktree: `/home/caizj/agilex_idata/.worktrees/action-jitter-repair`

- [ ] **Step 1: Capture the user's main-worktree state and verify the plan-only index**

Run:

```bash
cd /home/caizj/agilex_idata
ACTION_REPAIR_PLAN=docs/superpowers/plans/2026-07-28-stage2-action-dejitter-repair.md
git status --short
ACTION_REPAIR_MAIN_STATUS_BEFORE="$(
  git status --porcelain=v1 --untracked-files=all |
  awk -v plan="${ACTION_REPAIR_PLAN}" 'substr($0, 4) != plan'
)"
test -z "$(git diff --cached --name-only)"
git add -- "${ACTION_REPAIR_PLAN}"
test "$(git diff --cached --name-only)" = "${ACTION_REPAIR_PLAN}"
git diff --cached --check
```

The existing deleted tests and unrelated untracked plan remain untouched and
unstaged. If the initial index is nonempty, stop without `reset`, `restore`,
`stash`, or any index mutation and report the conflicting staged paths.

- [ ] **Step 2: Create and verify the plan-only commit**

Run:

```bash
git commit --only "${ACTION_REPAIR_PLAN}" \
  -m "docs: plan stage2 action jitter repair"
ACTION_REPAIR_PLAN_SHA="$(git rev-parse HEAD)"
test "$(
  git diff-tree --no-commit-id --name-only -r "${ACTION_REPAIR_PLAN_SHA}"
)" = "${ACTION_REPAIR_PLAN}"
git status --short
ACTION_REPAIR_MAIN_STATUS_AFTER="$(
  git status --porcelain=v1 --untracked-files=all |
  awk -v plan="${ACTION_REPAIR_PLAN}" 'substr($0, 4) != plan'
)"
test "${ACTION_REPAIR_MAIN_STATUS_AFTER}" = \
  "${ACTION_REPAIR_MAIN_STATUS_BEFORE}"
```

The exact porcelain comparison requires every pre-existing user change to
remain present with the same index/worktree status.

- [ ] **Step 3: Create the ignored implementation worktree**

Run:

```bash
git check-ignore -q .worktrees
test ! -e /home/caizj/agilex_idata/.worktrees/action-jitter-repair
git worktree add \
  /home/caizj/agilex_idata/.worktrees/action-jitter-repair \
  -b feature/action-jitter-repair \
  "${ACTION_REPAIR_PLAN_SHA}"
test -z "$(
  git -C /home/caizj/agilex_idata/.worktrees/action-jitter-repair status --porcelain
)"
```

- [ ] **Step 4: Run the clean-worktree baseline**

Run:

```bash
cd /home/caizj/agilex_idata/.worktrees/action-jitter-repair
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  /home/agilex/openpi/.venv/bin/python -m pytest -q tests
```

Record the exact baseline result before Task 1. A baseline failure must be
diagnosed and separated from this feature before proceeding.

---

### Task 1: Numerical detector and conservative repair mask

**Files:**
- Create: `scripts/embodied_data_pipeline-main/quality_pipeline/action_repair.py`
- Create: `tests/test_repair_action_jitter_hdf5.py`

**Interfaces:**
- Produces: `DetectionPolicy`, `ArmRepairEvent`, `UnresolvedEvent`, `DetectionResult`
- Produces: `detect_arm_repairs(actions: np.ndarray, states: np.ndarray, policy: DetectionPolicy = DetectionPolicy()) -> DetectionResult`
- Produces: `apply_events_to_actions(actions: np.ndarray, events: Sequence[ArmRepairEvent]) -> np.ndarray`

- [ ] **Step 1: Write failing detector tests**

Create synthetic 30 Hz arrays with a 7D arm helper and tests for valid zero, periodic zero-source mixing, periodic state-source mixing, isolated changes, smooth motion, and ambiguity:

```python
import importlib.util
from pathlib import Path
import sys

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = (
    REPO_ROOT
    / "scripts/embodied_data_pipeline-main/quality_pipeline/action_repair.py"
)


def load_module():
    spec = importlib.util.spec_from_file_location("action_repair", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def held_episode(frames=180, left=None, right=None):
    left_value = np.asarray(
        left if left is not None else [0.20, 0.40, -0.30, 0.10, -0.15, 0.25, 0.08],
        dtype=np.float64,
    )
    right_value = np.asarray(
        right if right is not None else [-0.15, 0.30, -0.20, 0.35, 0.12, -0.22, 0.09],
        dtype=np.float64,
    )
    actions = np.tile(np.concatenate([left_value, right_value]), (frames, 1))
    states = actions.copy()
    return actions, states


def test_continuous_zero_is_never_repaired():
    repair = load_module()
    actions, states = held_episode(left=np.zeros(7))
    result = repair.detect_arm_repairs(actions, states)
    assert result.events == ()
    assert np.array_equal(repair.apply_events_to_actions(actions, result.events), actions)


def test_periodic_single_frame_zero_excursions_are_interpolated():
    repair = load_module()
    clean, states = held_episode()
    corrupted = clean.copy()
    bad = np.arange(30, 120, 3)
    corrupted[bad, :7] = 0.0
    result = repair.detect_arm_repairs(corrupted, states)
    repaired = repair.apply_events_to_actions(corrupted, result.events)
    assert set(bad).issubset({idx for event in result.events for idx in event.frame_indices})
    assert np.allclose(repaired[bad, :7], clean[bad, :7])
    assert np.array_equal(repaired[:, 7:], corrupted[:, 7:])


def test_periodic_state_like_excursions_from_zero_baseline_are_removed():
    repair = load_module()
    clean, states = held_episode(left=np.zeros(7))
    states[:, :7] = np.array([0.03, 0.01, -0.02, 0.04, -0.01, 0.02, 0.06])
    corrupted = clean.copy()
    bad = np.arange(30, 120, 3)
    corrupted[bad, :7] = states[bad, :7]
    result = repair.detect_arm_repairs(corrupted, states)
    repaired = repair.apply_events_to_actions(corrupted, result.events)
    assert np.allclose(repaired[bad, :7], 0.0)


def test_isolated_or_fast_motion_changes_are_not_repaired():
    repair = load_module()
    actions, states = held_episode()
    actions[50, :6] += 0.2
    actions[:, :6] += np.linspace(0.0, 2.0, len(actions))[:, None]
    result = repair.detect_arm_repairs(actions, states)
    assert result.events == ()


def test_long_or_non_state_like_runs_are_unresolved():
    repair = load_module()
    actions, states = held_episode()
    for start in range(30, 120, 6):
        actions[start : start + 3, :7] = 0.0
    result = repair.detect_arm_repairs(actions, states)
    assert result.events == ()
    assert any(item.reason == "run_too_long" for item in result.unresolved)

    actions, states = held_episode()
    for frame_index in range(30, 120, 3):
        actions[frame_index, :6] += np.array([0.02, -0.02, 0.01, -0.01, 0.02, -0.02])
    result = repair.detect_arm_repairs(actions, states)
    assert result.events == ()
    assert any(item.reason == "ambiguous_source_direction" for item in result.unresolved)
```

- [ ] **Step 2: Run the focused tests and verify RED**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/agilex/openpi/.venv/bin/python -m pytest -q \
  tests/test_repair_action_jitter_hdf5.py
```

Expected: collection fails because `action_repair.py` does not exist.

- [ ] **Step 3: Implement the detector**

Implement immutable dataclasses with JSON-safe conversion:

```python
@dataclass(frozen=True)
class DetectionPolicy:
    window_frames: int = 60
    median_frames: int = 11
    residual_rms_min: float = 0.005
    baseline_velocity_rms_max: float = 0.01
    frequency_min_hz: float = 8.0
    frequency_max_hz: float = 12.0
    spectral_band_min_hz: float = 3.0
    spectral_band_max_hz: float = 14.5
    spectral_half_width_hz: float = 0.5
    spectral_concentration_min: float = 0.75
    min_phase_events: int = 6
    max_run_frames: int = 2
    excursion_l2_min: float = 0.005
    per_joint_min: float = 0.001
    min_changed_joints: int = 2
    endpoint_velocity_max: float = 0.11
    return_ratio_max: float = 0.25
    state_distance_ratio_max: float = 0.35
    zero_tolerance: float = 1e-12
    fps: float = 30.0


@dataclass(frozen=True)
class ArmRepairEvent:
    side: str
    frame_indices: tuple[int, ...]
    replacement: tuple[tuple[float, ...], ...]
    amplitude_l2: float
    dominant_frequency_hz: float
    spectral_concentration: float
    source_kind: str


@dataclass(frozen=True)
class UnresolvedEvent:
    side: str
    frame_indices: tuple[int, ...]
    reason: str


@dataclass(frozen=True)
class DetectionResult:
    events: tuple[ArmRepairEvent, ...]
    unresolved: tuple[UnresolvedEvent, ...]
```

The implementation must:

1. validate equal `(frames, 14)` finite arrays;
2. form an edge-padded 11-frame per-dimension rolling median;
3. enumerate one- and two-frame reversible run hypotheses with two anchors;
4. reject runs with fewer than two changed arm joints, excessive endpoint velocity, or excessive return ratio;
5. resolve zero/default versus state-like source direction using window zero duty and the `0.35` state-distance ratio;
6. group candidates by side and modulo-3 phase, requiring six events separated by 3 or 6 frames;
7. compute a 60-frame real FFT on the six-joint residual, select only 8–12 Hz peaks with concentration at least `0.75`, and enforce baseline velocity;
8. merge overlapping eligible windows without duplicating a frame;
9. emit unresolved reasons for boundary, long-run, motion, phase, spectral, and source-direction rejection;
10. interpolate each accepted run between anchors and change no other values.

- [ ] **Step 4: Run detector tests and verify GREEN**

Run the focused test command from Step 2.

Expected: all detector tests pass.

- [ ] **Step 5: Commit Task 1**

```bash
git add scripts/embodied_data_pipeline-main/quality_pipeline/action_repair.py \
  tests/test_repair_action_jitter_hdf5.py
test "$(git diff --cached --name-only | sort)" = "$(
  printf '%s\n' \
    scripts/embodied_data_pipeline-main/quality_pipeline/action_repair.py \
    tests/test_repair_action_jitter_hdf5.py |
  sort
)"
git commit -m "feat: detect periodic action source mixing"
```

---

### Task 2: HDF5 discovery, immutable plans, atomic apply, verify, and rollback

**Files:**
- Modify: `scripts/embodied_data_pipeline-main/quality_pipeline/action_repair.py`
- Modify: `tests/test_repair_action_jitter_hdf5.py`

**Interfaces:**
- Produces: `EpisodeRepairPlan`, `DatasetRepairPlan`, `ApplyReport`, `VerificationReport`
- Produces: `discover_scoped_hdf5(root: Path) -> tuple[Path, ...]`
- Produces: `build_dataset_plan(root: Path, policy: DetectionPolicy = DetectionPolicy()) -> DatasetRepairPlan`
- Produces: `write_plan(plan: DatasetRepairPlan, path: Path) -> None`
- Produces: `read_plan(path: Path) -> DatasetRepairPlan`
- Produces: `apply_dataset_plan(plan: DatasetRepairPlan, backup_root: Path, run_dir: Path | None = None) -> ApplyReport`
- Produces: `verify_manifest(manifest_path: Path) -> VerificationReport`
- Produces: `rollback_manifest(manifest_path: Path) -> VerificationReport`

`EpisodeRepairPlan` stores the source digest, frame count, exact changed-frame
before/replacement vectors, unresolved reasons, and detector policy. It does
not embed the full episode action array. The run manifest records `run_id`,
`plan_sha256`, `affected_batches`, every source/backup/installed digest, every
allowed mutation mask, and transaction state.

`DatasetRepairPlan` contains one immutable episode entry for every discovered
scoped HDF5, including unchanged episodes with empty event lists. Before making
any backup or mutation, `apply_dataset_plan` verifies every scoped source digest
and every mapped target digest, so a post-plan change anywhere in scope aborts
the transaction.

- [ ] **Step 1: Add failing HDF5 transaction tests**

Append helpers that create real frame-group HDF5 fixtures with root, group, and dataset attributes. Add tests asserting recursive scope, plan round-trip, mirrored gripper updates, unchanged object metadata, digest refusal, and rollback:

```python
import hashlib
import json

import h5py
import pytest


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def write_episode_h5(path, actions, states):
    path.parent.mkdir(parents=True, exist_ok=True)
    with h5py.File(path, "w") as output:
        output.attrs["fixture"] = "preserve"
        for index, (action, state) in enumerate(zip(actions, states)):
            frame = output.create_group(str(index))
            frame.attrs["frame_attr"] = index
            action_ds = frame.create_dataset(
                "action/joint/position",
                data=action,
                compression="gzip",
                compression_opts=1,
            )
            action_ds.attrs["unit"] = "rad"
            frame.create_dataset("action/left_effector/position", data=[action[6]])
            frame.create_dataset("action/right_effector/position", data=[action[13]])
            frame.create_dataset("state/joint/position", data=state)
            frame.create_dataset("main_timestamp", data=np.uint64(index * 33_333_333))


def test_apply_is_atomic_preserves_metadata_and_can_rollback(tmp_path):
    repair = load_module()
    root = tmp_path / "stage2_new"
    clean, states = held_episode()
    corrupted = clean.copy()
    corrupted[np.arange(30, 120, 3), :7] = 0.0
    h5_path = (
        root
        / "scene9/three_camera_global/hdf5_episodes/batch/episode48/states/aligned_joints.h5"
    )
    write_episode_h5(h5_path, corrupted, states)
    original_digest = sha256(h5_path)
    plan = repair.build_dataset_plan(root)
    report = repair.apply_dataset_plan(plan, tmp_path / "backups")
    verification = repair.verify_manifest(report.manifest_path)
    assert verification.ok
    with h5py.File(h5_path, "r") as repaired:
        assert repaired.attrs["fixture"] == "preserve"
        assert repaired["30/action/joint/position"].compression == "gzip"
        assert repaired["30/action/joint/position"].attrs["unit"] == "rad"
        assert repaired["30/action/left_effector/position"][0] == repaired["30/action/joint/position"][6]
    rolled_back = repair.rollback_manifest(report.manifest_path)
    assert rolled_back.ok
    assert sha256(h5_path) == original_digest


def test_apply_refuses_a_source_changed_after_planning(tmp_path):
    repair = load_module()
    root = tmp_path / "stage2_new"
    clean, states = held_episode()
    h5_path = (
        root
        / "scene9/three_camera_global/hdf5_episodes/batch/episode1/states/aligned_joints.h5"
    )
    write_episode_h5(h5_path, clean, states)
    plan = repair.build_dataset_plan(root)
    with h5py.File(h5_path, "r+") as changed:
        changed["0/action/joint/position"][0] += 1.0
    with pytest.raises(ValueError, match="digest changed"):
        repair.apply_dataset_plan(plan, tmp_path / "backups")
```

Add multi-file failure and crash-recovery coverage using `monkeypatch` around
the private atomic installer:

```python
def test_failure_after_first_install_rolls_every_file_back(tmp_path, monkeypatch):
    repair, plan, originals = build_two_h5_fixture(tmp_path)
    real_install = repair._install_verified_candidate
    calls = 0

    def fail_second(*args, **kwargs):
        nonlocal calls
        calls += 1
        if calls == 2:
            raise OSError("injected second install failure")
        return real_install(*args, **kwargs)

    monkeypatch.setattr(repair, "_install_verified_candidate", fail_second)
    with pytest.raises(OSError, match="injected"):
        repair.apply_dataset_plan(plan, tmp_path / "backups")
    manifest = only_manifest(tmp_path / "backups")
    assert read_json(manifest)["state"] == "rolled_back"
    assert {path: sha256(path) for path in originals} == originals


def test_rollback_refuses_to_overwrite_post_apply_external_change(tmp_path):
    repair, plan, _ = build_two_h5_fixture(tmp_path)
    report = repair.apply_dataset_plan(plan, tmp_path / "backups")
    changed_path = Path(report.installed_paths[0])
    with h5py.File(changed_path, "r+") as output:
        output["0/action/joint/position"][0] += 0.123
    before_refusal = sha256(changed_path)
    refused = repair.rollback_manifest(report.manifest_path)
    assert not refused.ok
    assert "external_change" in refused.errors
    assert sha256(changed_path) == before_refusal


def test_crash_checkpoint_is_reconciled_without_resuming_apply(tmp_path):
    repair, plan, _ = build_two_h5_fixture(tmp_path)
    manifest = build_manifest_crash_fixture(
        repair, plan, tmp_path / "backups", crash_after_replace=True
    )
    with pytest.raises(ValueError, match="nonterminal"):
        repair.apply_dataset_plan(
            plan, tmp_path / "backups", run_dir=manifest.parent
        )
    diagnosed = repair.verify_manifest(manifest)
    assert not diagnosed.ok
    assert diagnosed.state == "installing"
    assert repair.rollback_manifest(manifest).ok


def test_apply_rejects_semantically_edited_plan(tmp_path):
    repair, plan, _ = build_two_h5_fixture(tmp_path)
    edited = dataclasses.replace(
        plan,
        episodes=replace_first_replacement(plan.episodes, value=123.0),
    )
    with pytest.raises(ValueError, match="plan differs from fresh detection"):
        repair.apply_dataset_plan(edited, tmp_path / "backups")
```

Expose an optional private `fault_injector(checkpoint, physical_path)` argument
to the transaction engine, defaulting to `None` and never exposed by the CLI.
Use a child Python process whose injector calls `os._exit(86)` at
`after_backup_checkpoint`, `after_hdf5_replace_before_checkpoint`,
`after_parquet_replace_before_checkpoint`, and
`after_stats_replace_before_checkpoint`. For each crash, launch a new process
for `verify` and then `rollback`, and assert either complete original recovery
or a compare-and-swap conflict with no files changed. This tests actual durable
boundaries rather than synthesizing a manifest in memory.

- [ ] **Step 2: Run transaction tests and verify RED**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/agilex/openpi/.venv/bin/python -m pytest -q \
  tests/test_repair_action_jitter_hdf5.py -k \
  'atomic or rollback or digest or scoped or plan_round_trip'
```

Expected: failures report missing plan and transaction interfaces.

- [ ] **Step 3: Implement discovery and plan serialization**

Use exact discovery boundaries:

```python
def discover_scoped_hdf5(root: Path) -> tuple[Path, ...]:
    resolved = root.expanduser().resolve()
    found = set()
    for scene_number in range(1, 12):
        scene = resolved / f"scene{scene_number}"
        if not scene.is_dir():
            continue
        found.update(
            _resolve_regular_file_without_symlinks(path, resolved)
            for path in scene.rglob(
                "three_camera_global/hdf5_episodes/*/episode*/states/aligned_joints.h5"
            )
            if path.is_file()
        )
    return tuple(sorted(found, key=lambda path: natural_path_key(path.relative_to(resolved))))
```

Read numeric frame groups in integer order, stack only `action/joint/position`
and `state/joint/position`, call Task 1 detection, and store the source digest,
frame count, exact changed-frame before/replacement vectors, unresolved
reasons, and policy inside frozen plan objects. Do not serialize the complete
action sequence. JSON serialization uses sorted keys, two-space indentation,
UTF-8, newline termination, a same-directory temporary file, flush, `fsync`,
and `os.replace`.

`_resolve_regular_file_without_symlinks` rejects every symlink component and
requires the resolved result to satisfy `result.is_relative_to(resolved)`.
Discovery also audits near-matching `aligned_joints.h5` paths that fall outside
the one-batch/one-episode schema instead of silently accepting them.

HDF5 preflight requires integer frame groups exactly `0..n-1`; floating finite
`(14,)` action and state datasets at every frame; floating `(1,)` left/right
effector datasets; and pre-repair effector values bitwise equal to action
indices 6 and 13 after dtype conversion. Missing, duplicate, non-contiguous, or
wrong-shape frame data is a plan-construction error, not an unresolved repair.

- [ ] **Step 4: Implement physical backup and atomic HDF5 application**

Implement these private primitives and use them from `apply_dataset_plan`:

```python
def _copy_verified(source: Path, target: Path) -> str:
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, target)
    source_digest = sha256_file(source)
    if sha256_file(target) != source_digest:
        raise IOError(f"backup digest mismatch: {source}")
    return source_digest


def _build_verified_hdf5_candidate(
    entry: EpisodeRepairPlan, temp_suffix: str
) -> tuple[Path, str]:
    source = entry.h5_path
    fd, temp_name = tempfile.mkstemp(
        prefix=f".{source.name}.action_repair.",
        suffix=temp_suffix,
        dir=source.parent,
    )
    os.close(fd)
    temp_path = Path(temp_name)
    try:
        shutil.copy2(source, temp_path)
        with h5py.File(temp_path, "r+") as output:
            for event in entry.events:
                offset = 0 if event.side == "left" else 7
                effector = (
                    "action/left_effector/position"
                    if event.side == "left"
                    else "action/right_effector/position"
                )
                for frame_index, replacement in zip(event.frame_indices, event.replacement):
                    joint = output[f"{frame_index}/action/joint/position"]
                    joint[offset : offset + 7] = np.asarray(replacement, dtype=joint.dtype)
                    output[f"{frame_index}/{effector}"][0] = replacement[6]
            output.flush()
        _verify_hdf5_temp_against_plan(temp_path, entry)
        _fsync_file(temp_path)
        return temp_path, sha256_file(temp_path)
    except BaseException:
        temp_path.unlink(missing_ok=True)
        raise
```

The verifier must compare the backup and candidate object tree, every attribute,
dataset dtype/shape/chunks/compression/filter property, and every value outside
the allowed action slices. Any exception changes manifest state to
`rolling_back`, restores every installed file in reverse order through a
temporary copy plus `os.replace`, verifies original digests, and records
`rolled_back`.

Implement the full durable protocol above, including crash reconciliation and
rollback compare-and-swap tests. Before entering `backing_up`, rebuild a fresh
plan from `plan.root` and `plan.policy`, then compare canonical JSON after
excluding only the plan creation timestamp. Require exact equality of episode
paths, source digests, mappings, events, before values, replacements,
unresolved reasons, and inventory. This makes an edited plan fail even when the
source files themselves have not changed.

This candidate builder never installs. The WAL-controlled
`_install_verified_candidate` is the only function allowed to call
`os.replace` on a production target, and it must persist `installing` plus the
expected candidate digest before doing so.

- [ ] **Step 5: Run the full focused test module**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/agilex/openpi/.venv/bin/python -m pytest -q \
  tests/test_repair_action_jitter_hdf5.py
```

Expected: all Task 1 and Task 2 tests pass.

- [ ] **Step 6: Commit Task 2**

```bash
git add scripts/embodied_data_pipeline-main/quality_pipeline/action_repair.py \
  tests/test_repair_action_jitter_hdf5.py
test "$(git diff --cached --name-only | sort)" = "$(
  printf '%s\n' \
    scripts/embodied_data_pipeline-main/quality_pipeline/action_repair.py \
    tests/test_repair_action_jitter_hdf5.py |
  sort
)"
git commit -m "feat: add transactional HDF5 action repair"
```

---

### Task 3: LeRobot mapping preflight, Parquet synchronization, and statistics

**Files:**
- Modify: `scripts/embodied_data_pipeline-main/quality_pipeline/action_repair.py`
- Modify: `tests/test_repair_action_jitter_hdf5.py`

**Interfaces:**
- Produces: `LeRobotTarget`
- Extends: `build_dataset_plan()` with unique mapping and pre-repair Parquet checks
- Extends: `apply_dataset_plan()` and rollback with Parquet and `episodes_stats.jsonl`
- Produces: `verify_lerobot_target(target: LeRobotTarget, entry: EpisodeRepairPlan) -> tuple[str, ...]`

Each episode stores `lerobot_targets: tuple[LeRobotTarget, ...]`, so every
existing derivative is synchronized. Identical duplicate mapping rows are
deduplicated. Distinct source HDF5 files claiming the same physical
`(parquet_path, episode_index)` or `(stats_path, episode_index)` are fatal
collisions.

- [ ] **Step 1: Add failing LeRobot synchronization tests**

Create a real fixed-size-list float32 Parquet fixture with 18D action and schema
metadata, a unique mapping row, and `episodes_stats.jsonl`. Assert only action
0–13 and its stats change:

```python
import pyarrow as pa
import pyarrow.parquet as pq


def write_lerobot_fixture(dataset_root, source_h5, actions):
    data_path = dataset_root / "data/chunk-000/episode_000000.parquet"
    data_path.parent.mkdir(parents=True, exist_ok=True)
    full_action = np.concatenate(
        [actions.astype(np.float32), np.tile([200.0, 0.0, 0.0, 0.0], (len(actions), 1))],
        axis=1,
    )
    action_array = pa.FixedSizeListArray.from_arrays(
        pa.array(full_action.reshape(-1), type=pa.float32()),
        18,
    )
    table = pa.table(
        {
            "action": action_array,
            "timestamp": pa.array(np.arange(len(actions), dtype=np.float32) / 30.0),
            "frame_index": pa.array(np.arange(len(actions), dtype=np.int64)),
            "episode_index": pa.array(np.zeros(len(actions), dtype=np.int64)),
        }
    ).replace_schema_metadata({b"fixture": b"preserve"})
    pq.write_table(table, data_path, compression="snappy")
    meta = dataset_root / "meta"
    meta.mkdir(parents=True, exist_ok=True)
    mapping = {
        "episodes": [
            {
                "source_episode_name": source_h5.parents[1].name,
                "source_h5": str(source_h5.parents[1].relative_to(source_h5.parents[2])),
                "lerobot_episode_index": 0,
                "lerobot_episode_name": "episode_000000",
                "lerobot_data_file": "data/chunk-000/episode_000000.parquet",
            }
        ]
    }
    (meta / "episode_name_mapping.json").write_text(json.dumps(mapping), encoding="utf-8")
    stats = {
        "episode_index": 0,
        "stats": {
            "action": {
                "min": full_action.min(0).tolist(),
                "max": full_action.max(0).tolist(),
                "mean": full_action.mean(0).tolist(),
                "std": full_action.std(0).tolist(),
                "count": [len(full_action)],
            }
        },
    }
    (meta / "episodes_stats.jsonl").write_text(json.dumps(stats) + "\n", encoding="utf-8")
    return data_path


def test_lerobot_action_and_stats_follow_repaired_hdf5(tmp_path):
    repair = load_module()
    root = tmp_path / "stage2_new"
    clean, states = held_episode()
    corrupted = clean.copy()
    corrupted[np.arange(30, 120, 3), :7] = 0.0
    h5_path = (
        root
        / "scene9/three_camera_global/hdf5_episodes/batch/episode48/states/aligned_joints.h5"
    )
    write_episode_h5(h5_path, corrupted, states)
    lerobot = root / "scene9/three_camera_global/lerobot/batch/A"
    parquet = write_lerobot_fixture(lerobot, h5_path, corrupted)
    before = pq.read_table(parquet)
    plan = repair.build_dataset_plan(root)
    report = repair.apply_dataset_plan(plan, tmp_path / "backups")
    after = pq.read_table(parquet)
    assert after.schema.metadata == before.schema.metadata
    assert after.column("timestamp").equals(before.column("timestamp"))
    after_action = np.asarray(after.column("action").to_pylist(), dtype=np.float32)
    assert np.array_equal(after_action[:, 14:], np.asarray(before.column("action").to_pylist())[:, 14:])
    assert repair.verify_manifest(report.manifest_path).ok
```

- [ ] **Step 2: Run LeRobot tests and verify RED**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/agilex/openpi/.venv/bin/python -m pytest -q \
  tests/test_repair_action_jitter_hdf5.py -k lerobot
```

Expected: failure because plan entries do not yet resolve or synchronize
LeRobot targets.

- [ ] **Step 3: Implement unique mapping and preflight**

For every batch, inspect grade directories below
`three_camera_global/lerobot/<batch>/*/meta/episode_name_mapping.json`. Normalize
each mapping record's `source_h5` against the matching HDF5 batch root and
collect every unique target per source. While building the plan, keep the
current HDF5 action in a local `hdf5_actions` array only. For every mapped
affected HDF5, load the Parquet action column and require:

```python
episode_rows = np.asarray(table.column("episode_index")) == target.episode_index
parquet_action = np.asarray(table.column("action").to_pylist())[episode_rows]
parquet_action.shape[0] == entry.frame_count
parquet_action.shape[1] >= 14
np.allclose(
    parquet_action[:, :14],
    hdf5_actions,
    rtol=0.0,
    atol=1e-6,
)
```

Store mapping, Parquet, stats path, LeRobot episode index, original digests,
Arrow schema metadata, and per-column compression in `LeRobotTarget`. A
cross-source target collision, missing mapped file, row mismatch, action
mismatch, or unsupported schema makes plan construction fail before any backup
or mutation. Unmapped HDF5 entries remain valid with
`lerobot_targets=()`. At apply time,
source and target digests are checked against the plan before the current HDF5
actions are loaded again; no full pre-repair action array is needed in JSON.

Treat each grade directory as a canonical security boundary. Reject a grade,
mapping, stats, Parquet, or video path if it is absolute where a relative path
is required, contains `..`, contains any symlink component, is not a regular
file, or resolves outside that grade. Apply the same containment helper used
for HDF5. Add fixtures for `lerobot_data_file="../../outside.parquet"` and a
symlinked Parquet target; both must fail during plan construction without
creating backups.

For each affected mapped episode, discover videos only as regular files
matching `videos/**/<lerobot_episode_name>.*` beneath its grade. Record the
canonical relative path, size, and SHA-256 of every match in the plan and
manifest; an episode with no video records an explicit empty list. Recheck
these digests before backup, before each install, and at committed
verification. Add one video fixture whose digest must be unchanged after apply
and rollback.

- [ ] **Step 4: Implement atomic Parquet and statistics synchronization**

Read the post-repair 14D HDF5 action, replace only those dimensions in a copy of
the existing float32 action column, and preserve extra dimensions:

```python
updated = original_action.copy()
updated[episode_rows, :14] = repaired_hdf5_action.astype(np.float32)
flat = pa.array(updated.reshape(-1), type=pa.float32())
replacement = pa.FixedSizeListArray.from_arrays(flat, updated.shape[1])
updated_table = original_table.set_column(
    original_table.schema.get_field_index("action"),
    original_table.schema.field("action"),
    replacement,
).replace_schema_metadata(original_table.schema.metadata)
```

Preserve Arrow field order, types, nullability, schema metadata, Parquet row
group count/boundaries, and the original compression codec per column. A
Parquet rewrite may change low-level encoding bytes, so semantic equality—not
bitwise file equality—is required for unrelated columns; their arrays,
validity masks, and chunk-concatenated values must be equal. Recompute action
stats using NumPy float32 `min`, `max`, `mean`, `std`, and `count=[frames]`.

Group all planned updates by physical Parquet path and build one candidate that
contains every affected mapped episode in that file. Likewise, group by
physical `episodes_stats.jsonl`, replace all matching `stats.action` rows in a
single candidate, preserve every nonmatching line byte-for-byte, and install
each physical target only once. Back up each Parquet and stats file once per
run. Verify unrelated columns, extra action dimensions, schema metadata,
mapping files, stats rows, and videos before atomic installation.

Add a fixture with two HDF5 episodes mapped into distinct row selections of one
Parquet file and two rows of one stats physical file; assert both survive one
atomic update. Add a collision fixture where two sources claim the same target
row and require plan construction to fail before backups.

- [ ] **Step 5: Run the full focused module and commit**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/agilex/openpi/.venv/bin/python -m pytest -q \
  tests/test_repair_action_jitter_hdf5.py
```

Expected: all focused tests pass.

Commit:

```bash
git add scripts/embodied_data_pipeline-main/quality_pipeline/action_repair.py \
  tests/test_repair_action_jitter_hdf5.py
test "$(git diff --cached --name-only | sort)" = "$(
  printf '%s\n' \
    scripts/embodied_data_pipeline-main/quality_pipeline/action_repair.py \
    tests/test_repair_action_jitter_hdf5.py |
  sort
)"
git commit -m "feat: synchronize repaired LeRobot actions"
```

---

### Task 4: CLI, reports, regression verification, and documentation

**Files:**
- Create: `scripts/embodied_data_pipeline-main/scripts/repair_action_jitter_hdf5.py`
- Create: `scripts/embodied_data_pipeline-main/quality_pipeline/stage2_action_repair_acceptance.json`
- Modify: `scripts/embodied_data_pipeline-main/README.md`
- Modify: `tests/test_repair_action_jitter_hdf5.py`

**Interfaces:**
- Produces commands: `audit`, `plan`, `apply`, `verify`, `rollback`
- Consumes the Task 1–3 public interfaces without duplicating detection or persistence logic

- [ ] **Step 1: Add failing CLI contract tests**

Append tests which import the CLI, parse every subcommand, and run plan/apply/
verify/rollback against a temporary fixture. Require nonzero exit for a changed
source digest and JSON stdout containing command, counts, and artifact paths:

```python
def test_cli_exposes_all_transaction_commands():
    cli_path = (
        REPO_ROOT
        / "scripts/embodied_data_pipeline-main/scripts/repair_action_jitter_hdf5.py"
    )
    text = cli_path.read_text(encoding="utf-8")
    for command in ("audit", "plan", "apply", "verify", "rollback"):
        assert f'add_parser("{command}"' in text
    assert "--backup-root" in text
    assert "--plan" in text
    assert "--manifest" in text
```

Also execute the CLI rather than relying only on source inspection:

```python
import subprocess

PYTHON = Path("/home/agilex/openpi/.venv/bin/python")


def run_cli(*arguments):
    cli_path = (
        REPO_ROOT
        / "scripts/embodied_data_pipeline-main/scripts/repair_action_jitter_hdf5.py"
    )
    return subprocess.run(
        [str(PYTHON), str(cli_path), *map(str, arguments)],
        check=False,
        capture_output=True,
        text=True,
    )


def test_cli_plan_apply_verify_and_rollback(tmp_path):
    # Reuse the real HDF5 helpers above; return the stage2 root and source path.
    root, h5_path, acceptance = write_cli_fixture(tmp_path)
    run_dir = tmp_path / "backups/run-fixed"
    run_dir.mkdir(parents=True)
    audit_path = run_dir / "audit.json"
    plan_path = run_dir / "plan.json"
    release_sha = subprocess.run(
        ["git", "-C", str(REPO_ROOT), "rev-parse", "HEAD"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    audited = run_cli(
        "audit",
        "--root",
        root,
        "--acceptance",
        acceptance,
        "--release-sha",
        release_sha,
        "--output",
        audit_path,
    )
    assert audited.returncode == 0
    planned = run_cli(
        "plan",
        "--root",
        root,
        "--audit",
        audit_path,
        "--acceptance",
        acceptance,
        "--output",
        plan_path,
    )
    assert planned.returncode == 0
    applied = run_cli(
        "apply", "--plan", plan_path, "--backup-root", tmp_path / "backups"
    )
    assert applied.returncode == 0
    applied_json = json.loads(applied.stdout)
    assert Path(applied_json["manifest"]) == run_dir / "manifest.json"
    verified = run_cli(
        "verify",
        "--manifest",
        applied_json["manifest"],
        "--output",
        run_dir / "verification.json",
        "--independent-output",
        run_dir / "independent_verification.json",
        "--acceptance",
        acceptance,
    )
    assert verified.returncode == 0
    assert json.loads(verified.stdout)["ok"] is True
    rolled_back = run_cli("rollback", "--manifest", applied_json["manifest"])
    assert rolled_back.returncode == 0
    assert sha256(h5_path) == json.loads(rolled_back.stdout)["restored_sha256"]
```

Add negative CLI tests that independently assert exit status 1 for: changing
one scoped HDF5 after `audit` and before `plan`; using an audit from a different
resolved root; omitting `--acceptance` for the production root; mismatching the
acceptance root/count; one extra or missing expected repair path; and one
incorrect frame in an otherwise path-correct repair mask. Assert each failure
occurs before any backup directory or manifest is created.

- [ ] **Step 2: Run CLI tests and verify RED**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/agilex/openpi/.venv/bin/python -m pytest -q \
  tests/test_repair_action_jitter_hdf5.py -k cli
```

Expected: failure because the CLI file does not exist.

- [ ] **Step 3: Implement the thin CLI**

Use this command contract:

```text
repair_action_jitter_hdf5.py audit --root ROOT --acceptance ACCEPTANCE.json \
  --release-sha COMMIT --output REPORT.json
repair_action_jitter_hdf5.py plan --root ROOT --audit AUDIT.json \
  --acceptance ACCEPTANCE.json --output PLAN.json
repair_action_jitter_hdf5.py apply --plan PLAN.json --backup-root BACKUP_ROOT
repair_action_jitter_hdf5.py verify --manifest MANIFEST.json --output REPORT.json \
  --independent-output INDEPENDENT.json --acceptance ACCEPTANCE.json
repair_action_jitter_hdf5.py rollback --manifest MANIFEST.json
```

`audit` and `plan` are read-only with respect to HDF5 and LeRobot. The audit
schema includes resolved root, UTC timestamp, exact scoped relative paths and
SHA-256 digests, near-match exclusions, per-scene file/frame/byte counts,
mapping-file and mapping-row counts, target filesystem free bytes, and a
canonical inventory digest. `plan` requires `--audit`, recomputes inventory,
rejects any tree/digest difference, and embeds the audit SHA-256 and inventory
digest. `audit --release-sha` must equal the enclosing Git checkout's `HEAD`;
the value propagates to the plan and manifest, and `apply`/`verify` reject a
running checkout at a different commit. Resolve the enclosing checkout by
walking from `Path(__file__).resolve()`, never from the process current working
directory.

`apply`
requires `--backup-root`; there is no bypass flag. When `PLAN.json` is directly
inside `BACKUP_ROOT/<run-id>/`, that parent is the transaction run directory
and `apply` writes `manifest.json` beside the approved plan. Otherwise `apply`
creates a new unique child of `BACKUP_ROOT` and reports the exact path.
`apply_dataset_plan(..., run_dir=...)` requires the resolved run directory to
be a direct child of the resolved backup root. Backups use
`run_dir/backups/{hdf5,parquet,stats}/<root-relative-path>`; pre-existing
manifest, backup, or candidate paths are a hard failure, never overwritten.
`verify` freshly reopens every scoped source and target, writes the normal
manifest comparison to `--output`, and writes the plan-independent full
invariant scan to `--independent-output`. Every command prints one JSON object
to stdout and writes its detailed artifact atomically. Exceptions print one
concise error to stderr and return exit status 1.

- [ ] **Step 4: Document exact usage and safety guarantees**

Add a README section with the five commands, the production root
`/home/agilex/data/stage2_new`, the required backup root
`/home/agilex/repair_backups/action_jitter_repair`, the continuous-zero
guarantee, and the rule that `apply` must follow review of the generated plan.

Create `stage2_action_repair_acceptance.json` from the approved read-only
inventory. Its schema is:

```json
{
  "root": "/home/agilex/data/stage2_new",
  "expected_scoped_hdf5": 1819,
  "expected_frames": 400266,
  "must_not_change": [
    {
      "h5_relative_path": "scene10/three_camera_global/hdf5_episodes/20260724_scene10/episode0/states/aligned_joints.h5",
      "reason": "normal_control"
    }
  ],
  "expected_repair_masks": [
    {
      "h5_relative_path": "scene9/three_camera_global/hdf5_episodes/20260724_scene9/episode0/states/aligned_joints.h5",
      "side": "left",
      "frame_indices": [30, 33, 36]
    }
  ],
  "scene9_expected_problem_paths": [
    "scene9/three_camera_global/hdf5_episodes/20260724_scene9/episode0/states/aligned_joints.h5"
  ],
  "allowed_unresolved_reasons": [
    "ambiguous_source_direction",
    "boundary",
    "conflicting_hypothesis",
    "excessive_motion",
    "insufficient_periodic_support",
    "run_too_long",
    "short_episode",
    "spectral_gate"
  ]
}
```

The committed file must contain actual canonical relative paths. Include every
one of the 154 scene10 paths, every known
continuous-zero path, and all 51 approved scene9 candidate paths
`0–6,9–32,34–37,39–43,45–54,62`. The CLI accepts
`--acceptance ACCEPTANCE.json` on `audit`, `plan`, and `verify`; production
commands require it. Tests use a generated fixture-specific acceptance file.
Normalize the generated plan to the exact set of
`(h5_relative_path, side, sorted unique frame_indices)` masks and require exact
equality with `expected_repair_masks`. `scene9_expected_problem_paths` is the
exact set of scene9 paths having a nonempty accepted mask. Extra paths, missing
paths, wrong sides, or one wrong frame all fail the gate. Unresolved events may
vary only within `allowed_unresolved_reasons` and are always reported with
canonical paths and frames.

Populate `expected_repair_masks` by a read-only run of the completed detector
against the current root, then independently inspect every proposed run against
the raw HDF5 action/state, anchors, phase support, and spectral metrics. Confirm
the scene9 path set and all control paths before committing this file. Add
negative tests showing that an extra path, a missing path, and a path-correct
but frame-wrong acceptance each make `plan` exit 1.

- [ ] **Step 5: Run focused and repository regression checks**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/agilex/openpi/.venv/bin/python -m pytest -q \
  tests/test_repair_action_jitter_hdf5.py
/home/agilex/openpi/.venv/bin/python -m py_compile \
  scripts/embodied_data_pipeline-main/quality_pipeline/action_repair.py \
  scripts/embodied_data_pipeline-main/scripts/repair_action_jitter_hdf5.py
git diff --check
```

Expected: all focused tests pass, compilation exits 0, and diff check is clean.
Do not restore or include the user's pre-existing deleted tests.

- [ ] **Step 6: Commit Task 4**

```bash
git add scripts/embodied_data_pipeline-main/scripts/repair_action_jitter_hdf5.py \
  scripts/embodied_data_pipeline-main/quality_pipeline/stage2_action_repair_acceptance.json \
  scripts/embodied_data_pipeline-main/README.md \
  tests/test_repair_action_jitter_hdf5.py
test "$(git diff --cached --name-only | sort)" = "$(
  printf '%s\n' \
    scripts/embodied_data_pipeline-main/README.md \
    scripts/embodied_data_pipeline-main/quality_pipeline/stage2_action_repair_acceptance.json \
    scripts/embodied_data_pipeline-main/scripts/repair_action_jitter_hdf5.py \
    tests/test_repair_action_jitter_hdf5.py |
  sort
)"
git commit -m "feat: add action jitter repair workflow"
```

---

### Task 5: Production dry-run, full transactional apply, and derivative verification

**Files:**
- Write artifacts only under: `/home/agilex/repair_backups/action_jitter_repair/<UTC-run-id>/`
- Modify planned files only under: `/home/agilex/data/stage2_new/scene1` through `scene11`

**Interfaces:**
- Consumes the reviewed CLI from Task 4
- Produces a verified production manifest and repair report

- [ ] **Step 1: Pin and verify an immutable release checkout**

Run only after the whole implementation branch has passed final code review:

```bash
ACTION_REPAIR_IMPL_ROOT=/home/caizj/agilex_idata/.worktrees/action-jitter-repair
ACTION_REPAIR_CODE_ROOT=/home/caizj/agilex_idata/.worktrees/action-jitter-repair-release
test -z "$(git -C "${ACTION_REPAIR_IMPL_ROOT}" status --porcelain)"
ACTION_REPAIR_RELEASE_SHA="$(
  git -C "${ACTION_REPAIR_IMPL_ROOT}" rev-parse HEAD
)"
test ! -e "${ACTION_REPAIR_CODE_ROOT}"
git -C /home/caizj/agilex_idata worktree add \
  --detach "${ACTION_REPAIR_CODE_ROOT}" "${ACTION_REPAIR_RELEASE_SHA}"
test -z "$(git -C "${ACTION_REPAIR_CODE_ROOT}" status --porcelain)"

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  /home/agilex/openpi/.venv/bin/python -m pytest -q \
  "${ACTION_REPAIR_CODE_ROOT}/tests/test_repair_action_jitter_hdf5.py"
/home/agilex/openpi/.venv/bin/python -m py_compile \
  "${ACTION_REPAIR_CODE_ROOT}/scripts/embodied_data_pipeline-main/quality_pipeline/action_repair.py" \
  "${ACTION_REPAIR_CODE_ROOT}/scripts/embodied_data_pipeline-main/scripts/repair_action_jitter_hdf5.py"
```

Record `ACTION_REPAIR_RELEASE_SHA` in `audit.json`, `plan.json`, and
`manifest.json`. All production commands below use only this detached, clean
checkout—not `/home/caizj/agilex_idata`.

- [ ] **Step 2: Record preflight inventory and free space**

Run these read-only inventory commands before the audit:

```bash
df -B1 /home/agilex/data/stage2_new
find /home/agilex/data/stage2_new/scene{1..11} \
  -path '*/three_camera_global/hdf5_episodes/*/episode*/states/aligned_joints.h5' \
  -type f -printf '%s\n' |
  awk '{count += 1; bytes += $1} END {print "hdf5_count=" count, "hdf5_bytes=" bytes}'
find /home/agilex/data/stage2_new/scene{1..11} \
  -path '*/three_camera_global/lerobot/*/*/meta/episode_name_mapping.json' \
  -type f -print |
  wc -l
```

The `audit` artifact independently records HDF5 count, total bytes, LeRobot
mapping-file and mapping-row counts, and available filesystem bytes. Check the
source and backup filesystems separately. Before backup, require backup
filesystem free bytes of at least all planned physical backups plus 10 GiB,
and each source filesystem free bytes of at least its largest same-directory
candidate plus 10 GiB. If source and backup share a filesystem, require the sum
of both amounts. Recheck immediately before `backing_up`.

- [ ] **Step 3: Run production audit and plan**

Run:

```bash
ACTION_REPAIR_ROOT=/home/agilex/data/stage2_new
ACTION_REPAIR_BACKUPS=/home/agilex/repair_backups/action_jitter_repair
ACTION_REPAIR_ACCEPTANCE="${ACTION_REPAIR_CODE_ROOT}/scripts/embodied_data_pipeline-main/quality_pipeline/stage2_action_repair_acceptance.json"
ACTION_REPAIR_RUN_ID="$(date -u +%Y%m%dT%H%M%S.%6NZ)"
ACTION_REPAIR_RUN_DIR="${ACTION_REPAIR_BACKUPS}/${ACTION_REPAIR_RUN_ID}"
mkdir -p "${ACTION_REPAIR_RUN_DIR}"

/home/agilex/openpi/.venv/bin/python \
  "${ACTION_REPAIR_CODE_ROOT}/scripts/embodied_data_pipeline-main/scripts/repair_action_jitter_hdf5.py" \
  audit --root "${ACTION_REPAIR_ROOT}" \
  --acceptance "${ACTION_REPAIR_ACCEPTANCE}" \
  --release-sha "${ACTION_REPAIR_RELEASE_SHA}" \
  --output "${ACTION_REPAIR_RUN_DIR}/audit.json"

/home/agilex/openpi/.venv/bin/python \
  "${ACTION_REPAIR_CODE_ROOT}/scripts/embodied_data_pipeline-main/scripts/repair_action_jitter_hdf5.py" \
  plan --root "${ACTION_REPAIR_ROOT}" \
  --audit "${ACTION_REPAIR_RUN_DIR}/audit.json" \
  --acceptance "${ACTION_REPAIR_ACCEPTANCE}" \
  --output "${ACTION_REPAIR_RUN_DIR}/plan.json"
```

Require before proceeding:

- exactly 1819 scoped HDF5 files are readable unless the inventory documents a
  user-created change since the approved analysis;
- every canonical `must_not_change` path in the acceptance file has zero
  planned changes, covering all scene10 data and every approved continuous-zero
  episode;
- the exact canonical scene9 problem-path set equals the 51 entries encoded in
  the acceptance file, corresponding to
  `0–6,9–32,34–37,39–43,45–54,62`;
- canonical scene9 episode61 has zero planned changes; any unresolved reason is
  reported but not required;
- every mapped affected HDF5 passes LeRobot preflight.

If any requirement fails, stop before apply and return to detector diagnosis;
do not loosen thresholds merely to reproduce expected counts.

- [ ] **Step 4: Apply with mandatory backup**

Run:

```bash
/home/agilex/openpi/.venv/bin/python \
  "${ACTION_REPAIR_CODE_ROOT}/scripts/embodied_data_pipeline-main/scripts/repair_action_jitter_hdf5.py" \
  apply --plan "${ACTION_REPAIR_RUN_DIR}/plan.json" \
  --backup-root "${ACTION_REPAIR_BACKUPS}"
```

Capture the emitted manifest path. If the command exits nonzero, inspect
manifest state and require verified automatic rollback before any retry.

- [ ] **Step 5: Verify the complete production transaction**

Run:

```bash
/home/agilex/openpi/.venv/bin/python \
  "${ACTION_REPAIR_CODE_ROOT}/scripts/embodied_data_pipeline-main/scripts/repair_action_jitter_hdf5.py" \
  verify --manifest "${ACTION_REPAIR_RUN_DIR}/manifest.json" \
  --output "${ACTION_REPAIR_RUN_DIR}/verification.json" \
  --independent-output "${ACTION_REPAIR_RUN_DIR}/independent_verification.json" \
  --acceptance "${ACTION_REPAIR_ACCEPTANCE}"
```

Require `ok=true`, zero unexpected HDF5 differences, zero unexpected Parquet
differences, exact action stats, zero residual planned events, and backup
digests matching every pre-apply source.

- [ ] **Step 6: Review the independently generated full scan**

The preceding `verify --independent-output` performs a fresh read-only scan
that uses the manifest only to locate backups and legal masks, not detector
predictions. Assert:

- all 1819 HDF5 files reopen;
- every frame still has finite 14D state and action;
- total frame count remains 400266;
- scene10 is numerically unchanged against backups or preflight hashes;
- all action values outside manifest masks match backups bitwise;
- continuous-zero runs match backups bitwise;
- repaired values equal neighbor interpolation;
- every mapped LeRobot first 14 action dimensions match HDF5 within `1e-6`.
- a fresh post-repair detector run produces zero accepted repair events; retain
  and summarize all conservative unresolved events;
- replacements recomputed directly from backup anchors equal installed values,
  without reading replacement vectors from the plan.

For this comparison, recompute in float64, cast through the original HDF5
dataset dtype, and require bitwise equality with the installed dataset slice.
LeRobot comparisons cast that HDF5 result to float32 and use absolute tolerance
`1e-6`.

Require `${ACTION_REPAIR_RUN_DIR}/independent_verification.json` to report all
of these invariants as true before continuing.

---

### Task 6: Rerun QC for affected batches and final delivery audit

**Files:**
- Create new, run-specific QC outputs under each affected camera root's
  `qc_reports` directory
- Read only: production repair manifest, HDF5, LeRobot, backups, and all
  pre-existing QC/grade artifacts

**Interfaces:**
- Consumes the verified production manifest from Task 5
- Produces fresh compact QC reports and a final delivery summary

- [ ] **Step 1: Enumerate affected HDF5 batch roots from the manifest**

Build a unique sorted list of `.../three_camera_global/hdf5_episodes/<batch>`
directories containing at least one applied repair. Resolve each corresponding
`three_camera_global/qc_reports` directory without changing manual grades.

- [ ] **Step 2: Run compact QC for every affected batch**

Extract the exact batch roots from the manifest and run compact QC:

```bash
ACTION_REPAIR_MANIFEST=/home/agilex/repair_backups/action_jitter_repair/\
"${ACTION_REPAIR_RUN_ID}/manifest.json"
mapfile -t ACTION_REPAIR_BATCHES < <(
  /home/agilex/openpi/.venv/bin/python - "${ACTION_REPAIR_MANIFEST}" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    manifest = json.load(stream)
for batch in sorted(set(manifest["affected_batches"])):
    print(batch)
PY
)

for ACTION_REPAIR_BATCH_ROOT in "${ACTION_REPAIR_BATCHES[@]}"; do
  ACTION_REPAIR_CAMERA_ROOT="$(
    dirname "$(dirname "${ACTION_REPAIR_BATCH_ROOT}")"
  )"
  ACTION_REPAIR_BATCH_NAME="$(basename "${ACTION_REPAIR_BATCH_ROOT}")"
  mkdir -p "${ACTION_REPAIR_CAMERA_ROOT}/qc_reports"
  test ! -e "${ACTION_REPAIR_CAMERA_ROOT}/qc_reports/${ACTION_REPAIR_BATCH_NAME}_action_repair_${ACTION_REPAIR_RUN_ID}"
  /home/agilex/openpi/.venv/bin/python \
    "${ACTION_REPAIR_CODE_ROOT}/scripts/embodied_data_pipeline-main/scripts/run_quality_pipeline.py" \
    --profile "${ACTION_REPAIR_CODE_ROOT}/scripts/embodied_data_pipeline-main/robot_profiles/aloha.yaml" \
    --input "${ACTION_REPAIR_BATCH_ROOT}" \
    --output "${ACTION_REPAIR_CAMERA_ROOT}/qc_reports/${ACTION_REPAIR_BATCH_NAME}_action_repair_${ACTION_REPAIR_RUN_ID}" \
    --compact --num-workers 8
done
```

Any QC command failure is reported separately; it does not trigger rollback of
numerically verified action repair unless the failure demonstrates data
corruption.

- [ ] **Step 3: Run final code and data verification**

Run fresh commands:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/agilex/openpi/.venv/bin/python -m pytest -q \
  "${ACTION_REPAIR_CODE_ROOT}/tests/test_repair_action_jitter_hdf5.py"
/home/agilex/openpi/.venv/bin/python -m py_compile \
  "${ACTION_REPAIR_CODE_ROOT}/scripts/embodied_data_pipeline-main/quality_pipeline/action_repair.py" \
  "${ACTION_REPAIR_CODE_ROOT}/scripts/embodied_data_pipeline-main/scripts/repair_action_jitter_hdf5.py"
test "$(git -C "${ACTION_REPAIR_CODE_ROOT}" rev-parse HEAD)" = \
  "${ACTION_REPAIR_RELEASE_SHA}"
test -z "$(git -C "${ACTION_REPAIR_CODE_ROOT}" status --porcelain)"
```

Re-read `verification.json`, `independent_verification.json`, and new QC
summaries. Report exact affected episode/frame counts, unresolved counts,
backup path, rollback command, LeRobot synchronization count, QC successes or
failures, test totals, and commit IDs. Do not claim completion if any required
verification field is false.

- [ ] **Step 4: Fast-forward the main branch without disturbing user changes**

Capture `/home/caizj/agilex_idata` status again and require the same
pre-existing user paths from Task 0. Then run:

```bash
git -C /home/caizj/agilex_idata merge --ff-only \
  feature/action-jitter-repair
test "$(git -C /home/caizj/agilex_idata rev-parse HEAD)" = \
  "${ACTION_REPAIR_RELEASE_SHA}"
```

If Git reports that a user modification would be overwritten, do not stash,
reset, or overwrite it; leave the reviewed release commit and worktree intact
and report that integration blocker separately. Otherwise verify all original
user deletions/untracked files remain unchanged.

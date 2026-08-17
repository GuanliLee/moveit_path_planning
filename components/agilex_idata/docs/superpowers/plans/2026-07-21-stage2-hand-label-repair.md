# Stage2 Hand Label Repair Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Change exactly 24 mislabeled trajectories from `right hand` to `left hand` in the raw MCAP sidecars, HDF5 representations, and every three-/four-camera LeRobot derivative under `/home/agilex/data/stage2_new`.

**Architecture:** Treat the original episode name as the canonical key: scene2 `episode11`–`episode15`, and scene3 `episode81`–`episode96` plus `episode98`–`episode100`. Validate every expected old task before mutation, back up each file before its first write, update all task-bearing representations, and independently read every representation back after the repair. Resolve LeRobot indices only through `meta/episode_name_mapping.json`; never assume the LeRobot index equals the original episode number.

**Tech Stack:** Python 3.12, `h5py`, `pyarrow`, JSON/JSONL, MCAP CLI, SHA-256 manifests.

## Global Constraints

- Modify exactly 24 canonical episodes: scene2 `11–15`; scene3 `81–96, 98–100`.
- For task-bearing fields already matching the canonical old task, replace only the suffix `with the right hand.` with `with the left hand.` while preserving the product name. For source ALOHA sidecars that are historically stale or blank, replace the whole sidecar from the corrected canonical raw JSON so the HDF5 representation is internally consistent.
- Cover raw MCAP sidecar metadata, the source ALOHA HDF5 sidecar and instruction dataset, both `three_camera_global` and `four_camera` HDF5 outputs, and both camera profiles' LeRobot grade datasets.
- Do not rewrite MCAP binaries: `mcap info` confirms they contain only ROS messages and the `rosbag2` metadata record, while task text is held in `episode*_0_info.json`.
- Refuse to apply if a canonical episode is missing, has a different product/task, appears zero or multiple times in a camera profile's LeRobot mappings, or if any target Parquet contains mixed task indices.
- Back up every changed file and record its original and repaired SHA-256 digest.

---

### Task 1: Build the repair and audit utility

**Files:**
- Create: `/home/agilex/repair_backups/stage2_hand_label_repair/repair_stage2_hand_labels.py`
- Create: `/home/agilex/repair_backups/stage2_hand_label_repair/test_repair_stage2_hand_labels.py`

**Interfaces:**
- Consumes: dataset root, fixed canonical episode/task map, and optional backup directory.
- Produces: `audit(root) -> AuditReport`, `apply(root, backup_dir) -> RepairReport`, and `verify(root) -> VerificationReport`.

- [ ] **Step 1: Write fixture tests for exact episode selection and LeRobot mapping resolution**

```python
def test_targets_are_exactly_24():
    assert len(all_targets()) == 24
    assert sorted(TARGETS["scene2"]) == [11, 12, 13, 14, 15]
    assert sorted(TARGETS["scene3"]) == [*range(81, 97), 98, 99, 100]

def test_mapping_resolution_uses_source_episode_name():
    mapping = {"episodes": [{"source_episode_name": "episode89", "lerobot_episode_index": 1}]}
    assert resolve_lerobot_episode(mapping, "episode89") == 1
```

- [ ] **Step 2: Run the tests and confirm they fail before implementation**

Run: `python3 -m unittest -v /home/agilex/repair_backups/stage2_hand_label_repair/test_repair_stage2_hand_labels.py`

Expected: failure because the repair module does not yet exist.

- [ ] **Step 3: Implement strict audit, backup, mutation, and verification functions**

```python
TARGETS = {
    "scene2": {episode: "Grasp HK Orange Fanta" for episode in range(11, 16)},
    "scene3": {
        **{episode: "Grasp Daily C Orange Juice" for episode in range(81, 83)},
        **{episode: "Grasp Wanglaoji" for episode in range(83, 91)},
        **{episode: "Grasp Sprite" for episode in (*range(91, 97), 98, 99, 100)},
    },
}

def old_task(prefix: str) -> str:
    return f"{prefix} with the right hand."

def new_task(prefix: str) -> str:
    return f"{prefix} with the left hand."
```

The implementation must update raw `full-instructions-en` and `full-instructions-zh`; synchronize each source ALOHA `episode*_0_info.json` from the corrected canonical raw sidecar; update source ALOHA HDF5 `instructions/full_instructions/text`; update HDF5 `meta/episode_meta.json` fields `task`, `tasks`, `full_instructions_en`, and `full_instructions_zh`; update the aligned HDF5 root attribute `task`; and update LeRobot `episodes.jsonl`, `episode_name_mapping.json`, a newly reused-or-created `tasks.jsonl` entry, Parquet `task_index`, and `info.json.total_tasks`.

- [ ] **Step 4: Run the fixture tests and confirm they pass**

Run: `python3 -m unittest -v /home/agilex/repair_backups/stage2_hand_label_repair/test_repair_stage2_hand_labels.py`

Expected: all tests pass.

### Task 2: Preflight and apply the repair

**Files:**
- Modify: `/home/agilex/data/stage2_new/scene{2,3}/<date>/episode*/episode*_0_info.json`
- Modify: `/home/agilex/data/stage2_new/scene{2,3}/<date>/aloha/episode*/episode*_0_info.json`
- Modify: `/home/agilex/data/stage2_new/scene{2,3}/<date>/aloha/episode*/episode*.hdf5`
- Modify: `/home/agilex/data/stage2_new/scene{2,3}/{three_camera_global,four_camera}/hdf5_episodes/<date>/episode*/meta/episode_meta.json`
- Modify: `/home/agilex/data/stage2_new/scene{2,3}/{three_camera_global,four_camera}/hdf5_episodes/<date>/episode*/states/aligned_joints.h5`
- Modify: matching LeRobot `A`/`F` metadata and Parquet files under both camera profiles.
- Create: `/home/agilex/repair_backups/stage2_hand_label_repair/<timestamp>/manifest.json`

**Interfaces:**
- Consumes: a clean preflight report with exactly 24 original episodes and 48 LeRobot counterparts.
- Produces: repaired data plus a restorable per-file backup tree and digest manifest.

- [ ] **Step 1: Run read-only preflight**

Run: `python3 repair_stage2_hand_labels.py audit --root /home/agilex/data/stage2_new`

Expected: `24 canonical episodes`, `48 HDF5 outputs`, `48 LeRobot episodes`, and zero errors.

- [ ] **Step 2: Apply with backups**

Run: `python3 repair_stage2_hand_labels.py apply --root /home/agilex/data/stage2_new --backup-root /home/agilex/repair_backups/stage2_hand_label_repair`

Expected: exactly 24 raw sidecars, 24 source ALOHA sidecars, 24 source ALOHA HDF5 files, 48 HDF5 metadata files, 48 aligned HDF5 files, 48 LeRobot Parquet files, and their required dataset-level metadata files are updated.

### Task 3: Independently verify all representations

**Files:**
- Read: every modified file and all 24 MCAP binaries.
- Create: `/home/agilex/repair_backups/stage2_hand_label_repair/<timestamp>/verification.json`

**Interfaces:**
- Consumes: repaired data and the manifest.
- Produces: machine-readable proof that all representations resolve to the corrected task and non-task data stayed intact.

- [ ] **Step 1: Verify labels and indices by fresh reads**

Run: `python3 repair_stage2_hand_labels.py verify --root /home/agilex/data/stage2_new --manifest <manifest-path>`

Expected: all 24 raw sidecars, 24 source ALOHA sidecars, 24 source HDF5 instructions, 48 HDF5 task attributes/metadata records, and 48 LeRobot mappings/episode rows/Parquet indices resolve to `left hand`; no target representation resolves to `right hand`.

- [ ] **Step 2: Verify MCAP structural health**

Run: `for file in <24-explicit-mcap-paths>; do mcap doctor "$file"; done`

Expected: all 24 files pass structural checks and their SHA-256 hashes are unchanged from the manifest.

- [ ] **Step 3: Verify non-task Parquet columns are unchanged**

Run: `python3 repair_stage2_hand_labels.py verify --root /home/agilex/data/stage2_new --manifest <manifest-path> --compare-backups`

Expected: schema and row counts match backups; every column except `task_index` has the same Arrow values; HDF5 frame/group counts match backups.

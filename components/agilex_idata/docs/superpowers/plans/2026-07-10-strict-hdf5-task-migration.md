# Strict HDF5 Task Migration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove the ALOHA fallback task, make HDF5 the only ALOHA LeRobot task source, normalize the approved 19 beverage names, and migrate existing grade metadata safely.

**Architecture:** A focused `quality_pipeline.task_names` module owns the vocabulary, dataset identity mapping, HDF5 resolution, and task-only writes. The converter and Web pipeline consume these helpers; a standalone migration CLI reuses them for audit/apply/rollback and grade verification.

**Tech Stack:** Python 3.10+, h5py, PyArrow, JSON/JSONL, pytest, Bash.

## Global Constraints

- The sentence is exactly `Grasp <canonical beverage name> with the left hand.`
- The vocabulary is exactly the approved 19 names; `NEVER Light Latte` is excluded.
- Data mutation is limited to `/home/agilex/data/stage2/hdf5_episodes` and `/home/agilex/data/stage2/lerobot`.
- Never change category, grade, episode index, frames, media, state, or action values.
- Manual Web input fills only missing HDF5 tasks and never replaces an existing task.
- Unknown or ambiguous dataset identities fail before mutation.
- Record every changed field in `/home/agilex/data/stage2/task-migration-<timestamp>.json` before apply mode mutates data.
- Remove `grasp the bottles on the table` from the ALOHA profile and migrated task metadata.

---

### Task 1: Canonical vocabulary and HDF5 task primitives

**Files:**
- Create: `scripts/embodied_data_pipeline-main/quality_pipeline/task_names.py`
- Modify: `scripts/embodied_data_pipeline-main/robot_profiles/aloha.yaml:10-11`
- Create: `tests/test_task_name_normalization.py`

**Interfaces:**
- Produces: `CANONICAL_BEVERAGE_NAMES: tuple[str, ...]`
- Produces: `canonical_product_for_dataset_name(name: str) -> str | None`
- Produces: `canonical_task(product: str) -> str`
- Produces: `canonical_task_for_dataset_name(name: str) -> str | None`
- Produces: `read_hdf5_task(path: Path) -> str`
- Produces: `write_hdf5_task(path: Path, task: str) -> dict[str, object]`
- Produces: `write_episode_sidecar_task(path: Path, task: str) -> dict[str, object]`

- [ ] **Step 1: Write failing vocabulary and HDF5 tests**

```python
def test_canonical_vocabulary_is_exact():
    assert list(task_names.CANONICAL_BEVERAGE_NAMES) == [
        "Yakult", "Guangming Probiotic Milk", "Coca-Cola",
        "COSTA Peach Oolong", "Taro Milk", "Robuk Velvet Latte",
        "Yili Peach Yogurt", "HK Orange Fanta", "Aojiru",
        "NEVER Coconut Latte", "Daily C Orange Juice",
        "Daily C Grape Juice", "COSTA Grape Jasmine", "Wanglaoji",
        "AD Calcium Milk", "Yili Strawberry Yogurt",
        "Dahongpao Milk Tea", "Sprite", "Royal Coconut",
    ]

def test_dataset_slug_formats_exact_task():
    assert task_names.canonical_task_for_dataset_name(
        "20260630_aloha_market_left-grasp_hk-orange-fanta"
    ) == "Grasp HK Orange Fanta with the left hand."

def test_hdf5_task_prefers_task_attribute(tmp_path):
    path = tmp_path / "aligned_joints.h5"
    with h5py.File(path, "w") as file_obj:
        file_obj.attrs["task"] = "Grasp Wanglaoji with the left hand."
        file_obj.attrs["text"] = "wrong"
    assert task_names.read_hdf5_task(path) == "Grasp Wanglaoji with the left hand."
```

- [ ] **Step 2: Run RED**

Run: `python3 -m pytest -q tests/test_task_name_normalization.py`

Expected: collection fails because `quality_pipeline.task_names` is absent.

- [ ] **Step 3: Implement the vocabulary and task primitives**

```python
CANONICAL_BEVERAGE_NAMES = (
    "Yakult", "Guangming Probiotic Milk", "Coca-Cola",
    "COSTA Peach Oolong", "Taro Milk", "Robuk Velvet Latte",
    "Yili Peach Yogurt", "HK Orange Fanta", "Aojiru",
    "NEVER Coconut Latte", "Daily C Orange Juice",
    "Daily C Grape Juice", "COSTA Grape Jasmine", "Wanglaoji",
    "AD Calcium Milk", "Yili Strawberry Yogurt",
    "Dahongpao Milk Tea", "Sprite", "Royal Coconut",
)

def canonical_task(product: str) -> str:
    if product not in CANONICAL_BEVERAGE_NAMES:
        raise ValueError(f"unknown canonical beverage: {product}")
    return f"Grasp {product} with the left hand."
```

Use an explicit longest-marker-first slug table for all 19 names. Read HDF5
in priority order `task`, `tasks_json`, then plain non-JSON `text`. Task-only
writes update `task`, existing `tasks_json`, existing plain `text`, and sidecar
`task`/`tasks`/`full_instructions_en` through atomic replacement.

- [ ] **Step 4: Delete the fallback from `aloha.yaml`**

```yaml
tasks:
  default: "grasp the bottles on the table"
```

- [ ] **Step 5: Run GREEN and commit**

```bash
python3 -m pytest -q tests/test_task_name_normalization.py
git add scripts/embodied_data_pipeline-main/quality_pipeline/task_names.py \
  scripts/embodied_data_pipeline-main/robot_profiles/aloha.yaml \
  tests/test_task_name_normalization.py
git commit -m "feat: define canonical ALOHA beverage tasks"
```

### Task 2: Strict ALOHA HDF5-to-LeRobot sourcing

**Files:**
- Modify: `scripts/embodied_data_pipeline-main/quality_pipeline/episode_io.py:458-503`
- Modify: `scripts/embodied_data_pipeline-main/lerobot_conversion/scripts/convert_hdf5_to_lerobot_v2.py:235-236,664-696,942-947,2182-2230`
- Create: `tests/test_strict_hdf5_task_source.py`

**Interfaces:**
- Consumes: Task 1 helpers
- Produces: `aloha_task_for_conversion(h5_path: Path) -> str`
- Produces: `completed_record_task_matches(record: dict[str, Any], expected_task: str) -> bool`

- [ ] **Step 1: Write failing strict-source tests**

```python
def test_missing_aloha_hdf5_task_is_rejected(tmp_path):
    path = tmp_path / "aligned_joints.h5"
    with h5py.File(path, "w"):
        pass
    with pytest.raises(ValueError, match="HDF5 task metadata is missing"):
        task_names.require_hdf5_task(path)

def test_profile_contains_no_fallback_literal():
    assert "grasp the bottles on the table" not in ALOHA_PROFILE.read_text()

```

- [ ] **Step 2: Run RED**

Run: `python3 -m pytest -q tests/test_strict_hdf5_task_source.py`

Expected: missing helpers and current permissive behavior fail assertions.

- [ ] **Step 3: Implement strict ALOHA behavior**

For ALOHA, read directly from `h5_path` and raise:

```python
raise ValueError(
    f"{h5_path}: HDF5 task metadata is missing; expected root attribute "
    "task, tasks_json, or plain text. Enter task text in the Web pipeline and retry."
)
```

Force LeRobot episode `task`, `tasks`, and `full_instructions_en` to this HDF5
value. Reject ALOHA `--task` and `--default-task`; keep non-ALOHA behavior.
Resume mode compares each mapping task with current HDF5 and rewrites stale
episodes instead of skipping them.

- [ ] **Step 4: Run GREEN and commit**

```bash
python3 -m pytest -q tests/test_strict_hdf5_task_source.py
/home/agilex/openpi/.venv/bin/python -m pytest -q tests/test_lerobot_conversion_stats.py
git add scripts/embodied_data_pipeline-main/quality_pipeline/episode_io.py \
  scripts/embodied_data_pipeline-main/lerobot_conversion/scripts/convert_hdf5_to_lerobot_v2.py \
  tests/test_strict_hdf5_task_source.py
git commit -m "fix: source ALOHA LeRobot tasks strictly from HDF5"
```

### Task 3: Web missing-task flow and grade validation

**Files:**
- Modify: `scripts/embodied_data_pipeline-main/scripts/pipeline_web_app.py:2977-3074,5544-5556,6412-6424`
- Modify: `scripts/collection/collect_mobile_pipeline_qc_web.sh:59`
- Create: `tests/test_pipeline_task_consistency.py`

**Interfaces:**
- Produces: `prepare_hdf5_tasks_step(cfg) -> Callable[[dict[str, Any]], None]`
- Produces: `validate_lerobot_grade_tasks(cfg) -> dict[str, Any]`
- Produces: `validate_lerobot_grade_tasks_step(cfg) -> Callable[[dict[str, Any]], None]`

- [ ] **Step 1: Write failing Web tests**

```python
def make_episode_hdf5(root, episode_name, task):
    path = root / episode_name / "states" / "aligned_joints.h5"
    path.parent.mkdir(parents=True)
    with h5py.File(path, "w") as file_obj:
        if task:
            file_obj.attrs["task"] = task
    return path

def minimal_cfg(tmp_path):
    return app.derive_paths({
        "dataset_name": "20260630_aloha_market_left-grasp_hk-orange-fanta",
        "hdf5_root": str(tmp_path / "hdf5"),
        "lerobot_root": str(tmp_path / "lerobot"),
        "robot_type": "aloha",
    })

def test_lerobot_command_never_passes_task_override(tmp_path):
    cfg = minimal_cfg(tmp_path)
    cfg["task_text"] = "manual"
    command, _, _ = app.lerobot_command(cfg, "A", [0])
    assert "--task" not in command

def test_manual_value_fills_only_missing_hdf5_tasks(tmp_path, monkeypatch):
    root = tmp_path / "hdf5"
    existing = make_episode_hdf5(root, "episode0", EXISTING)
    missing = make_episode_hdf5(root, "episode1", "")
    cfg = minimal_cfg(tmp_path)
    cfg.update(hdf5_root=root, task_text=CANONICAL)
    monkeypatch.setattr(app, "append_job", lambda *_: None)
    app.prepare_hdf5_tasks_step(cfg)({})
    assert read_hdf5_task(existing) == EXISTING
    assert read_hdf5_task(missing) == CANONICAL

def test_missing_task_without_manual_value_names_episode(tmp_path, monkeypatch):
    root = tmp_path / "hdf5"
    make_episode_hdf5(root, "episode1", "")
    cfg = minimal_cfg(tmp_path)
    cfg.update(hdf5_root=root, task_text="")
    monkeypatch.setattr(app, "append_job", lambda *_: None)
    with pytest.raises(ValueError, match="episode1"):
        app.prepare_hdf5_tasks_step(cfg)({})
```

- [ ] **Step 2: Run RED**

Run: `python3 -m pytest -q tests/test_pipeline_task_consistency.py`

Expected: helpers are absent and `lerobot_command` contains `--task`.

- [ ] **Step 3: Implement preparation, copy, and validation**

Insert task preparation before A/B/C/F commands. Fill only missing HDF5 and
sidecar tasks; otherwise log retained counts. If manual text is absent, raise a
Chinese error listing missing episodes. Remove `--task` from LeRobot commands.
After grade conversion, resolve every Parquet `task_index` through
`tasks.jsonl`, correlate mapping records to source HDF5, and fail on mismatch.
For a single-product source, every grade must expose the same singleton task.

- [ ] **Step 4: Update launcher/UI wording**

Use “仅用于补写缺少任务的 HDF5，不覆盖已有 HDF5 task” and remove all copy
claiming that a profile default will be used.

- [ ] **Step 5: Run GREEN and commit**

```bash
python3 -m pytest -q tests/test_pipeline_task_consistency.py \
  tests/test_pipeline_dataset_discovery.py tests/test_pipeline_qc_web_script.py
bash -n scripts/collection/collect_mobile_pipeline_qc_web.sh
python3 -m py_compile scripts/embodied_data_pipeline-main/scripts/pipeline_web_app.py
git add scripts/embodied_data_pipeline-main/scripts/pipeline_web_app.py \
  scripts/collection/collect_mobile_pipeline_qc_web.sh \
  tests/test_pipeline_task_consistency.py
git commit -m "feat: enforce grade task consistency in Web pipeline"
```

### Task 4: Reversible existing-data migration

**Files:**
- Create: `scripts/embodied_data_pipeline-main/scripts/normalize_aloha_task_metadata.py`
- Create: `tests/test_normalize_aloha_task_metadata.py`

**Interfaces:**
- Produces: `audit_roots(hdf5_root: Path, lerobot_root: Path) -> dict[str, Any]`
- Produces: `build_migration_plan(...) -> dict[str, Any]`
- Produces: `apply_migration(plan: dict[str, Any], manifest_path: Path) -> None`
- Produces: `rollback_migration(manifest_path: Path) -> None`
- CLI: `audit`, `apply`, `rollback`, `verify`

- [ ] **Step 1: Write failing migration tests**

```python
def make_fixture(tmp_path, task):
    hroot = tmp_path / "hdf5_episodes"
    lroot = tmp_path / "lerobot"
    h5_path = make_episode_hdf5(
        hroot / "20260630_aloha_market_left-grasp_hk-orange-fanta",
        "episode0",
        task,
    )
    grade = lroot / "20260630_aloha_market_left-grasp_hk-orange-fanta" / "A"
    write_lerobot_fixture(grade, h5_path, task_index=1, tasks={0: task, 1: task})
    return hroot, lroot, h5_path, grade

def write_lerobot_fixture(grade, h5_path, task_index, tasks):
    (grade / "meta").mkdir(parents=True)
    data_dir = grade / "data" / "chunk-000"
    data_dir.mkdir(parents=True)
    rows = [
        json.dumps({"task_index": index, "task": task})
        for index, task in sorted(tasks.items())
    ]
    (grade / "meta" / "tasks.jsonl").write_text("\n".join(rows) + "\n")
    table = pa.table({
        "episode_index": pa.array([0], type=pa.int64()),
        "task_index": pa.array([task_index], type=pa.int64()),
    })
    pq.write_table(table, data_dir / "episode_000000.parquet")
    mapping = {"episodes": [{
        "lerobot_episode_index": 0,
        "source_h5": str(h5_path),
        "task": tasks[task_index],
    }]}
    (grade / "meta" / "episode_name_mapping.json").write_text(
        json.dumps(mapping) + "\n"
    )

def test_audit_is_read_only(tmp_path):
    hroot, lroot, h5_path, grade = make_fixture(
        tmp_path, "Grasp Fanta with the left hand."
    )
    before_hdf5 = read_hdf5_task(h5_path)
    before_tasks = (grade / "meta" / "tasks.jsonl").read_text()
    assert migration.audit_roots(hroot, lroot)["noncanonical_hdf5"] == 1
    assert read_hdf5_task(h5_path) == before_hdf5
    assert (grade / "meta" / "tasks.jsonl").read_text() == before_tasks

def test_apply_normalizes_hdf5_sidecar_and_grade_index(tmp_path):
    hroot, lroot, h5_path, grade = make_fixture(
        tmp_path, "Grasp Fanta with the left hand."
    )
    manifest = tmp_path / "manifest.json"
    plan = migration.build_migration_plan(hroot, lroot)
    migration.apply_migration(plan, manifest)
    expected = "Grasp HK Orange Fanta with the left hand."
    assert read_hdf5_task(h5_path) == expected
    assert migration.audit_roots(hroot, lroot)["pending_changes"] == 0

def test_rollback_restores_original_task_metadata(tmp_path):
    hroot, lroot, h5_path, _ = make_fixture(
        tmp_path, "Grasp Fanta with the left hand."
    )
    before = read_hdf5_task(h5_path)
    manifest = tmp_path / "manifest.json"
    migration.apply_migration(migration.build_migration_plan(hroot, lroot), manifest)
    migration.rollback_migration(manifest)
    assert read_hdf5_task(h5_path) == before
```

- [ ] **Step 2: Run RED**

Run: `python3 -m pytest -q tests/test_normalize_aloha_task_metadata.py`

Expected: migration module is absent.

- [ ] **Step 3: Implement audit/apply/rollback/verify**

Map only explicit dataset slugs. Build and persist the complete manifest before
mutation. Change only task metadata. Atomically replace JSON, JSONL, and
Parquet files. Collapse current single-product grades to task index `0`,
rewriting only Parquet files whose task index differs. Preserve schema and all
non-task columns. Report orphan LeRobot episodes without deleting them.

- [ ] **Step 4: Run GREEN and commit**

```bash
python3 -m pytest -q tests/test_normalize_aloha_task_metadata.py
git add scripts/embodied_data_pipeline-main/scripts/normalize_aloha_task_metadata.py \
  tests/test_normalize_aloha_task_metadata.py
git commit -m "feat: add reversible ALOHA task metadata migration"
```

### Task 5: Migrate and verify approved roots

**Files:**
- Runtime task metadata under the two approved roots
- Create: `/home/agilex/data/stage2/task-migration-<timestamp>.json`

**Interfaces:**
- Consumes: Task 4 CLI
- Produces: canonical task metadata and exact rollback manifest

- [ ] **Step 1: Run fresh audit**

```bash
/home/agilex/openpi/.venv/bin/python \
  scripts/embodied_data_pipeline-main/scripts/normalize_aloha_task_metadata.py audit \
  --hdf5-root /home/agilex/data/stage2/hdf5_episodes \
  --lerobot-root /home/agilex/data/stage2/lerobot
```

Expected: 1,012 HDF5 episodes and 414 non-canonical HDF5 tasks; no writes.

- [ ] **Step 2: Apply once, then verify and prove idempotence**

Run `apply --manifest /home/agilex/data/stage2/task-migration-<timestamp>.json`,
then `verify`, then `audit` again. Expected: zero pending recognized task
changes, zero grade task mismatches, zero fallback task occurrences; orphan
counts are reported without deletion.

### Task 6: Final verification

**Files:**
- No new files

**Interfaces:**
- Consumes: all tasks
- Produces: fresh completion evidence

- [ ] **Step 1: Run focused tests and syntax checks**

```bash
python3 -m pytest -q tests/test_task_name_normalization.py \
  tests/test_strict_hdf5_task_source.py tests/test_pipeline_task_consistency.py \
  tests/test_normalize_aloha_task_metadata.py \
  tests/test_pipeline_dataset_discovery.py tests/test_pipeline_qc_web_script.py
bash -n scripts/collection/collect_mobile_pipeline_qc_web.sh
python3 -m py_compile \
  scripts/embodied_data_pipeline-main/quality_pipeline/task_names.py \
  scripts/embodied_data_pipeline-main/scripts/pipeline_web_app.py \
  scripts/embodied_data_pipeline-main/scripts/normalize_aloha_task_metadata.py
! rg -n 'grasp the bottles on the table' \
  scripts/embodied_data_pipeline-main/robot_profiles/aloha.yaml
git diff --check
```

- [ ] **Step 2: Run full tests and inspect final state**

Run `python3 -m pytest -q`, report known baseline/dependency failures exactly,
then run `git status --short --branch` and `git log --oneline -8`. Completion
requires focused tests passing, no new full-suite failures, a clean worktree,
an existing runtime manifest, and migration verification with zero recognized
task-name inconsistencies.

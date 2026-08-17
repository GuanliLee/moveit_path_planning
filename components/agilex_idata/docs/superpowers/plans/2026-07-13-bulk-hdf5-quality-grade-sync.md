# Bulk HDF5 Quality Grade Sync Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a QC-report button that writes every finalized Web quality grade to HDF5 episode sidecars and strictly verifies that every HDF5 episode has a consistent grade.

**Architecture:** Reuse `authoritative_quality_grade_entries()` to freeze one Web-status snapshot and `sync_hdf5_quality_grade_entries()` for atomic writes. Add a HDF5-only validator and background workflow, expose it as a new `/api/run` stage, and bind a button inside the QC panel through the existing generic `data-stage` JavaScript handler.

**Tech Stack:** Python 3.11 standard library, `ThreadingHTTPServer`, vanilla HTML/JavaScript, pytest.

## Global Constraints

- Grades are exactly `A`, `B`, `C`, or `F`.
- Persist grades in `meta/episode_meta.json`; do not modify `aligned_joints.h5` internals.
- Preserve unrelated sidecar keys and use atomic writes.
- The bulk action must not generate or modify LeRobot datasets.
- Validation must fail closed and identify inconsistent episodes.

---

### Task 1: HDF5-only validation and workflow

**Files:**
- Modify: `tests/test_pipeline_quality_grade_consistency.py`
- Modify: `scripts/embodied_data_pipeline-main/scripts/pipeline_web_app.py`

**Interfaces:**
- Consumes: `authoritative_quality_grade_entries(cfg)` and `sync_hdf5_quality_grade_entries(entries)`.
- Produces: `validate_hdf5_quality_grade_entries(entries) -> dict[str, Any]`, `validate_hdf5_quality_grades_step(entries)`, and `hdf5_quality_grade_sync_steps(cfg) -> list[JobStep]`.

- [ ] **Step 1: Write failing validator tests**

Add tests which create two real episode sidecars, call the wished-for validator, and assert the returned episode/grade counts. Add parameterized corrupt-sidecar cases for a missing grade, mismatched manual grade, and incorrect `manual_failure`.

- [ ] **Step 2: Verify RED**

Run: `python3 -m pytest -q tests/test_pipeline_quality_grade_consistency.py -k 'validate_hdf5_quality_grade_entries or hdf5_quality_grade_sync_steps'`

Expected: FAIL because the new validator/workflow functions do not exist.

- [ ] **Step 3: Implement the validator and workflow**

Implement strict disk reads and collect errors before raising one `ValueError`:

```python
def validate_hdf5_quality_grade_entries(entries: list[dict[str, Any]]) -> dict[str, Any]:
    counts = {grade: 0 for grade in QUALITY_GRADES}
    errors = []
    for entry in entries:
        expected = normalise_quality_grade(entry.get("quality_grade"))
        sidecar = load_json_object_strict(
            Path(entry["episode_dir"]) / "meta" / "episode_meta.json"
        )
        # Compare quality_grade, manual_quality_grade, and manual_failure.
    if errors:
        raise ValueError("HDF5 质量等级完整性校验失败：\n" + "\n".join(errors))
    return {"episode_count": len(entries), "quality_grade_counts": counts}
```

Build the background steps from one authoritative snapshot:

```python
def hdf5_quality_grade_sync_steps(cfg: dict[str, Any]) -> list[JobStep]:
    entries = authoritative_quality_grade_entries(cfg)
    return [
        sync_hdf5_quality_grades_step(entries),
        validate_hdf5_quality_grades_step(entries),
    ]
```

- [ ] **Step 4: Verify GREEN**

Run: `python3 -m pytest -q tests/test_pipeline_quality_grade_consistency.py`

Expected: all tests pass.

### Task 2: API stage and QC report button

**Files:**
- Modify: `tests/test_pipeline_qc_report_actions.py`
- Modify: `scripts/embodied_data_pipeline-main/scripts/pipeline_web_app.py`

**Interfaces:**
- Consumes: `hdf5_quality_grade_sync_steps(cfg)` from Task 1.
- Produces: `/api/run` stage `sync_hdf5_grades` and QC-panel button `syncHdf5GradesBtn`.

- [ ] **Step 1: Write the failing page/route test**

Assert the source contains all of:

```python
assert 'id="syncHdf5GradesBtn"' in text
assert 'data-stage="sync_hdf5_grades"' in text
assert "批量写入 HDF5 等级" in text
assert 'stage == "sync_hdf5_grades"' in text
assert "hdf5_quality_grade_sync_steps(cfg)" in text
```

Also compare string offsets to require the button to occur after `id="qcPanel"` and before the QC table.

- [ ] **Step 2: Verify RED**

Run: `python3 -m pytest -q tests/test_pipeline_qc_report_actions.py -k bulk_hdf5_quality_grade`

Expected: FAIL because the button and stage are absent.

- [ ] **Step 3: Implement route and button**

Add this branch before the LeRobot stage:

```python
elif stage == "sync_hdf5_grades":
    job = create_job(
        "批量写入并校验 HDF5 质量等级",
        hdf5_quality_grade_sync_steps(cfg),
    )
```

Add this button to `.selection-actions` inside `#qcPanel`:

```html
<button id="syncHdf5GradesBtn" class="primary" type="button"
        data-stage="sync_hdf5_grades">批量写入 HDF5 等级</button>
```

The existing generic `[data-stage]` event binding starts and polls the job.

- [ ] **Step 4: Verify GREEN**

Run: `python3 -m pytest -q tests/test_pipeline_qc_report_actions.py tests/test_pipeline_quality_grade_consistency.py`

Expected: all tests pass.

### Task 3: Regression, deployment, and existing-data repair

**Files:**
- Verify: `scripts/embodied_data_pipeline-main/scripts/pipeline_web_app.py`
- Update data: `/home/agilex/data/stage2/hdf5_episodes/*/meta/episode_meta.json`

**Interfaces:**
- Consumes: the same `hdf5_quality_grade_sync_steps` logic used by the webpage.
- Produces: a running Web service and a zero-error HDF5 grade audit.

- [ ] **Step 1: Run focused regression and syntax checks**

Run:

```bash
python3 -m pytest -q tests/test_pipeline_qc_report_actions.py tests/test_pipeline_quality_grade_consistency.py tests/test_pipeline_task_consistency.py
python3 -m py_compile scripts/embodied_data_pipeline-main/scripts/pipeline_web_app.py
git diff --check
```

Expected: all focused tests and checks pass.

- [ ] **Step 2: Commit implementation on main**

```bash
git add scripts/embodied_data_pipeline-main/scripts/pipeline_web_app.py \
  tests/test_pipeline_qc_report_actions.py \
  tests/test_pipeline_quality_grade_consistency.py \
  docs/superpowers/plans/2026-07-13-bulk-hdf5-quality-grade-sync.md
git commit -m "feat: add bulk HDF5 quality grade sync"
```

- [ ] **Step 3: Restart and verify the webpage**

Run `systemctl --user restart collect-mobile-pipeline-qc-web.service`, wait conditionally for `curl http://127.0.0.1:8001/` to return 200, and confirm the returned HTML includes `syncHdf5GradesBtn`.

- [ ] **Step 4: Repair existing stage2 datasets**

For every direct child dataset of `/home/agilex/data/stage2/hdf5_episodes`, derive its Web config, call `authoritative_quality_grade_entries`, synchronize the entries, and immediately call `validate_hdf5_quality_grade_entries`.

- [ ] **Step 5: Audit all HDF5 episodes**

Re-enumerate all `aligned_joints.h5` source episodes and assert every sidecar has matching `quality_grade`, `manual_quality_grade`, and `manual_failure`. Report totals and A/B/C/F counts; any dataset exception makes the command exit nonzero.

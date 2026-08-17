# HDF5 and LeRobot Quality-Grade Consistency Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Guarantee that every Web-visible HDF5 episode sidecar and every corresponding LeRobot episode record contains the same final Web QC quality grade.

**Architecture:** Build one strict authoritative episode-to-grade map from finalized Web status rows, synchronize it atomically into HDF5 sidecars, pass each grade explicitly across the converter boundary, then atomically backfill and validate both LeRobot metadata files. Existing-data repair reuses the same metadata-only synchronization and validation functions and never opens Parquet or video files.

**Tech Stack:** Python 3, JSON/JSONL, argparse, pathlib, pytest.

## Global Constraints

- Accepted quality grades are exactly `A`, `B`, `C`, and `F`.
- The Web QC report's finalized `quality_grade` is authoritative.
- Store HDF5 episode grades only in `meta/episode_meta.json`; do not add an HDF5 binary attribute.
- Preserve unrelated sidecar, mapping, and LeRobot episode fields.
- Existing-data repair must not read or rewrite Parquet or video files.
- Missing, invalid, duplicate, conflicting, or unresolvable episode grades fail with the affected path and episode identity.
- All JSON and JSONL mutations use a temporary file followed by atomic replacement.

---

### Task 1: Add an explicit quality-grade contract to the converter

**Files:**
- Modify: `scripts/embodied_data_pipeline-main/lerobot_conversion/scripts/convert_hdf5_to_lerobot_v2.py:52-2600`
- Create: `tests/test_lerobot_quality_grade.py`

**Interfaces:**
- Consumes: episode metadata from `read_raw_episode()` and optional CLI `--quality-grade`.
- Produces: `normalise_quality_grade(value: Any, *, allow_empty: bool = True) -> str`, `resolve_episode_quality_grade(meta: dict[str, Any], explicit_grade: str = "") -> str`, and `instruction_meta_from_episode_meta(meta: dict[str, Any], task: str, *, authoritative_task: bool = False, quality_grade: str = "") -> dict[str, Any]`.

- [ ] **Step 1: Write failing converter contract tests**

```python
from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[1]
PIPELINE_ROOT = REPO_ROOT / "scripts" / "embodied_data_pipeline-main"
CONVERTER_PATH = PIPELINE_ROOT / "lerobot_conversion" / "scripts" / "convert_hdf5_to_lerobot_v2.py"


def load_converter():
    sys.path.insert(0, str(PIPELINE_ROOT))
    spec = importlib.util.spec_from_file_location("converter_quality_grade_test", CONVERTER_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_explicit_grade_is_written_when_sidecar_has_no_grade():
    converter = load_converter()
    grade = converter.resolve_episode_quality_grade({}, "B")
    output = converter.instruction_meta_from_episode_meta({}, "pick", quality_grade=grade)
    assert output["quality_grade"] == "B"


def test_explicit_grade_rejects_sidecar_conflict():
    converter = load_converter()
    with pytest.raises(ValueError, match="quality grade conflict"):
        converter.resolve_episode_quality_grade({"quality_grade": "C"}, "B")


def test_invalid_sidecar_grade_is_rejected_without_explicit_grade():
    converter = load_converter()
    with pytest.raises(ValueError, match="invalid quality grade"):
        converter.resolve_episode_quality_grade({"quality_grade": "D"})
```

- [ ] **Step 2: Run the tests and verify the missing API fails**

Run: `pytest -q tests/test_lerobot_quality_grade.py`

Expected: FAIL because `resolve_episode_quality_grade` and the `quality_grade` keyword do not exist.

- [ ] **Step 3: Implement strict grade resolution and CLI propagation**

Add these helpers next to `compact_quality_meta`:

```python
QUALITY_GRADES = ("A", "B", "C", "F")


def normalise_quality_grade(value: Any, *, allow_empty: bool = True) -> str:
    text = str(value or "").strip().upper()
    if not text and allow_empty:
        return ""
    if text not in QUALITY_GRADES:
        raise ValueError(f"invalid quality grade: {value!r}; expected one of {QUALITY_GRADES}")
    return text


def resolve_episode_quality_grade(meta: dict[str, Any], explicit_grade: str = "") -> str:
    collection_quality = meta.get("collection_quality")
    collection_quality = collection_quality if isinstance(collection_quality, dict) else {}
    source_value = (
        meta.get("quality_grade")
        or meta.get("manual_quality_grade")
        or collection_quality.get("grade")
        or ""
    )
    source_grade = normalise_quality_grade(source_value)
    requested_grade = normalise_quality_grade(explicit_grade)
    if source_grade and requested_grade and source_grade != requested_grade:
        raise ValueError(
            f"quality grade conflict: sidecar={source_grade}, explicit={requested_grade}"
        )
    return requested_grade or source_grade
```

Add `--quality-grade` with `choices=QUALITY_GRADES`, pass `args.quality_grade`
through both ordered and replacement `ProcessPoolExecutor` submissions, add it to
`preprocess_episode`, resolve it immediately after `read_raw_episode`, and pass
the result to `instruction_meta_from_episode_meta`. Extend that function with
the keyword-only `quality_grade: str = ""`; when non-empty, assign
`out["quality_grade"] = normalise_quality_grade(quality_grade, allow_empty=False)`
after `out.update(compact_quality_meta(meta))`.

- [ ] **Step 4: Run focused and existing converter tests**

Run: `pytest -q tests/test_lerobot_quality_grade.py tests/test_lerobot_conversion_stats.py`

Expected: PASS.

- [ ] **Step 5: Commit the converter contract**

```bash
git add scripts/embodied_data_pipeline-main/lerobot_conversion/scripts/convert_hdf5_to_lerobot_v2.py tests/test_lerobot_quality_grade.py
git commit -m "feat: enforce explicit LeRobot quality grades"
```

---

### Task 2: Synchronize authoritative Web grades into HDF5 sidecars

**Files:**
- Modify: `scripts/embodied_data_pipeline-main/scripts/pipeline_web_app.py:1840-2000,3040-3330`
- Create: `tests/test_pipeline_quality_grade_consistency.py`

**Interfaces:**
- Consumes: `dataset_status(stringify_config(cfg))["episodes"]` and `lerobot_source_episode_entries(cfg["hdf5_root"])`.
- Produces: `authoritative_quality_grade_entries(cfg: dict[str, Any]) -> list[dict[str, Any]]`, `sync_hdf5_quality_grade_entries(entries: list[dict[str, Any]]) -> int`, and `sync_hdf5_quality_grades_step(entries: list[dict[str, Any]]) -> Callable`.

- [ ] **Step 1: Write failing authoritative-map and sidecar tests**

```python
from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[1]
APP_PATH = REPO_ROOT / "scripts" / "embodied_data_pipeline-main" / "scripts" / "pipeline_web_app.py"


def load_app():
    spec = importlib.util.spec_from_file_location("quality_grade_consistency_test", APP_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_sync_hdf5_quality_grades_preserves_sidecar_and_sets_failure(tmp_path):
    app = load_app()
    episode_dir = tmp_path / "episode0"
    sidecar = episode_dir / "meta" / "episode_meta.json"
    sidecar.parent.mkdir(parents=True)
    sidecar.write_text(json.dumps({"task": "pick", "reason_codes": ["grasp_failure"]}))
    entries = [{"episode_id": "episode0", "episode_dir": episode_dir, "quality_grade": "F"}]
    changed = app.sync_hdf5_quality_grade_entries(entries)
    payload = json.loads(sidecar.read_text())
    assert changed == 1
    assert payload["quality_grade"] == "F"
    assert payload["manual_quality_grade"] == "F"
    assert payload["manual_failure"] is True
    assert payload["task"] == "pick"
    assert payload["reason_codes"] == ["grasp_failure"]


def test_sync_hdf5_default_a_clears_manual_failure(tmp_path):
    app = load_app()
    episode_dir = tmp_path / "episode1"
    entries = [{"episode_id": "episode1", "episode_dir": episode_dir, "quality_grade": "A"}]
    app.sync_hdf5_quality_grade_entries(entries)
    payload = json.loads((episode_dir / "meta" / "episode_meta.json").read_text())
    assert payload == {"quality_grade": "A", "manual_quality_grade": "A", "manual_failure": False}


def test_authoritative_entries_reject_missing_status_row(tmp_path, monkeypatch):
    app = load_app()
    monkeypatch.setattr(app, "lerobot_source_episode_entries", lambda root: [
        {"episode_id": "episode9", "episode_dir": tmp_path / "episode9", "hdf5_file": tmp_path / "episode9.h5"}
    ])
    monkeypatch.setattr(app, "dataset_status", lambda cfg: {"episodes": []})
    with pytest.raises(ValueError, match="episode9"):
        app.authoritative_quality_grade_entries({"hdf5_root": tmp_path})


def test_sync_hdf5_quality_grades_rejects_malformed_sidecar(tmp_path):
    app = load_app()
    episode_dir = tmp_path / "episode2"
    sidecar = episode_dir / "meta" / "episode_meta.json"
    sidecar.parent.mkdir(parents=True)
    sidecar.write_text("not-json", encoding="utf-8")
    entries = [{"episode_id": "episode2", "episode_dir": episode_dir, "quality_grade": "B"}]
    with pytest.raises(ValueError, match="episode_meta.json"):
        app.sync_hdf5_quality_grade_entries(entries)
```

- [ ] **Step 2: Run the tests and verify they fail on missing functions**

Run: `pytest -q tests/test_pipeline_quality_grade_consistency.py`

Expected: FAIL because the authoritative-map and synchronization functions do not exist.

- [ ] **Step 3: Implement strict entries and atomic sidecar synchronization**

Implement `authoritative_quality_grade_entries` so it rejects duplicate,
missing, or invalid status rows and attaches the normalized final grade without
a local A fallback. Read an existing sidecar with a strict helper that raises
on invalid JSON or a non-object root. Add this atomic writer and synchronization
behavior:

```python
def atomic_write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = path.with_suffix(path.suffix + ".tmp")
    tmp_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    tmp_path.replace(path)


def sync_hdf5_quality_grade_entries(entries: list[dict[str, Any]]) -> int:
    changed = 0
    for entry in entries:
        grade = normalise_quality_grade(entry.get("quality_grade"))
        if not grade:
            raise ValueError(f"{entry.get('episode_id')}: missing authoritative quality grade")
        meta_path = Path(entry["episode_dir"]) / "meta" / "episode_meta.json"
        payload = load_json_object_strict(meta_path) if meta_path.is_file() else {}
        updated = dict(payload)
        updated["quality_grade"] = grade
        updated["manual_quality_grade"] = grade
        updated["manual_failure"] = grade == "F"
        if updated != payload:
            atomic_write_json(meta_path, updated)
            changed += 1
    return changed
```

Make `quality_grade_episode_groups` and `build_renumber_plan` accept the
already-built entry list. In `lerobot_commands`, build it once, capture it in a
new synchronization step, reuse it for the renumber plan and grade commands,
and add `--quality-grade <grade>` in `lerobot_command` whenever `grade` is
present.

- [ ] **Step 4: Extend command-order tests**

Assert that the first callable stages are task preparation, HDF5 grade
synchronization, and plan writing. Also assert:

```python
command, _, _ = app.lerobot_command(cfg, "C", [3])
grade_pos = command.index("--quality-grade")
assert command[grade_pos + 1] == "C"
```

- [ ] **Step 5: Run focused Web pipeline tests**

Run: `pytest -q tests/test_pipeline_quality_grade_consistency.py tests/test_pipeline_task_consistency.py tests/test_pipeline_qc_report_actions.py`

Expected: PASS.

- [ ] **Step 6: Commit HDF5 synchronization**

```bash
git add scripts/embodied_data_pipeline-main/scripts/pipeline_web_app.py tests/test_pipeline_quality_grade_consistency.py tests/test_pipeline_task_consistency.py
git commit -m "feat: persist Web quality grades to HDF5 metadata"
```

---

### Task 3: Backfill and validate LeRobot quality metadata

**Files:**
- Modify: `scripts/embodied_data_pipeline-main/scripts/pipeline_web_app.py:1990-2020,3170-3370`
- Modify: `tests/test_pipeline_quality_grade_consistency.py`

**Interfaces:**
- Consumes: authoritative entry list, renumber plan, grade dataset mapping, and `meta/episodes.jsonl`.
- Produces: `sync_lerobot_quality_grade_metadata(cfg: dict[str, Any], entries: list[dict[str, Any]]) -> dict[str, int]`, `validate_quality_grade_consistency(cfg: dict[str, Any], entries: list[dict[str, Any]]) -> dict[str, int]`, and their callable Web workflow steps.

- [ ] **Step 1: Write failing complete/mixed backfill tests**

Create a grade fixture with two mapping records and two `episodes.jsonl` rows,
one missing `quality_grade` and one already correct. Snapshot sentinel Parquet
and video files as bytes, run synchronization twice, and assert:

```python
result = app.sync_lerobot_quality_grade_metadata(cfg, entries)
second = app.sync_lerobot_quality_grade_metadata(cfg, entries)
assert result == {"datasets": 1, "episodes": 2, "changed_files": 2}
assert second == {"datasets": 1, "episodes": 2, "changed_files": 0}
assert [row["quality_grade"] for row in app.load_jsonl(episodes_path)] == ["B", "B"]
mapping = app.load_json_file(mapping_path)
assert [row["quality_grade"] for row in mapping["episodes"]] == ["B", "B"]
assert parquet_path.read_bytes() == parquet_before
assert video_path.read_bytes() == video_before
```

Add failure tests for duplicate episode indices, an extra `episodes.jsonl` row,
an unresolvable source episode, and a mapping record whose source belongs to a
different Web grade.

- [ ] **Step 2: Run focused tests and verify synchronization API failures**

Run: `pytest -q tests/test_pipeline_quality_grade_consistency.py`

Expected: FAIL because LeRobot synchronization and validation do not exist.

- [ ] **Step 3: Implement strict JSONL loading and metadata-only backfill**

Add an atomic JSONL writer:

```python
def atomic_write_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = path.with_suffix(path.suffix + ".tmp")
    tmp_path.write_text(
        "".join(json.dumps(row, ensure_ascii=False) + "\n" for row in rows),
        encoding="utf-8",
    )
    tmp_path.replace(path)
```

Implement `sync_lerobot_quality_grade_metadata` to index authoritative entries
by resolved HDF5 file path, with episode name only as a checked compatibility
fallback for old mappings. Require unique mapping and JSONL episode indices,
require the exact same index set, update both records with the directory grade,
and write only changed metadata files atomically. Do not enumerate, open, or
write anything below `data/` or `videos/`.

- [ ] **Step 4: Implement final cross-layer validation**

Implement `validate_quality_grade_consistency` to re-read every HDF5 sidecar,
mapping record, and `episodes.jsonl` row and require:

```python
sidecar["quality_grade"] == expected_grade
sidecar["manual_quality_grade"] == expected_grade
sidecar["manual_failure"] == (expected_grade == "F")
mapping_row["quality_grade"] == expected_grade
episode_row["quality_grade"] == expected_grade
directory_grade == expected_grade
```

Collect all errors and raise one `ValueError` whose lines identify the grade,
episode index, source episode, expected grade, and actual value. Return counts
for HDF5 and LeRobot episodes on success.

- [ ] **Step 5: Replace the mapping-only step and add final validation order**

Replace `patch_lerobot_grade_mappings_step` with a step that calls the new
metadata synchronizer using the captured authoritative entries. Append the
quality consistency validation after existing task consistency validation in
`lerobot_commands`. Keep the synchronization function independently callable
so existing datasets can be repaired without executing converter commands.

- [ ] **Step 6: Run focused consistency tests**

Run: `pytest -q tests/test_pipeline_quality_grade_consistency.py tests/test_pipeline_task_consistency.py`

Expected: PASS.

- [ ] **Step 7: Commit LeRobot backfill and validation**

```bash
git add scripts/embodied_data_pipeline-main/scripts/pipeline_web_app.py tests/test_pipeline_quality_grade_consistency.py tests/test_pipeline_task_consistency.py
git commit -m "fix: backfill and validate LeRobot quality grades"
```

---

### Task 4: Run regression verification and document the operational guarantee

**Files:**
- Modify: `scripts/embodied_data_pipeline-main/README.md:623-740`

**Interfaces:**
- Consumes: completed Tasks 1-3.
- Produces: user-facing documentation and final verification evidence.

- [ ] **Step 1: Document grade synchronization and repair semantics**

Add a subsection under HDF5-to-LeRobot conversion stating that the Web final
grade is written to HDF5 `meta/episode_meta.json`, passed explicitly to the
converter, synchronized to both LeRobot metadata files, and validated per
episode. State that calling the synchronization workflow for existing outputs
changes JSON/JSONL only and does not re-encode Parquet or videos.

- [ ] **Step 2: Run syntax checks**

Run:

```bash
python3 -m py_compile \
  scripts/embodied_data_pipeline-main/scripts/pipeline_web_app.py \
  scripts/embodied_data_pipeline-main/lerobot_conversion/scripts/convert_hdf5_to_lerobot_v2.py
```

Expected: exit 0 with no output.

- [ ] **Step 3: Run all directly affected tests**

Run:

```bash
pytest -q \
  tests/test_lerobot_quality_grade.py \
  tests/test_lerobot_conversion_stats.py \
  tests/test_pipeline_quality_grade_consistency.py \
  tests/test_pipeline_task_consistency.py \
  tests/test_pipeline_qc_report_actions.py
```

Expected: all tests pass with zero failures.

- [ ] **Step 4: Run the full repository test suite**

Run: `pytest -q`

Expected: all tests pass with zero failures.

- [ ] **Step 5: Check formatting and diff integrity**

Run: `git diff --check && git status --short`

Expected: `git diff --check` exits 0; status lists only the intended source,
test, README, and plan changes.

- [ ] **Step 6: Commit documentation**

```bash
git add scripts/embodied_data_pipeline-main/README.md docs/superpowers/plans/2026-07-13-quality-grade-consistency.md
git commit -m "docs: explain quality grade consistency"
```

---

### Task 5: Persist an operator grade at Web save time

**Files:**
- Modify: `scripts/embodied_data_pipeline-main/scripts/pipeline_web_app.py:1469-1499,1918-1936`
- Modify: `tests/test_pipeline_qc_report_actions.py`

**Interfaces:**
- Consumes: `save_qc_report_quality_grade(cfg, episode_name, quality_grade, reason_label, reason_codes)`, `lerobot_source_episode_entries(cfg["hdf5_root"])`, and `sync_hdf5_quality_grade_entries(entries)`.
- Produces: `hdf5_quality_grade_entry_for_episode(cfg: dict[str, Any], episode_name: str, quality_grade: str) -> dict[str, Any]`; a successful Web save has already updated the matching HDF5 `meta/episode_meta.json`.

- [ ] **Step 1: Extend the Web-save test with an HDF5 episode and sidecar assertions**

Before calling `save_qc_report_quality_grade`, create
`hdf5_episodes/episode2/states/aligned_joints.h5` and a sidecar containing an
unrelated `task` field. After the save, assert:

```python
sidecar_payload = json.loads(sidecar.read_text(encoding="utf-8"))
assert sidecar_payload["quality_grade"] == "C"
assert sidecar_payload["manual_quality_grade"] == "C"
assert sidecar_payload["manual_failure"] is False
assert sidecar_payload["task"] == "pick"
```

- [ ] **Step 2: Add failure tests for unresolved and malformed HDF5 episodes**

```python
def test_qc_report_quality_review_rejects_missing_hdf5_episode(tmp_path):
    module = load_module()
    cfg = make_cfg(module, tmp_path)
    with pytest.raises(ValueError, match="missing HDF5 episode.*episode9"):
        module.save_qc_report_quality_grade(cfg, "episode9", "B")
    assert not module.manual_failure_hdf5_path(cfg).exists()


def test_qc_report_quality_review_rejects_malformed_hdf5_sidecar(tmp_path):
    module = load_module()
    cfg = make_cfg(module, tmp_path)
    episode_dir = make_hdf5_episode(cfg, "episode3")
    sidecar = episode_dir / "meta" / "episode_meta.json"
    sidecar.parent.mkdir(parents=True)
    sidecar.write_text("not-json", encoding="utf-8")
    with pytest.raises(ValueError, match="episode_meta.json"):
        module.save_qc_report_quality_grade(cfg, "episode3", "F")
    assert not module.manual_failure_hdf5_path(cfg).exists()
```

- [ ] **Step 3: Run the new tests and verify the sidecar behavior fails**

Run: `python3 -m pytest -q tests/test_pipeline_qc_report_actions.py`

Expected: FAIL because a successful save does not update the sidecar and missing HDF5 episodes are accepted.

- [ ] **Step 4: Resolve exactly one HDF5 episode and synchronize it before the annotation overlay**

Add:

```python
def hdf5_quality_grade_entry_for_episode(
    cfg: dict[str, Any], episode_name: str, quality_grade: str
) -> dict[str, Any]:
    matches = [
        entry
        for entry in lerobot_source_episode_entries(cfg["hdf5_root"])
        if str(entry.get("episode_id") or "").strip() == episode_name
    ]
    if not matches:
        raise ValueError(f"missing HDF5 episode for quality grade save: {episode_name}")
    if len(matches) != 1:
        raise ValueError(f"duplicate HDF5 episodes for quality grade save: {episode_name}")
    entry = dict(matches[0])
    entry["quality_grade"] = quality_grade
    return entry
```

In `save_qc_report_quality_grade`, after validating the grade and before
writing `manual_failure_annotations.json`, call:

```python
entry = hdf5_quality_grade_entry_for_episode(cfg, name, grade)
sync_hdf5_quality_grade_entries([entry])
```

This ordering ensures a malformed or unresolved HDF5 episode cannot produce a
successful annotation-only save.

- [ ] **Step 5: Run focused grade tests**

Run:

```bash
python3 -m pytest -q \
  tests/test_pipeline_qc_report_actions.py \
  tests/test_pipeline_quality_grade_consistency.py \
  tests/test_pipeline_task_consistency.py
```

Expected: PASS.

- [ ] **Step 6: Run syntax and diff checks, then commit**

Run:

```bash
python3 -m py_compile scripts/embodied_data_pipeline-main/scripts/pipeline_web_app.py
git diff --check
git status --short
```

Expected: syntax and diff checks exit 0; only the planned source, tests, and plan are modified.

Commit:

```bash
git add \
  scripts/embodied_data_pipeline-main/scripts/pipeline_web_app.py \
  tests/test_pipeline_qc_report_actions.py \
  docs/superpowers/plans/2026-07-13-quality-grade-consistency.md
git commit -m "fix: persist manual grades to HDF5 immediately"
```

# Automatic Collection Grade Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a case-insensitive optional `--grade A|B|F` argument that automatically finalizes every successfully saved episode with that grade while preserving the current manual review flow by default.

**Architecture:** The staged launcher owns the public CLI, validates and normalizes the option, and passes it to the existing collection controller through `COLLECTION_AUTO_GRADE`. The controller keeps its current save and review state machine, but immediately feeds a validated review payload into `finalize_quality_review_from_json` after a save when automatic grading is enabled.

**Tech Stack:** Bash, embedded Python/JSON already present in the scripts, pytest static and subprocess regression tests.

## Global Constraints

- Accept both `--grade a` and `--grade=A`; normalize case to exactly `A`, `B`, or `F`.
- Reject missing or invalid values before any collection module starts.
- Without `--grade`, preserve the existing manual A/B/F/discard dialog and behavior.
- Automatic grading applies to every new episode for the lifetime of this launcher process.
- If quality metadata persistence fails, keep the episode pending for manual retry and do not advance the episode or publish save success.
- Do not change ROS topic names, message types, or QoS.
- Do not modify or restore the unrelated test-file deletions currently present in the main working tree.

---

### Task 1: Parse, validate, and propagate the automatic grade

**Files:**
- Modify: `scripts/collection/collect_mobile_pipeline_web_staged.sh`
- Create: `tests/test_collection_auto_grade.py`

**Interfaces:**
- Consumes: launcher arguments `--grade VALUE` and `--grade=VALUE`.
- Produces: normalized shell variable `AUTO_GRADE` and child environment variable `COLLECTION_AUTO_GRADE` containing `""`, `A`, `B`, or `F`.

- [ ] **Step 1: Write the failing CLI and propagation tests**

Create `tests/test_collection_auto_grade.py`:

```python
from __future__ import annotations

import subprocess
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
STAGED_SCRIPT = REPO_ROOT / "scripts" / "collection" / "collect_mobile_pipeline_web_staged.sh"
COLLECTION_SCRIPT = REPO_ROOT / "scripts" / "collection" / "collect_mobile_episode_web.sh"


def run_staged(*args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["bash", str(STAGED_SCRIPT), *args],
        cwd=REPO_ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=10,
        check=False,
    )


def test_help_documents_optional_automatic_grade():
    result = run_staged("--help")

    assert result.returncode == 0
    assert "--grade A|B|F" in result.stdout
    assert "默认仍为人工选择" in result.stdout


def test_grade_parser_rejects_missing_empty_and_invalid_values():
    text = STAGED_SCRIPT.read_text(encoding="utf-8")

    assert "--grade)" in text
    assert "--grade=*" in text
    assert "AUTO_GRADE_ARG_SEEN" in text
    assert "--grade 需要质量等级参数" in text
    assert "--grade 只能是 A、B 或 F" in text
    assert "[:lower:]" in text
    assert "[:upper:]" in text


def test_launcher_normalizes_and_passes_grade_to_collection_process():
    text = STAGED_SCRIPT.read_text(encoding="utf-8")

    assert "AUTO_GRADE_ARG" in text
    assert "AUTO_GRADE_ARG_SEEN" in text
    assert "--grade=*" in text
    assert "AUTO_GRADE=" in text
    assert 'COLLECTION_AUTO_GRADE="${AUTO_GRADE}"' in text
```

- [ ] **Step 2: Run the tests and verify RED**

Run:

```bash
python3 -m pytest -q tests/test_collection_auto_grade.py
```

Expected: three failures because the help, parser, validation, and environment propagation do not exist. Only `--help` is executed; invalid pre-feature arguments are checked statically so the old positional parser cannot accidentally launch collection hardware.

- [ ] **Step 3: Implement the minimal launcher CLI**

In `collect_mobile_pipeline_web_staged.sh`:

1. Add `[--grade A|B|F]` to the usage signatures and an example using `--grade a`.
2. Add this help description:

```text
  --grade A|B|F                 自动为每条成功保存的数据记录该等级；默认仍为人工选择
```

3. Initialize the parser state:

```bash
AUTO_GRADE_ARG=""
AUTO_GRADE_ARG_SEEN=0
```

4. Add both option forms before the positional fallback:

```bash
        --grade)
            [ "$#" -ge 2 ] || {
                echo "[错误] --grade 需要质量等级参数" >&2
                exit 1
            }
            AUTO_GRADE_ARG="$2"
            AUTO_GRADE_ARG_SEEN=1
            shift 2
            ;;
        --grade=*)
            AUTO_GRADE_ARG="${1#*=}"
            AUTO_GRADE_ARG_SEEN=1
            shift
            ;;
```

5. Normalize and validate immediately after option parsing and before dataset validation:

```bash
if [ "${AUTO_GRADE_ARG_SEEN}" -eq 1 ] && [ -z "${AUTO_GRADE_ARG}" ]; then
    echo "[错误] --grade 需要质量等级参数" >&2
    exit 1
fi
AUTO_GRADE="$(printf '%s' "${AUTO_GRADE_ARG}" | tr '[:lower:]' '[:upper:]')"
case "${AUTO_GRADE}" in
    ""|A|B|F) ;;
    *)
        echo "[错误] --grade 只能是 A、B 或 F: ${AUTO_GRADE_ARG}" >&2
        exit 1
        ;;
esac
```

6. Add this entry to the `env` array in `start_collection`:

```bash
COLLECTION_AUTO_GRADE="${AUTO_GRADE}"
```

7. Print the configured mode in the final startup summary:

```bash
echo "  COLLECTION_AUTO_GRADE=${AUTO_GRADE:-<manual>}"
```

- [ ] **Step 4: Run the tests and verify GREEN**

Run:

```bash
python3 -m pytest -q tests/test_collection_auto_grade.py
bash -n scripts/collection/collect_mobile_pipeline_web_staged.sh
```

Expected: `3 passed`; Shell syntax exits 0.

- [ ] **Step 5: Commit Task 1**

```bash
git add scripts/collection/collect_mobile_pipeline_web_staged.sh tests/test_collection_auto_grade.py
git commit -m "feat: add automatic grade launcher option"
```

### Task 2: Finalize saved episodes automatically through the existing review path

**Files:**
- Modify: `scripts/collection/collect_mobile_episode_web.sh`
- Modify: `tests/test_collection_auto_grade.py`

**Interfaces:**
- Consumes: `COLLECTION_AUTO_GRADE` from Task 1 and existing `saved_episode` after MCAP/metadata save.
- Produces: a call to `finalize_quality_review_from_json(payload)` with `action=review`, the configured grade, empty `reason_codes`, and an empty `reason_note`; on failure, the existing function leaves `QUALITY_REVIEW_PENDING=1`.

- [ ] **Step 1: Write the failing automatic-finalization tests**

Append to `tests/test_collection_auto_grade.py`:

```python
def test_collection_controller_validates_internal_automatic_grade():
    text = COLLECTION_SCRIPT.read_text(encoding="utf-8")

    assert 'COLLECTION_AUTO_GRADE="${COLLECTION_AUTO_GRADE:-}"' in text
    assert "COLLECTION_AUTO_GRADE=\"$(printf '%s'" in text
    assert "COLLECTION_AUTO_GRADE 只能是 A、B 或 F" in text


def test_saved_episode_uses_existing_review_path_only_in_auto_mode():
    text = COLLECTION_SCRIPT.read_text(encoding="utf-8")
    save_block = text.split("save_current_episode() {", 1)[1].split(
        "check_required_topics() {",
        1,
    )[0]

    pending_at = save_block.index("QUALITY_REVIEW_PENDING=1")
    auto_branch_at = save_block.index('if [ -n "${COLLECTION_AUTO_GRADE}" ]; then')
    finalize_at = save_block.index("finalize_quality_review_from_json")
    manual_at = save_block.index("已保存，请选择质量等级或放弃")
    assert pending_at < auto_branch_at < finalize_at < manual_at
    assert '\"action\":\"review\"' in save_block
    assert '\"reason_codes\":[]' in save_block
    assert '\"reason_note\":\"\"' in save_block


def test_automatic_grade_does_not_publish_review_status_before_finalize():
    text = COLLECTION_SCRIPT.read_text(encoding="utf-8")
    save_block = text.split("save_current_episode() {", 1)[1].split(
        "check_required_topics() {",
        1,
    )[0]
    auto_block = save_block.split(
        'if [ -n "${COLLECTION_AUTO_GRADE}" ]; then',
        1,
    )[1].split("    else", 1)[0]

    assert 'write_status "review"' not in auto_block
    assert "finalize_quality_review_from_json" in auto_block


def test_automatic_review_reuses_recoverable_metadata_failure_path():
    text = COLLECTION_SCRIPT.read_text(encoding="utf-8")
    save_block = text.split("save_current_episode() {", 1)[1].split(
        "check_required_topics() {",
        1,
    )[0]
    review_block = text.split("finalize_quality_review_from_json() {", 1)[1].split(
        "qc_aloha_hdf5_episode() {",
        1,
    )[0]

    assert 'if [ -n "${COLLECTION_AUTO_GRADE}" ]; then' in save_block
    assert "finalize_quality_review_from_json" in save_block
    failure_block = review_block.split('if ! "${cmd[@]}"; then', 1)[1].split("fi", 1)[0]
    assert 'write_status "review"' in failure_block
    assert "QUALITY_REVIEW_PENDING=0" not in failure_block
    assert "CURRENT_EPISODE=$((CURRENT_EPISODE + 1))" not in failure_block
```

- [ ] **Step 2: Run the new tests and verify RED**

Run:

```bash
python3 -m pytest -q tests/test_collection_auto_grade.py
```

Expected: the three Task 2 tests fail because the controller has no automatic-grade configuration or save branch. The recovery test first requires the new automatic branch, so it also proves RED before checking the existing metadata-failure behavior.

- [ ] **Step 3: Validate the internal environment value**

After controller configuration is loaded, normalize and validate:

```bash
COLLECTION_AUTO_GRADE="${COLLECTION_AUTO_GRADE:-}"
COLLECTION_AUTO_GRADE="$(printf '%s' "${COLLECTION_AUTO_GRADE}" | tr '[:lower:]' '[:upper:]')"
case "${COLLECTION_AUTO_GRADE}" in
    ""|A|B|F) ;;
    *)
        echo "[错误] COLLECTION_AUTO_GRADE 只能是 A、B 或 F: ${COLLECTION_AUTO_GRADE}" >&2
        exit 1
        ;;
esac
```

Also document `COLLECTION_AUTO_GRADE=<empty>` in the controller's environment help.

- [ ] **Step 4: Add automatic finalization after a successful raw save**

Replace the final review-status write in `save_current_episode` with:

```bash
    if [ -n "${COLLECTION_AUTO_GRADE}" ]; then
        log "episode${saved_episode} 已保存，正在自动记录质量等级 ${COLLECTION_AUTO_GRADE}。"
        finalize_quality_review_from_json \
            "{\"episode_id\":\"${saved_episode}\",\"action\":\"review\",\"grade\":\"${COLLECTION_AUTO_GRADE}\",\"reason_codes\":[],\"reason_note\":\"\"}"
    else
        write_status "review" "${CURRENT_EPISODE}" \
            "episode${saved_episode} 已保存，请选择质量等级或放弃。"
    fi
```

This deliberately sets the internal `QUALITY_REVIEW_PENDING` variables before the call but does not publish an intermediate review status, so successful automatic grading cannot briefly open the Web dialog. The existing metadata-failure branch publishes review status and remains recoverable through the Web dialog.

- [ ] **Step 5: Run Task 2 and related tests**

Run:

```bash
python3 -m pytest -q tests/test_collection_auto_grade.py
python3 -m pytest -q tests/test_collection_failure_ui.py
bash -n scripts/collection/collect_mobile_episode_web.sh
bash -n scripts/collection/collect_mobile_pipeline_web_staged.sh
```

Expected: all tests pass and both syntax checks exit 0.

- [ ] **Step 6: Commit Task 2**

```bash
git add scripts/collection/collect_mobile_episode_web.sh tests/test_collection_auto_grade.py
git commit -m "feat: automatically finalize collection grades"
```

### Task 3: Final regression and integration verification

**Files:**
- Verify: `scripts/collection/collect_mobile_pipeline_web_staged.sh`
- Verify: `scripts/collection/collect_mobile_episode_web.sh`
- Verify: `tests/test_collection_auto_grade.py`

**Interfaces:**
- Consumes: completed Task 1 and Task 2 commits.
- Produces: evidence that automatic grading works without regressing manual review or the retained save-success topic.

- [ ] **Step 1: Run focused tests and syntax checks**

```bash
python3 -m pytest -q tests/test_collection_auto_grade.py tests/test_collection_failure_ui.py
bash -n scripts/collection/collect_mobile_pipeline_web_staged.sh
bash -n scripts/collection/collect_mobile_episode_web.sh
awk 'found && $0 == "PY" {exit} /^from __future__ import annotations$/ {count++; if (count == 2) found=1} found' \
  scripts/collection/collect_mobile_pipeline_web_staged.sh | \
  python3 -c 'import ast, sys; ast.parse(sys.stdin.read())'
git diff --check
```

Expected: focused tests pass; all syntax and formatting checks exit 0.

- [ ] **Step 2: Run the repository suite in the established OpenPI environment**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/agilex/openpi/.venv/bin/python -m pytest -q
```

Expected comparison baseline: before this feature the repository reports `10 failed, 164 passed`; no new failure may be introduced. The known failures are in LeRobot conversion stats, missing `quality_pipeline.manual_failures`, and pipeline failure-path expectations.

- [ ] **Step 3: Review the final diff and repository state**

```bash
git diff main...HEAD --check
git diff main...HEAD --stat
git status --short
```

Expected: only the two scripts, the new test, and feature documentation/plan are changed in the feature branch; the feature worktree is clean.

# Collection Target Dropdowns Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace startup/free-form collection targets with validated left/right dropdown selections that generate hand-aware stage-2 and stage-4 prompts in fixed and full staged collection.

**Architecture:** A focused `collection_targets.py` module owns the exact catalog, validation, prompt formatting, and stage-template tokens. The shared collection web controller renders dropdowns, persists selections through `/config`, and uses the helper while producing metadata; both launcher scripts start with empty targets and the staged launcher writes a target-independent template.

**Tech Stack:** Bash, embedded Python 3 HTTP server, vanilla HTML/JavaScript, pytest.

## Global Constraints

- Both dropdowns contain one empty option followed by the 16 approved English names in the exact approved order.
- Both targets start empty and terminal target arguments/environment defaults do not choose them.
- Stage 2 uses exact hand-specific `Grasp` sentences and joins dual targets with `. and `.
- Stages 1, 3, and 5 retain their existing text.
- Stage 4 retains its existing hand-specific `Place` behavior using runtime selections.
- Explicitly empty stage-2 and stage-4 prompts remain empty in saved metadata.
- Existing unrelated deleted tests and worktree changes must remain untouched.

---

### Task 1: Shared target catalog and prompt formatter

**Files:**
- Create: `scripts/collection/collection_targets.py`
- Create: `tests/test_collection_target_dropdowns.py`

**Interfaces:**
- Produces: `TARGET_OPTIONS: tuple[str, ...]`, `GRASP_PROMPT_TOKEN: str`, `PLACE_PROMPT_TOKEN: str`, `validate_target(value: object) -> str`, `grasp_instruction(left: object, right: object) -> str`, `place_instruction(left: object, right: object) -> str`, `render_target_prompts(text: str, left: object, right: object) -> str`, and `stage_instruction_templates() -> list[str]`.

- [ ] **Step 1: Write failing unit tests**

Add tests that assert the exact 16-item tuple, acceptance of empty/known values, rejection of unknown values, all four grasp mappings, all four place mappings, token rendering, and the unchanged stage 1/3/5 templates.

- [ ] **Step 2: Run the tests and verify RED**

Run: `pytest -q tests/test_collection_target_dropdowns.py`

Expected: collection fails because `scripts/collection/collection_targets.py` does not exist.

- [ ] **Step 3: Add the minimal helper implementation**

Implement the catalog and pure functions. Dual grasp output must be exactly:

```python
f"Grasp {left} with the left hand. and Grasp {right} with the right hand."
```

The stage template list must be:

```python
[
    "Move chassis to shelf.",
    GRASP_PROMPT_TOKEN,
    "Move chassis to cart.",
    PLACE_PROMPT_TOKEN,
    "Move chassis to start.",
]
```

- [ ] **Step 4: Run the focused tests and verify GREEN**

Run: `pytest -q tests/test_collection_target_dropdowns.py`

Expected: all helper tests pass.

- [ ] **Step 5: Commit the helper and tests**

```bash
git add scripts/collection/collection_targets.py tests/test_collection_target_dropdowns.py
git commit -m "feat: add collection target prompt model"
```

### Task 2: Hand-aware fixed-stage metadata

**Files:**
- Modify: `scripts/collection/collection_episode_metadata.py`
- Modify: `tests/test_collection_target_dropdowns.py`

**Interfaces:**
- Consumes: `grasp_instruction(left, right)` from Task 1.
- Produces: optional `left_target` and `right_target` parameters for `ensure_info_payload`, `prepare_episode`, and `review_episode`; CLI flags `--left-target` and `--right-target` for both metadata commands while retaining `--target` for scene matching compatibility.

- [ ] **Step 1: Add failing metadata tests**

Test explicit empty/empty, left-only, right-only, and dual `prepare_episode` calls. Assert `mark.full-instructions-en[0]` follows the exact stage-2 mapping, especially right-only and empty/empty.

- [ ] **Step 2: Run metadata tests and verify RED**

Run: `pytest -q tests/test_collection_target_dropdowns.py -k metadata`

Expected: calls fail because hand-specific keyword arguments are not supported.

- [ ] **Step 3: Delegate metadata fallback formatting to the helper**

Add optional hand-specific parameters without breaking existing callers that only pass `targets`. When hand parameters are provided, including explicit empty strings, use `grasp_instruction`; otherwise preserve the legacy list fallback. Add CLI flags and pass them through `main`.

- [ ] **Step 4: Run metadata and helper tests**

Run: `pytest -q tests/test_collection_target_dropdowns.py`

Expected: all tests pass.

- [ ] **Step 5: Commit metadata behavior**

```bash
git add scripts/collection/collection_episode_metadata.py tests/test_collection_target_dropdowns.py
git commit -m "feat: write hand-aware collection metadata"
```

### Task 3: Shared dropdown UI and runtime target application

**Files:**
- Modify: `scripts/collection/collect_mobile_episode_web.sh`
- Modify: `tests/test_collection_target_dropdowns.py`

**Interfaces:**
- Consumes: the Task 1 helper module from embedded Python blocks.
- Produces: status field `target_options: list[str]`; `<select id="target_bottle_a">` and `<select id="target_bottle_b">`; validated `/config` target application; hand-specific metadata CLI arguments.

- [ ] **Step 1: Add failing controller contract tests**

Assert the source contains dropdown labels `左手目标物品` and `右手目标物品`, both target controls are selects, status serializes `target_options`, config parsing imports `validate_target`, target change listeners persist the choice, and metadata commands include `--left-target`/`--right-target` while scene matching still receives non-empty `--target` values.

- [ ] **Step 2: Run controller tests and verify RED**

Run: `pytest -q tests/test_collection_target_dropdowns.py -k web_controller`

Expected: assertions fail on the current text inputs and missing target catalog/status behavior.

- [ ] **Step 3: Implement server-side target state**

Default both target variables to empty, export the helper path/catalog to status generation, parse `target_options`, validate values before evaluating config exports, and preserve the existing recording/review mutation guard. Pass hand-specific target flags into metadata prepare/review commands.

- [ ] **Step 4: Render prompt tokens and preserve explicit blanks**

In both preset-reading embedded Python blocks, import `render_target_prompts` and apply it after legacy bottle placeholder replacement. When an explicit segment exists with an empty rendered description, save the empty string instead of replacing it with `Stage N`.

- [ ] **Step 5: Implement dropdowns and persistence**

Replace both text inputs with selects, populate them from `status.target_options` with `空` first, and submit on change. After submitting, poll `/status` until both selected target values match so an automatic state-machine start cannot reuse stale values. Keep the existing disabled-state behavior during recording/review.

- [ ] **Step 6: Run focused tests and shell syntax**

Run:

```bash
pytest -q tests/test_collection_target_dropdowns.py
bash -n scripts/collection/collect_mobile_episode_web.sh
```

Expected: all tests pass and shell parsing exits 0.

- [ ] **Step 7: Commit the shared controller**

```bash
git add scripts/collection/collect_mobile_episode_web.sh tests/test_collection_target_dropdowns.py
git commit -m "feat: select collection targets in web UI"
```

### Task 4: Remove launcher target arguments and generate target-independent presets

**Files:**
- Modify: `scripts/collection/collect_mobile_pipeline_web_staged.sh`
- Modify: `scripts/collection/collect_mobile_pipeline_web_fixed_stage.sh`
- Modify: `tests/test_collection_target_dropdowns.py`

**Interfaces:**
- Consumes: `stage_instruction_templates()` and prompt tokens from Task 1.
- Produces: launcher CLI limited to dataset directory, start index, and grade; empty initial targets; five-stage preset containing runtime-rendered target tokens.

- [ ] **Step 1: Add failing launcher tests**

Assert both help texts omit target arguments, staged parsing omits target flags/variables and startup target environment fallbacks, the auto-preset writer imports `stage_instruction_templates`, fixed wrapper forwards dataset/index without adding targets, and stages 1/3/5 remain exact.

- [ ] **Step 2: Run launcher tests and verify RED**

Run: `pytest -q tests/test_collection_target_dropdowns.py -k launcher`

Expected: assertions fail because target arguments and startup preset interpolation still exist.

- [ ] **Step 3: Simplify the staged launcher**

Remove target flags and positional parsing, initialize both targets empty, remove startup arm-mode validation, use one runtime preset filename, and generate its five template stages by importing Task 1. Keep data-dir, start-index, grade, camera, state-machine, pipeline, and visualizer behavior unchanged.

- [ ] **Step 4: Update the fixed-stage wrapper**

Remove target arguments from usage/comments while preserving fixed-stage validation and forwarding of supported staged arguments.

- [ ] **Step 5: Run focused tests and both syntax checks**

Run:

```bash
pytest -q tests/test_collection_target_dropdowns.py
bash -n scripts/collection/collect_mobile_pipeline_web_staged.sh
bash -n scripts/collection/collect_mobile_pipeline_web_fixed_stage.sh
```

Expected: all tests pass and both scripts parse successfully.

- [ ] **Step 6: Commit launcher changes**

```bash
git add scripts/collection/collect_mobile_pipeline_web_staged.sh \
  scripts/collection/collect_mobile_pipeline_web_fixed_stage.sh \
  tests/test_collection_target_dropdowns.py
git commit -m "feat: move collection targets out of launcher CLI"
```

### Task 5: Regression verification

**Files:**
- Verify all files changed in Tasks 1-4.

**Interfaces:**
- Consumes: all preceding tasks.
- Produces: fresh verification evidence and a clean scoped diff.

- [ ] **Step 1: Run complete focused tests**

Run: `pytest -q tests/test_collection_target_dropdowns.py tests/test_collection_auto_grade.py tests/test_selectable_camera_qc.py`

Expected: all available tests pass.

- [ ] **Step 2: Run syntax and compile checks**

```bash
bash -n scripts/collection/collect_mobile_episode_web.sh
bash -n scripts/collection/collect_mobile_pipeline_web_staged.sh
bash -n scripts/collection/collect_mobile_pipeline_web_fixed_stage.sh
python3 -m py_compile scripts/collection/collection_targets.py scripts/collection/collection_episode_metadata.py
```

Expected: every command exits 0.

- [ ] **Step 3: Check whitespace and scope**

```bash
git diff --check HEAD~4..HEAD
git status --short
```

Expected: no whitespace errors; status contains only the user's pre-existing unrelated deletions.

- [ ] **Step 4: Review requirements against the diff**

Confirm exact catalog/order, empty defaults, both dropdowns, four stage-2 cases, four stage-4 cases, target metadata synchronization, terminal target removal, and unchanged stages 1/3/5.

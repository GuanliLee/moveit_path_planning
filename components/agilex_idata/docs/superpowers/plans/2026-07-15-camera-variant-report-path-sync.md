# Camera Variant Report Path Sync Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the selectable camera QC Web console load HDF5 data and QC reports from the directory matching the selected four-camera, three-camera-global, or three-camera-front configuration.

**Architecture:** Keep `derive_paths()` as the single backend path authority, adding a narrow remapper that only rewrites paths matching the generated `<camera_namespace>/<artifact_directory>` structure. Update the embedded Web UI so a parent with exactly one MCAP dataset selects that child automatically, and make global head the default head choice when entering three-camera mode. Status, report, replay, and job endpoints continue consuming one derived configuration.

**Tech Stack:** Python 3, `pathlib`, embedded vanilla JavaScript, Bash, pytest, Node.js, systemd user service.

## Global Constraints

- Port 8001 continues to start in four-camera mode.
- Three-camera mode defaults to `three_camera_global` and retains `three_camera_front`.
- Generated namespaces are exactly `four_camera`, `three_camera_global`, and `three_camera_front`.
- Custom HDF5 and QC paths outside the generated namespace structure remain authoritative.
- No HDF5, QC report, MCAP, or LeRobot data is migrated or rewritten.
- Do not restart the service while a conversion or QC job is active.
- Preserve unrelated existing worktree changes.

---

## File Structure

- `scripts/embodied_data_pipeline-main/scripts/pipeline_web_app.py`: camera variant resolution, effective paths, dataset discovery, and embedded UI.
- `scripts/collection/collect_mobile_pipeline_qc_web_four_camera.sh`: selectable service startup defaults.
- `tests/test_selectable_camera_qc.py`: backend, launcher, and embedded-JavaScript regression tests.
- `README.md`: concise operator documentation.
- `scripts/embodied_data_pipeline-main/README.md`: detailed launcher and directory documentation.

### Task 1: Remap generated HDF5 and QC paths

**Files:**
- Modify: `tests/test_selectable_camera_qc.py`
- Modify: `scripts/embodied_data_pipeline-main/scripts/pipeline_web_app.py:152-163`
- Modify: `scripts/embodied_data_pipeline-main/scripts/pipeline_web_app.py:462-473`
- Modify: `scripts/embodied_data_pipeline-main/scripts/pipeline_web_app.py:586-714`

**Interfaces:**
- Consumes: `resolve_camera_variant(payload: dict[str, Any]) -> dict[str, Any]`.
- Produces: `remap_generated_camera_path(value: str, namespace: str, artifact_dir: str) -> str`.
- Produces: `data_root_from_hdf5_path(path: Path, fallback: Path, camera_namespaces: Collection[str] = ()) -> Path`.

- [ ] **Step 1: Write failing backend path tests**

Add these behaviors to `tests/test_selectable_camera_qc.py`:

```python
@pytest.mark.parametrize(
    ("source_namespace", "camera_count", "head_source", "namespace"),
    [
        ("four_camera", 3, "global", "three_camera_global"),
        ("four_camera", 3, "front", "three_camera_front"),
        ("three_camera_global", 4, "global", "four_camera"),
    ],
)
def test_switching_camera_mode_remaps_generated_hdf5_and_qc_overrides(
    monkeypatch, tmp_path, source_namespace, camera_count, head_source, namespace
):
    stale_hdf5 = tmp_path / "data" / source_namespace / "hdf5_episodes" / "demo"
    stale_qc = tmp_path / "data" / source_namespace / "qc_reports"
    _, cfg = derive_selectable_config(
        monkeypatch, tmp_path,
        camera_count=camera_count,
        head_camera_source=head_source,
        hdf5_root=str(stale_hdf5),
        qc_root=str(stale_qc),
    )
    assert cfg["hdf5_root"] == tmp_path / "data" / namespace / "hdf5_episodes" / "demo"
    assert cfg["qc_root"] == tmp_path / "data" / namespace / "qc_reports"


def test_camera_mode_keeps_non_generated_custom_path_overrides(monkeypatch, tmp_path):
    custom_hdf5 = tmp_path / "external" / "episodes"
    custom_qc = tmp_path / "external" / "reports"
    _, cfg = derive_selectable_config(
        monkeypatch, tmp_path,
        camera_count=3,
        head_camera_source="global",
        hdf5_root=str(custom_hdf5),
        qc_root=str(custom_qc),
    )
    assert cfg["hdf5_root"] == custom_hdf5.resolve()
    assert cfg["qc_root"] == custom_qc.resolve()


def test_namespaced_hdf5_only_input_infers_matching_qc_root(monkeypatch, tmp_path):
    set_selectable_web_environment(monkeypatch, tmp_path)
    app = load_module(PIPELINE_WEB_APP, f"namespaced_hdf5_only_{len(sys.modules)}")
    source = tmp_path / "data" / "four_camera" / "hdf5_episodes" / "demo"
    cfg = app.derive_paths({
        "hdf5_root": str(source),
        "camera_count": 3,
        "head_camera_source": "global",
    })
    assert cfg["data_root"] == (tmp_path / "data").resolve()
    assert cfg["hdf5_root"] == (tmp_path / "data" / "three_camera_global" / "hdf5_episodes" / "demo").resolve()
    assert cfg["qc_root"] == (tmp_path / "data" / "three_camera_global" / "qc_reports").resolve()
```

Also create one `dataset_status()` regression with a `marker` in each namespace's `batch_summary.json`; selecting three-camera global while supplying stale four-camera inputs must return the global marker and global report path.

- [ ] **Step 2: Run tests and verify RED**

Run:

```bash
pytest -q tests/test_selectable_camera_qc.py -k 'remaps_generated or keeps_non_generated or namespaced_hdf5_only or status_loads_report'
```

Expected: generated-path assertions fail because stale explicit paths remain authoritative; the custom-path characterization passes.

- [ ] **Step 3: Implement minimal remapping**

Add:

```python
CAMERA_OUTPUT_NAMESPACES = frozenset(
    {"four_camera", "three_camera_global", "three_camera_front"}
)


def remap_generated_camera_path(value: str, namespace: str, artifact_dir: str) -> str:
    if not value or namespace not in CAMERA_OUTPUT_NAMESPACES:
        return value
    parts = list(Path(value).expanduser().parts)
    for index in range(len(parts) - 1):
        if parts[index] in CAMERA_OUTPUT_NAMESPACES and parts[index + 1] == artifact_dir:
            parts[index] = namespace
            return str(Path(*parts))
    return value
```

Import `Collection` from `collections.abc`, then extend root inference with:

```python
def data_root_from_hdf5_path(
    path: Path,
    fallback: Path,
    camera_namespaces: Collection[str] = (),
) -> Path:
    parts = list(path.parts)
    if "hdf5_episodes" in parts:
        idx = parts.index("hdf5_episodes")
        if idx > 0:
            base = Path(*parts[:idx]).resolve()
            return base.parent if base.name in camera_namespaces else base
    if path.is_file() or path.suffix.lower() in {".h5", ".hdf5"}:
        if path.parent.name == "states":
            return path.parent.parent.parent.resolve()
        return path.parent.resolve()
    return path.parent.resolve() if path.name else fallback.resolve()
```

In `derive_paths()`, resolve the selected namespace first, remap `hdf5_root` and `qc_root` text only for selectable generated namespaces, use remapped HDF5 text for inference, pass `CAMERA_OUTPUT_NAMESPACES` into root inference on selectable pages, and use remapped QC text in `path_from_user()`.

- [ ] **Step 4: Verify GREEN**

Run:

```bash
pytest -q tests/test_selectable_camera_qc.py
```

Expected: all tests pass.

- [ ] **Step 5: Commit**

```bash
git add tests/test_selectable_camera_qc.py scripts/embodied_data_pipeline-main/scripts/pipeline_web_app.py
git commit -m "fix: resolve reports by camera configuration"
```

### Task 2: Automatically select a sole nested MCAP dataset

**Files:**
- Modify: `tests/test_selectable_camera_qc.py`
- Modify: `scripts/embodied_data_pipeline-main/scripts/pipeline_web_app.py:6848-6895`

**Interfaces:**
- Consumes: `/api/mcap-datasets` response `{parent_is_dataset, selected_path, datasets}`.
- Produces: `renderMcapDatasetPicker(data) -> boolean` that fills the sole child or requires a choice among multiple children.

- [ ] **Step 1: Write a failing embedded-JavaScript test**

Add a Node harness that extracts `renderMcapDatasetPicker()` and stubs the `mcapPath`, datalist, select, and hint elements. The test is:

```python
def test_mcap_picker_automatically_selects_only_nested_dataset(monkeypatch, tmp_path):
    set_selectable_web_environment(monkeypatch, tmp_path)
    app = load_module(PIPELINE_WEB_APP, f"sole_mcap_picker_{len(sys.modules)}")
    state = run_mcap_dataset_picker(
        app.HTML,
        initial_path="/data/stage2_new/scene1",
        datasets=[{
            "name": "20260715_scene1",
            "path": "/data/stage2_new/scene1/20260715_scene1",
            "mcap_count": 156,
        }],
    )
    assert state["changed"] is True
    assert state["mcapPath"] == "/data/stage2_new/scene1/20260715_scene1"
    assert state["selectHidden"] is True
```

- [ ] **Step 2: Run the test and verify RED**

Run:

```bash
pytest -q tests/test_selectable_camera_qc.py::test_mcap_picker_automatically_selects_only_nested_dataset
```

Expected: FAIL because the renderer keeps the parent path and displays a selector for one child.

- [ ] **Step 3: Implement sole-child selection**

After the empty list branch, add:

```javascript
if (datasets.length === 1) {
  const only = datasets[0];
  if (mcapInput.value !== only.path) {
    mcapInput.value = only.path;
    changed = true;
  }
  select.innerHTML = `<option value="${escapeHtml(only.path)}">${escapeHtml(only.name || only.path)}</option>`;
  select.value = only.path;
  select.classList.add("hidden");
  hint.textContent = `仅发现一个 MCAP 数据集，已自动选择 ${only.name || only.path}。`;
  return changed;
}
```

For multiple datasets, clear `mcapPath` when its current value is not one of the discovered datasets, so the parent cannot be derived as a raw dataset.

- [ ] **Step 4: Verify GREEN and commit**

Run:

```bash
pytest -q tests/test_selectable_camera_qc.py
```

Expected: all tests pass.

Then:

```bash
git add tests/test_selectable_camera_qc.py scripts/embodied_data_pipeline-main/scripts/pipeline_web_app.py
git commit -m "fix: select sole nested mcap dataset"
```

### Task 3: Default three-camera mode to global head

**Files:**
- Modify: `tests/test_selectable_camera_qc.py`
- Modify: `scripts/collection/collect_mobile_pipeline_qc_web_four_camera.sh:22`
- Modify: `scripts/embodied_data_pipeline-main/scripts/pipeline_web_app.py:282-288`
- Modify: `scripts/embodied_data_pipeline-main/scripts/pipeline_web_app.py:6494-6498`
- Modify: `scripts/embodied_data_pipeline-main/scripts/pipeline_web_app.py:6785-6788`
- Modify: `README.md:50-58`
- Modify: `scripts/embodied_data_pipeline-main/README.md:132-154`

**Interfaces:**
- Consumes: `PIPELINE_HEAD_CAMERA_SOURCE` and `headCameraSource` selector state.
- Produces: default head source `global`; explicit `front` remains accepted.

- [ ] **Step 1: Change expectations and verify RED**

Require:

```python
assert 'PIPELINE_HEAD_CAMERA_SOURCE="${PIPELINE_HEAD_CAMERA_SOURCE:-global}"' in selectable_text
assert selectable_state["head"] == "global"
```

Run:

```bash
pytest -q tests/test_selectable_camera_qc.py -k 'launcher_default_ports or empty_page_bootstraps'
```

Expected: FAIL because the launcher and inactive selector currently use `front`.

- [ ] **Step 2: Implement the global default**

Set the launcher default:

```bash
export PIPELINE_HEAD_CAMERA_SOURCE="${PIPELINE_HEAD_CAMERA_SOURCE:-global}"
```

Set the selectable backend fallback, initial HTML selected option, and UI fallback to `global`. Retain the `front` option and explicit front behavior.

- [ ] **Step 3: Update documentation**

State that port 8001 starts in four-camera mode and three-camera mode defaults to global. Keep explicit front/global examples and all three namespace names.

- [ ] **Step 4: Verify and commit**

Run:

```bash
pytest -q tests/test_selectable_camera_qc.py
bash -n scripts/collection/collect_mobile_pipeline_qc_web.sh scripts/collection/collect_mobile_pipeline_qc_web_four_camera.sh
```

Expected: tests pass and scripts exit 0.

Then:

```bash
git add tests/test_selectable_camera_qc.py scripts/collection/collect_mobile_pipeline_qc_web_four_camera.sh scripts/embodied_data_pipeline-main/scripts/pipeline_web_app.py README.md scripts/embodied_data_pipeline-main/README.md
git commit -m "feat: default three-camera mode to global head"
```

### Task 4: Verify real reports and restart port 8001 safely

**Files:**
- Verify: `scripts/embodied_data_pipeline-main/scripts/pipeline_web_app.py`
- Verify: `/home/caizj/.config/systemd/user/collect-mobile-pipeline-qc-web.service`

**Interfaces:**
- Consumes: implementation, real data under `/home/agilex/data/stage2_new/scene1`, and the user service.
- Produces: active port-8001 service returning matching report paths and 156 records for both existing namespaces.

- [ ] **Step 1: Run fresh verification**

```bash
pytest -q tests/test_selectable_camera_qc.py
python3 -m py_compile scripts/embodied_data_pipeline-main/scripts/pipeline_web_app.py
bash -n scripts/collection/collect_mobile_pipeline_qc_web.sh scripts/collection/collect_mobile_pipeline_qc_web_four_camera.sh
git diff --check
```

Expected: tests, compilation, and syntax checks pass; diff check is empty.

- [ ] **Step 2: Wait until server-owned jobs are inactive**

```bash
curl -fsS http://127.0.0.1:8001/api/jobs | python3 -c '
import json, sys
active = [job for job in json.load(sys.stdin)["jobs"] if job.get("status") in {"queued", "running"}]
print(json.dumps(active, ensure_ascii=False, indent=2))
raise SystemExit(bool(active))
'
```

Expected: `[]` and exit 0. If active, monitor without restarting.

- [ ] **Step 3: Restart and verify the service**

```bash
systemctl --user restart collect-mobile-pipeline-qc-web.service
for attempt in $(seq 1 30); do curl -fsS http://127.0.0.1:8001/ >/dev/null && break; sleep 1; done
systemctl --user is-active collect-mobile-pipeline-qc-web.service
ss -ltnp '( sport = :8001 )'
```

Expected: endpoint succeeds, state is `active`, and one listener exists on `0.0.0.0:8001`.

- [ ] **Step 4: Smoke-test real reports**

POST `/api/status` for raw dataset `/home/agilex/data/stage2_new/scene1/20260715_scene1` with three-camera global and four-camera configurations. Assert matching HDF5/QC namespaces and 156 episode and overview records in each response.

- [ ] **Step 5: Inspect scope**

```bash
git status --short
git log -4 --oneline
```

Expected: only pre-existing unrelated user changes remain; implementation commits are present.

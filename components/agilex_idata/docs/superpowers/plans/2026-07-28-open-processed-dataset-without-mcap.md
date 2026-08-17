# Open Processed Dataset Without MCAP Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Allow the QC web console to discover and open camera-specific HDF5 and QC output after the original MCAP files have been deleted.

**Architecture:** Extend source-directory discovery with a structured `processed_variants` map. The existing browser applies an available processed variant as explicit HDF5/QC/LeRobot paths while leaving MCAP empty, and reapplies the matching paths when the camera mode changes.

**Tech Stack:** Python 3, standard library HTTP server, embedded JavaScript, pytest.

## Global Constraints

- Existing MCAP discovery and conversion behavior must remain unchanged when MCAP files exist.
- A processed-only selection must leave the MCAP input empty.
- Supported namespaces are `four_camera`, `three_camera_front`, and `three_camera_global`.
- Camera switching must never reuse the previous variant's explicit HDF5 or report paths.
- No new runtime dependencies.

---

### Task 1: Discover Processed Camera Variants

**Files:**
- Modify: `scripts/embodied_data_pipeline-main/scripts/pipeline_web_app.py`
- Modify: `tests/test_trajectory_duration_qc_and_status_performance.py`

**Interfaces:**
- Produces: `discover_processed_dataset_variants(dataset_path: Path) -> dict[str, list[dict[str, Any]]]`
- Extends: each `discover_dataset_choices()` item with `has_mcap` and `processed_variants`

- [ ] **Step 1: Write the failing discovery test**

Create a temporary two-level scan root containing:

```text
stage2_new/scene2/four_camera/hdf5_episodes/20260715_scene2/episode0/states/aligned_joints.h5
stage2_new/scene2/four_camera/qc_reports/20260715_scene2_20260716_120632/batch_summary.json
```

Assert the `scene2` choice reports no MCAP, one HDF5 episode, and exact explicit
HDF5/QC/LeRobot paths under `processed_variants["four_camera"]`.

- [ ] **Step 2: Run the test and verify the missing metadata failure**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/agilex/openpi/.venv/bin/python -m pytest -q \
  tests/test_trajectory_duration_qc_and_status_performance.py -k processed_dataset
```

Expected: FAIL because `processed_variants` is absent.

- [ ] **Step 3: Implement bounded processed-output discovery**

Scan only the three known camera namespace directories and direct children of
their `hdf5_episodes` directory. Include a child only when it contains at least
one `episode*` directory. Return:

```python
{
    "four_camera": [{
        "dataset_name": "20260715_scene2",
        "hdf5_root": ".../four_camera/hdf5_episodes/20260715_scene2",
        "qc_root": ".../four_camera/qc_reports",
        "lerobot_root": ".../four_camera/lerobot",
        "episode_count": 1,
    }]
}
```

Use the maximum raw/processed episode count for the existing dropdown count.

- [ ] **Step 4: Run the focused test**

Expected: PASS.

### Task 2: Apply Processed Paths in the Browser

**Files:**
- Modify: `scripts/embodied_data_pipeline-main/scripts/pipeline_web_app.py`
- Modify: `tests/test_trajectory_duration_qc_and_status_performance.py`

**Interfaces:**
- Consumes: source choice `has_mcap` and `processed_variants`
- Produces: `applyProcessedDatasetVariant(choice) -> boolean`

- [ ] **Step 1: Write failing browser contract tests**

Assert the embedded JavaScript:

- clears `mcapPath` for a processed-only choice;
- fills `datasetName`, `hdf5Root`, `qcRoot`, `lerobotRoot`, and `repoId`;
- calls processed path application in both camera-count and head-source change
  handlers;
- can keep the source choice selected by matching an explicit HDF5 path.

- [ ] **Step 2: Run and verify RED**

Run the focused test module and confirm failure is caused by the absent browser
logic.

- [ ] **Step 3: Implement selection and camera switching**

Add a helper that computes the active namespace from `cameraCount` and
`headCameraSource`, prefers a processed dataset matching the current dataset
name, and otherwise chooses the naturally first entry. For processed-only
choices, skip the MCAP picker, apply explicit paths, update the hint, and
refresh status. If a camera variant has no processed data, clear stale explicit
paths and show a specific hint.

- [ ] **Step 4: Run focused and full tests**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/agilex/openpi/.venv/bin/python -m pytest -q tests
```

Expected: all current tests pass.

- [ ] **Step 5: Verify and deploy**

Run Python compilation, embedded JavaScript `node --check`, and
`git diff --check`. Commit only files from this change, restart
`collect-mobile-pipeline-qc-web.service`, then query the live status endpoint
using the explicit four-camera and three-camera-global paths discovered for
`/home/agilex/data/stage2_new/scene2`. Both must return nonzero episodes and
the expected QC report directory.

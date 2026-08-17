# Open Processed Dataset Without MCAP

## Problem

The QC web console scans `/home/agilex/data` as two-level source directories and
assumes a selected directory still contains MCAP data. When the MCAP files have
been deleted but camera-specific HDF5 and QC output remain, the UI passes the
scene directory as `mcap_path`. Path derivation then points at the wrong HDF5
location and reports zero episodes.

For example, selecting `/home/agilex/data/stage2_new/scene2` currently derives:

`/home/agilex/data/stage2_new/four_camera/hdf5_episodes/scene2`

The actual processed data is:

`/home/agilex/data/stage2_new/scene2/four_camera/hdf5_episodes/20260715_scene2`

## Design

`discover_dataset_choices()` will inspect each source directory for processed
camera variants:

- `four_camera`
- `three_camera_front`
- `three_camera_global`

Each discovered HDF5 dataset will expose its dataset name, HDF5 path, QC root,
LeRobot root, and episode count in a `processed_variants` mapping. Existing MCAP
discovery remains unchanged.

When a user selects a source directory with no MCAP but with processed data, the
browser will:

1. Keep the source directory selected.
2. Clear the MCAP input so conversion actions remain unavailable.
3. Fill dataset name, HDF5 root, QC root, LeRobot root, and repo ID from the
   selected camera variant.
4. Reapply those paths when switching between three- and four-camera modes.

If multiple processed dataset names exist in one camera variant, the current
dataset name is preferred; otherwise the naturally first dataset is selected.
Existing source directories that still contain MCAP continue through the
current MCAP picker flow.

## Error Handling

- A missing camera variant does not invent a path. The page clears processed
  paths and shows that the selected variant has no processed HDF5 dataset.
- Filesystem permission and traversal errors are treated as no discovered
  processed dataset, matching existing scan behavior.
- Status, QC, repair, renumber, replay, and LeRobot actions can use the explicit
  HDF5 path. MCAP conversion still requires a non-empty real MCAP input.

## Tests

- Dataset scanning discovers processed variants and counts HDF5 episodes without
  any MCAP files.
- The embedded browser code applies processed paths, clears MCAP, and reapplies
  paths on camera variant changes.
- Existing selectable-camera and QC tests remain green.
- The live service is verified against
  `/home/agilex/data/stage2_new/scene2` for both four-camera and
  three-camera-global modes after restart.

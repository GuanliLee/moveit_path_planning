# Camera Variant Report Path Sync Design

## Goal

Make the selectable camera QC console consistently read and write HDF5 data
and QC reports from the namespace that matches the camera configuration chosen
in the Web UI. The same resolved paths must be used by conversion, QC status,
report display, HDF5 replay, repair, deletion, and LeRobot export.

The port-8001 service continues to start in four-camera mode. When the user
switches to three-camera mode, the default head source is the global camera,
while the existing front-camera option remains available.

## Directory contract

The selected raw MCAP dataset name is preserved beneath the chosen camera
namespace. For a raw dataset such as
`/home/agilex/data/stage2_new/scene1/20260715_scene1`, the mappings are:

| UI configuration | Camera layout | HDF5 path | QC report root |
| --- | --- | --- | --- |
| Four cameras | `four_camera` | `scene1/four_camera/hdf5_episodes/20260715_scene1` | `scene1/four_camera/qc_reports` |
| Three cameras, global head | `three_camera_global` | `scene1/three_camera_global/hdf5_episodes/20260715_scene1` | `scene1/three_camera_global/qc_reports` |
| Three cameras, front head | `three_camera_front` | `scene1/three_camera_front/hdf5_episodes/20260715_scene1` | `scene1/three_camera_front/qc_reports` |

The three-camera selector defaults to `global`. The service-level camera count
default remains four.

## Path resolution

The backend remains the source of truth for resolving effective paths. Every
operation sends the camera count and head source, and all handlers call the
same resolver before accessing HDF5 data or QC reports.

When HDF5 or QC inputs point into one of the recognized generated namespaces
(`four_camera`, `three_camera_global`, or `three_camera_front`), switching the
camera configuration remaps those paths to the selected sibling namespace
while retaining the dataset name. This prevents a stale four-camera path from
overriding a newly selected three-camera configuration, and vice versa.

Paths outside the recognized generated namespace structure remain explicit
manual overrides. This preserves support for users who intentionally inspect
or process data stored elsewhere.

## Dataset selection behavior

The two-level source dataset selector may select a parent such as `scene1`
whose raw MCAP dataset is one level deeper. If discovery finds exactly one
MCAP dataset below that parent, the UI automatically selects it and uses that
child path for derivation. If multiple datasets are found, the existing MCAP
dataset selector remains visible and requires an explicit choice.

Choosing a different source dataset clears generated-path overrides as it does
today. Changing the camera count or head source also refreshes effective paths
and report status immediately.

## Report and replay behavior

Status refresh selects the newest valid `batch_summary.json` under the QC root
for the active namespace and dataset name. The report table and overview are
built only from that matching HDF5 root and report directory.

HDF5 replay uses the same resolved HDF5 root and matching QC report directory.
Other HDF5 operations, including QC, repair, deletion, grade synchronization,
renumbering, and LeRobot conversion, continue to use the same configuration
object so they cannot drift to another camera namespace.

## Validation

Automated tests will cover:

- all three camera configuration-to-namespace mappings;
- global as the default head source for three-camera mode;
- automatic selection of a sole nested MCAP dataset;
- remapping recognized stale four-camera paths to three-camera global paths;
- remapping back to four-camera and to three-camera front paths;
- preservation of genuine custom HDF5 and QC path overrides;
- loading the newest report only from the selected namespace;
- UI camera changes refreshing the matching status paths.

After the focused tests pass, the full relevant test group and shell syntax
checks will run. The existing port-8001 user service will be restarted only
after confirming that no conversion or QC job is still active, then its unit
state, listener, page, and both real camera-mode report responses will be
smoke-tested.

## Compatibility and safety

The non-selectable generic launcher retains its existing behavior. No HDF5,
QC report, MCAP, or LeRobot data is migrated or rewritten by this change.
Existing custom paths remain usable, and service restart must not interrupt an
active conversion or QC process.

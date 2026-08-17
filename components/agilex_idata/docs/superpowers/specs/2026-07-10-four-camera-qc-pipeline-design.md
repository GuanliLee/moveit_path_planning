# Four-Camera Mobile QC Pipeline Design

## Goal

Add an isolated four-camera MCAP conversion and quality-control service for the
mobile ALOHA collector. The existing three-camera service remains available on
port 8001 with its current defaults and data contract. The new service runs on
port 8002 and requires the fourth RGB stream collected from
`/camera_h/color/image_raw`.

The new path reuses the existing conversion, QC, repair, replay, and LeRobot
implementation. It must not become an independent copy of the pipeline core.

## Compatibility Boundary

The existing entry point stays operational:

- entry: `scripts/collection/collect_mobile_pipeline_qc_web.sh`
- default port: `8001`
- camera contract: `left`, `front`, `right`
- profile: `robot_profiles/aloha.yaml`
- conversion topic config: `mcap_conversion/topic_configs/aloha_data_params.yaml`

The new entry point is isolated:

- entry: `scripts/collection/collect_mobile_pipeline_qc_web_four_camera.sh`
- default port: `8002`
- camera contract: `left`, `front`, `right`, `head`
- profile: `robot_profiles/aloha_four_camera.yaml`
- conversion topic config:
  `mcap_conversion/topic_configs/aloha_four_camera_data_params.yaml`
- output namespace: `four_camera`, unless the operator explicitly supplies
  output paths

The 8001 defaults must not change. Shared code may gain explicit parameters,
but omitting them must reproduce the current three-camera behavior.

## Camera Data Contract

The four-camera service preserves all existing mappings and appends one camera:

| Collection name | ROS topic | QC/video key | LeRobot key |
| --- | --- | --- | --- |
| `left` | `/camera_l/color/image_raw` | `hand_left_color` | `observation.images.hand_left_color` |
| `front` | `/camera_f/color/image_raw` | `head_color` | `observation.images.head_color` |
| `right` | `/camera_r/color/image_raw` | `hand_right_color` | `observation.images.hand_right_color` |
| `head` | `/camera_h/color/image_raw` | `head` | `observation.images.head` |

The fourth camera produces these artifacts:

- ALOHA intermediate images and sync index at `camera/color/head`
- intermediate mobile HDF5 dataset `camera/color/head`
- aligned-frame timestamp `timestamp/camera/head`
- ICRA-style video `videos/head.mp4`
- QC camera key `head`
- LeRobot feature `observation.images.head`

`/camera_h/color/camera_info` is intentionally excluded because it contains
untrusted placeholder calibration. The four-camera conversion must not require
that topic and must not create `head` intrinsic or extrinsic datasets.

## Architecture

### Isolated launchers and fixed variant settings

The four-camera shell launcher selects port 8002, the four-camera profile, the
four-camera topic YAML, the four-camera conversion layout, and an independent
output namespace. These settings are fixed by the launcher but remain
overridable through explicit environment variables or page paths where the
existing launcher already allows overrides.

Thin four-camera conversion entry points provide an independently invokable
command while delegating to shared conversion functions. They set four-camera
defaults rather than duplicating the converter implementation.

### Shared parameterized conversion

The current converter is extended with explicit inputs for:

- topic YAML
- camera layout/video source map
- robot profile

The default values remain the three-camera files and mappings. The four-camera
entry point supplies the new values. Both host and Docker conversion command
builders must pass the same variant settings.

The ALOHA-to-aligned-HDF5 adapter reads `camera/color/head` when present and
writes `timestamp/camera/head` for every aligned frame. The three existing
timestamp keys are unchanged.

Video generation uses the selected camera layout. The four-camera layout adds
`head.mp4` sourced from `camera/color/head/sync.txt`; the old layout still
contains exactly three videos.

### QC, repair, replay, and LeRobot

The four-camera profile contains four required cameras. Camera completeness,
sync-repair statistics, frame-count checks, and report summaries therefore
include `head` without separate special-case QC logic.

Repair and replay camera discovery must consume the active profile or episode
metadata instead of assuming only three fixed videos. Existing three-camera
episodes retain their current ordering and labels; four-camera episodes append
`head`.

LeRobot conversion remains profile-driven. The four-camera profile adds
`observation.images.head`, while the existing ALOHA profile remains unchanged.

### Output isolation

When the operator does not supply explicit output paths, the four-camera service
writes derived HDF5, QC, and LeRobot outputs below a `four_camera` namespace.
This prevents simultaneous 8001 and 8002 jobs with the same dataset name from
overwriting each other. Source MCAP discovery continues to scan the existing
data root.

## Data Flow

1. The 8002 Web launcher starts the shared Web application with the
   four-camera variant settings.
2. MCAP conversion reads four image topics using the four-camera topic YAML.
3. Synchronization produces four `camera/color/*/sync.txt` sequences using the
   existing timestamp tolerance and repair rules.
4. Mobile HDF5 contains four color datasets but calibration only for the three
   calibrated cameras.
5. The aligned HDF5 adapter records timestamps for all four logical cameras.
6. Video generation creates the original three MP4 files plus `head.mp4`.
7. QC loads `aloha_four_camera.yaml` and requires all four camera keys.
8. Replay, review, repair, and LeRobot export discover and retain the fourth
   view.

## Failure Handling

The four-camera path fails closed when:

- `/camera_h/color/image_raw` is absent from the input MCAP;
- the synchronized `head` sequence is absent or empty;
- `head.mp4` cannot be produced or its frame count differs from aligned HDF5;
- aligned HDF5 lacks `timestamp/camera/head`;
- QC camera completeness or the existing sync-repair thresholds fail;
- LeRobot output lacks `observation.images.head` when export is requested.

Errors identify the missing camera key and failed stage. A failure in the 8002
service must not stop or mutate a running 8001 service.

## Testing and Acceptance

Implementation follows test-driven development.

Automated acceptance tests cover:

1. Characterization of unchanged 8001 defaults, three-camera mappings, and
   output layout.
2. The new launcher defaults to 8002 and selects only four-camera resources.
3. The four-camera topic YAML contains four RGB topics and only three trusted
   camera-info topics.
4. Four-camera conversion produces four synchronized image streams, four
   aligned timestamp keys, and four MP4 files.
5. The four-camera QC profile rejects a three-camera episode and accepts a
   valid four-camera episode.
6. Repair/replay discovery and LeRobot conversion preserve `head`.
7. No `head` intrinsic or extrinsic dataset is emitted.
8. 8001 and 8002 can run concurrently and both return their Web pages.
9. Host conversion and, where available, Docker command generation select the
   same variant resources.

End-to-end verification uses a controlled four-camera fixture containing the
full robot-state/action schema and four image streams. If live robot topics are
available, a short real capture is also converted. Delivery requires successful
MCAP conversion, four video artifacts, a passing QC report, and a LeRobot dataset
whose metadata contains all four image features.

## Non-Goals

- Changing the current 8001 three-camera contract.
- Treating the untrusted `/camera_h/color/camera_info` as calibration.
- Renaming the existing `front -> head_color` mapping.
- Duplicating the full pipeline source tree.
- Changing motion, action, duration, vision, or grading thresholds unrelated to
  the fourth camera.

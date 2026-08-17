# Four-RGB Mobile Collection Design

## Goal

Extend both mobile web collection entry points from three RGB cameras to four by adding the real image stream `/camera_h/color/image_raw` as the `head` camera. The fourth stream must be synchronized with the existing robot and camera data, must be checked for severe frame loss, and must be preserved through MCAP, ALOHA intermediate data, HDF5, QC, LeRobot conversion, and web visualization.

## Scope

The fixed-stage and staged entry points share the same collection controller and `aloha_mobile` data profile. The change therefore updates both entry points and the shared components that define or validate the persisted format:

- `scripts/collection/collect_mobile_pipeline_web_fixed_stage.sh`
- `scripts/collection/collect_mobile_pipeline_web_staged.sh`
- the standard `aloha_mobile` ROS data profile
- the mobile capture launch and native compressed-image transport
- required-topic checks
- HDF5 conversion and QC
- HDF5/LeRobot visualization ordering

Existing `left`, `front`, and `right` camera keys remain unchanged.

## Data Contract

The ordered RGB cameras are:

1. `left` from `/camera_l/color/image_raw`
2. `front` from `/camera_f/color/image_raw`
3. `right` from `/camera_r/color/image_raw`
4. `head` from `/camera_h/color/image_raw`

The ALOHA intermediate directory for the new stream is `camera/color/head`. The HDF5 dataset is `camera/color/head`. LeRobot conversion exposes the stream as `observation.images.head` through the existing dynamic camera conversion.

`/camera_h/color/camera_info` is not a trustworthy calibration source for this driver-free USB camera. It must not be recorded as head calibration. The HDF5 output must not contain fabricated `camera/colorIntrinsic/head` or `camera/colorExtrinsic/head` datasets. Existing calibration handling for the original three cameras is unchanged.

## Capture and Synchronization

All four deployed camera drivers already publish reliable `/color/image_raw/compressed` streams. The mobile launch therefore starts only the recorder and does not start redundant compressor nodes. The recorder subscribes to `/camera_h/color/image_raw/compressed` while retaining `/camera_h/color/image_raw` as the MCAP topic name, matching the existing three-camera behavior. Keeping one publisher per compressed topic prevents duplicate frames and timestamp jitter.

The standard `aloha_mobile` YAML adds `head` to the RGB names and raw-image topic lists. Its `configTopics` list remains limited to the three trustworthy camera-info topics. The Web controller passes this exact YAML path to capture, MCAP conversion, and the synchronization launch instead of relying on a possibly stale installed default. The synchronization tool reads all four RGB names from this profile and creates a `sync.txt` for each stream against the common episode time axis. HDF5 conversion reads the same profile, so all four datasets use the synchronized frame selection.

Both web entry points explicitly identify the four-RGB contract in their runtime configuration and help text. The fixed-stage wrapper continues to delegate to the staged entry point, so both modes use the same capture and conversion profile.

## Frame-Loss and Failure Handling

The pre-recording topic check requires non-empty raw and native-compressed images from all four cameras. A missing `/camera_h/color/image_raw` or `/camera_h/color/image_raw/compressed` stream blocks recording when required-topic checks are enabled.

The recorder's per-topic frequency monitor applies the configured minimum rate to head as part of the shared camera list. It latches runtime rate failures and writes `capture_health.json` at the end of each episode. That sidecar compares every camera's raw MCAP message count with the full recording duration and requires at least 80% of the configured minimum-rate coverage. This catches a camera that starts late or dies midway even if synchronization later truncates all streams to an equal shorter interval.

HDF5 QC requires `capture_health.json` for new mobile episodes, consumes its full-duration result, requires all four RGB datasets, compares each camera frame count with the synchronized robot frame count, and enforces the configured minimum frame count. A missing health sidecar, capture coverage failure, missing stream, unreadable sample, or mismatched frame count rejects the episode. Effective synchronized FPS remains subject to the existing mobile minimum threshold (default 20 FPS). Explicit `ALLOW_MISSING_CAPTURE_HEALTH=1` is available only for converting legacy episodes that predate the sidecar.

No frame duplication or synthetic padding is introduced. If head cannot supply a synchronized frame within the configured time-difference limit, the resulting shortfall is reported rather than hidden.

## Compatibility

Previously collected three-camera HDF5 files remain readable. New collection runs use the updated four-camera `aloha_mobile` contract and will fail the new collection QC if head is absent. Replay code orders cameras as `left`, `front`, `right`, `head`; dynamically discovered nonstandard cameras remain supported after those preferred names.

## Testing

Automated regression tests will first fail against the current three-camera implementation, then cover:

- the standard mobile YAML's four RGB names/topics and three trustworthy camera-info topics
- absence of redundant compressor nodes in the mobile capture launch
- fixed-stage and staged entry-point four-camera configuration
- required-topic checking for the real head image
- HDF5 QC rejection when `head` is missing and acceptance when four synchronized RGB datasets are present
- recorder coverage rejection when head disappears during an episode, even if synchronized HDF5 lengths are equal
- absence of fabricated head intrinsic/extrinsic datasets when no trustworthy config exists
- visualization discovery/order for the head stream
- Bash syntax and embedded Python compilation

Final verification includes the focused regression tests, the relevant existing test suite, Python compilation, Bash syntax checks, and a ROS workspace build when the local ROS dependencies are available.

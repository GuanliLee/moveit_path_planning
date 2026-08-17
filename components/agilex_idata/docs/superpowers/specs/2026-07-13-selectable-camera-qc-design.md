# Selectable Three/Four-Camera QC Design

## Goal

Extend `collect_mobile_pipeline_qc_web_four_camera.sh` so one QC/conversion console can save either three or four RGB streams, while keeping the generic QC launcher backward compatible. Change the specialized console's default port from `8002` to `8012`.

## Camera contract

The four-camera topic configuration already records four raw streams. This feature treats `/camera_f` as the normal head camera and `/camera_h` as the wide-angle/global camera.

| Mode | Raw video | LeRobot feature |
| --- | --- | --- |
| Four cameras | `head_color.mp4` (`/camera_f`) | `observation.images.hand_head_color` |
| Four cameras | `head.mp4` (`/camera_h`) | `observation.images.global_color` |
| Four cameras | `hand_left_color.mp4` | `observation.images.hand_left_color` |
| Four cameras | `hand_right_color.mp4` | `observation.images.hand_right_color` |
| Three cameras, normal head | `head_color.mp4` (`/camera_f`) | `observation.images.hand_head_color` |
| Three cameras, wide-angle head | `head.mp4` (`/camera_h`) | `observation.images.hand_head_color` |
| Either three-camera mode | left/right videos | Existing left/right feature names |

In three-camera mode the unselected head stream is neither required nor exported. In four-camera mode all four videos and their timestamps are required.

## User interface and startup interface

The specialized page exposes two selectors:

- Camera count: `4` (default) or `3`.
- Three-camera head source: `front` (`/camera_f`, default) or `global` (`/camera_h`). The second selector is disabled in four-camera mode.

The launcher also accepts `--camera-count 3|4` and `--head-camera front|global`. These values establish the initial page selection; page selections are sent with every operation so conversion, QC, repair, and LeRobot export use one consistent contract. The generic `collect_mobile_pipeline_qc_web.sh` page hides these controls unless `PIPELINE_CAMERA_VARIANT_SELECTABLE=1` is set.

Each selection uses an isolated default output namespace: `four_camera`, `three_camera_front`, or `three_camera_global`. An explicitly supplied output namespace remains authoritative.

## Implementation

Add strict `three_camera_front` and `three_camera_global` layouts while retaining the existing permissive `three_camera` layout for compatibility. The selectable page resolves camera count and head source to one of these layouts.

The four-camera profile becomes the canonical source of output naming. Four-camera mode uses it directly. Three-camera mode writes a deterministic derived profile under the web log root: it filters out the unselected raw camera, maps the selected head camera to `observation.images.hand_head_color`, updates required-camera and annotation references, and keeps the existing robot/state/action settings. Docker conversion mounts a derived profile explicitly instead of assuming every profile lives in `robot_profiles/`.

## Validation and compatibility

Invalid camera counts, head-source names, or camera layouts fail before starting work. Tests cover launcher defaults and arguments, layout source/required-artifact contracts, derived profile mappings, Web configuration derivation, UI controls, Docker profile mounting, and command-line parser acceptance. Existing non-selectable three-camera behavior remains unchanged.

## Considered alternatives

- Separate launchers and profiles for all three combinations would be simple at runtime but would duplicate most shell, YAML, and UI behavior.
- A command-line-only switch would require restarting the service whenever the operator changes mode.
- A single selectable service with small derived profiles keeps one source of truth and supports both startup defaults and per-session Web selection, so it is the selected approach.

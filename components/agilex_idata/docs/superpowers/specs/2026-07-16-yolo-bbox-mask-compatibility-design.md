# YOLO No-Mask BBox Compatibility Design

## Problem

The production YOLO API now omits all mask fields by default. The four-camera Web
gateway already uses the default request, but it rejects an entire successful
`/predict` response whenever `mask_polygon_xy` is absent. All four camera results
therefore remain stale even though camera input, YOLO health, bbox detections, and
the returned overlay are valid.

## Selected approach

Keep the existing multi-target, temporal, and overlay request behavior. Do not
send the optional `mask` flag. Validate each detection by its bbox geometry and
do not inspect optional mask fields:

- `bbox_xyxy` must contain four finite numbers with positive width and height.
- Missing or invalid bbox geometry remains a protocol error.
- Missing, empty, or malformed optional mask data does not affect the result.

The gateway will retain the YOLO service's detection count, class counts, and
overlay JPEG. It will not attempt to regenerate or rewrite the server overlay.
This restores the previous box-based output and decouples the gateway from the
server's optional mask schema.

## Alternatives not selected

1. Opt into masks and accept bbox-only companion detections. The user explicitly
   chose the previous no-mask output.
2. Require the YOLO server to restore mask fields by default. This is a larger
   cross-service change and unnecessary for a box-based client.

## Code and data flow

Only `YoloHttpClient.predict` response validation changes. Camera capture,
scheduling, HTTP endpoints, target selection, and rendering remain unchanged:

1. Send the existing `/predict` request without a `mask` field.
2. Receive `/predict` JSON.
3. Validate image dimensions, detection count, and each detection's bbox.
4. Ignore any optional mask fields.
5. Validate and publish the server overlay and result metadata as before.

## Tests

Assert that gateway prediction requests omit `mask`. Make the default regression
response a bbox-only detection with no mask keys; prediction must succeed. Add a
response containing malformed optional mask data and verify it is ignored.
Continue rejecting missing or invalid bbox geometry.
Run the focused YOLO HTTP tests, the four-camera core/Web tests, then restart port
7788 and verify that all four result sequences advance, `result_stale` becomes
false, and default no-mask requests succeed against the production YOLO service.

## Success criteria

- Default bbox-only detections no longer freeze the monitoring result.
- Optional mask data cannot affect the gateway result.
- Missing or malformed bbox geometry is still rejected.
- Port 7788 reports fresh results for all enabled online cameras after restart.

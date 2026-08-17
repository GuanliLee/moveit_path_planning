# Four-Camera Live YOLO Web Console Design

## Goal

Add a standalone Web console that continuously sends the latest frames from the
mobile ALOHA robot's four RGB cameras to the remote product YOLO service. The
console listens on port 7788 and shows the full YOLO mask overlay for every
camera, with a control that switches the card to the exact source image used for
that inference result.

The operator builds one global multi-target selection shared by all four
cameras. Each camera can be enabled independently. One globally adjustable
interval defaults to 3 seconds; every enabled camera starts one request at each
interval, for at most four requests per round. The workers never create an
unbounded frame or request queue.

## Entry Points and Defaults

The feature adds two entry points:

- `scripts/inference/four_camera_yolo_web.py`: ROS subscriptions, inference
  scheduling, state management, and HTTP Web application.
- `scripts/inference/run_four_camera_yolo_web.sh`: sources ROS environments,
  applies robot defaults, checks the port, and launches the Python application.

Defaults are:

- Web bind address: `0.0.0.0`
- Web port: `7788`
- ROS domain: `99`
- YOLO base URL: `http://192.168.4.121:7881`
- Global inference interval: `3.0 seconds`
- Allowed interval range: `1.0` through `60.0 seconds`
- Target prompt: empty, so inference is initially idle

The host, port, ROS domain, YOLO URL, and default interval are command-line
options and may also be supplied by the launcher through environment variables.
If port 7788 is occupied, startup fails with a clear error and does not terminate
or replace the existing listener.

## Camera Contract

The application subscribes to the four existing compressed-image topics:

| Camera key | Label | ROS topic |
| --- | --- | --- |
| `left` | Left / 左视角 | `/camera_l/color/image_raw/compressed` |
| `front` | Front / 前视角 | `/camera_f/color/image_raw/compressed` |
| `right` | Right / 右视角 | `/camera_r/color/image_raw/compressed` |
| `head` | Head / 头部视角 | `/camera_h/color/image_raw/compressed` |

Subscriptions use sensor-data QoS with a queue depth of one. A callback replaces
the previous JPEG for that camera; it never appends camera frames to an
application-level queue.

## Architecture

### Camera hub

`CameraHub` owns the ROS node and subscriptions. For each camera it records the
latest JPEG bytes, ROS timestamp, monotonic receipt time, and input sequence.
Access is protected by a small lock, and consumers copy a frame reference
without holding that lock during network I/O.

Camera liveness is derived from the age of the most recent ROS message. An
unseen or stale topic is reported independently and does not block the other
three cameras.

### Global scheduler and per-camera YOLO workers

A global scheduler wakes once per configured interval and offers the same round
to all four camera workers. Each camera has an enabled flag and independent
request state. At the start of a round, an idle enabled worker snapshots its
current latest frame, configuration generation, and global target list. The four
eligible requests run concurrently. At most one request per camera may be in
flight; if a camera's previous request is still running, that camera skips the
new round. Frames received during a request replace the camera cache and are
candidates for the next round; they are never queued behind the current request.

Workers call `POST /predict` with:

```json
{
  "request_id": "four-camera-left-<unique-id>",
  "image_base64": "<jpeg bytes>",
  "targets": ["Wanglaoji", "Sprite"],
  "return_overlay": true
}
```

The HTTP client bypasses process proxy variables explicitly because the robot's
current proxy configuration does not exempt `192.168.4.121`. Each worker may
reuse its own connection when the remote service permits keep-alive and must
reconnect after any transport failure.

On success the worker validates `ok`, `overlay_jpeg_base64`, image dimensions,
the detection list, and every returned mask polygon. It then atomically
publishes a result containing:

- the exact source JPEG sent in that request;
- the decoded overlay JPEG;
- source frame sequence and timestamp;
- detection count and per-class counts;
- remote service latency and measured round-trip latency;
- result sequence and completion time.

Publishing the source and overlay atomically ensures switching between them
always shows the same inference frame. Topic liveness still reflects the newest
camera frame, which may be newer than the displayed inference result.

### Shared application state

The browser presents selected targets as removable tags. Enter and comma add a
typed target; the backend also accepts comma- or newline-separated text, trims
it, removes empty items, and deduplicates names while preserving order. The
resulting target tuple is sent identically to all enabled workers. An empty
target list pauses inference without stopping camera subscriptions.

The application queries `/health` and `/classes` at startup and periodically
refreshes service health. Official class names are exposed to the page for
suggestions. Names not listed as deployed classes produce a visible warning but
remain allowed so the server can resolve supported aliases.

Configuration changes are atomic. The global interval is validated in the
closed range from 1 through 60 seconds. Applying a new target list or interval
advances a config generation; results from an older generation may finish but
are discarded rather than displayed under the new configuration.

### Web server

The application uses the repository's existing standard-library pattern based
on `ThreadingHTTPServer`. It does not add FastAPI, Uvicorn, or browser WebSocket
dependencies.

The HTTP surface is:

| Method and path | Purpose |
| --- | --- |
| `GET /` | Four-camera console HTML |
| `GET /api/status` | Service, camera, configuration, inference, and error state |
| `GET /api/classes` | Cached deployed YOLO class metadata |
| `POST /api/config` | Apply targets, global interval, pause state, and per-camera enabled flags |
| `GET /api/cameras/<key>/source.jpg?seq=N` | Exact source frame for the latest inference result |
| `GET /api/cameras/<key>/overlay.jpg?seq=N` | Matching latest YOLO overlay |
| `GET /api/cameras/<key>/latest.jpg?seq=N` | Latest ROS frame before the first inference result or for diagnostics |

Image responses use no-store caching and return `404` until the requested image
exists. Invalid JSON, unknown camera keys, empty or out-of-range numeric values,
and malformed targets return `400` with a structured error. Unexpected backend
failures return `500` without exposing a traceback to the browser.

The browser polls `/api/status` twice per second. It only refreshes an image URL
when that camera's sequence changes, preventing repeated transfer of unchanged
JPEGs and avoiding base64 image data in status responses.

## User Interface

The selected layout is a two-by-two grid of camera cards. Each card uses the
YOLO mask overlay as its large primary image and provides a local `Mask / Source`
view toggle. Switching views does not change inference configuration or start a
new request.

The top control bar contains:

- remote YOLO health and last health-check time;
- one searchable tag input with deployed-class suggestions, Enter/comma tag
  creation, alias entry, and individual tag removal;
- one global interval input, defaulting to 3 seconds and accepting 1 through 60;
- `Apply and start` and `Pause all` actions;
- the latest configuration or service error.

Draft target or interval edits do not affect the workers until the operator
selects `Apply and start`.

Each camera card contains:

- bilingual camera label and ROS topic state;
- one primary image with Mask/Source switching and explicit empty/loading states;
- an individual enabled switch;
- configured interval and measured result rate;
- remote inference latency and total round-trip latency;
- result age, source-frame timestamp, detection total, and per-class counts;
- the most recent camera-specific error.

When no prompt has been applied, the result area says `Waiting for target
prompt`. When a topic has no image it says `Waiting for camera`. A failed YOLO
request keeps the last successful pair visible but marks it stale and shows its
age, so an old overlay cannot be mistaken for a current result.

The page is usable on a desktop browser and collapses to one card per row on a
narrow display. The primary acceptance target is a desktop on the robot's local
network.

## Scheduling and Performance

At the default interval, four enabled cameras produce at most four concurrent
requests every 3 seconds. A round never enqueues work behind an already running
camera request, so a slow service reduces that camera's effective result rate
instead of building a backlog.

The status page reports the configured global interval and each camera's
measured result rate separately. The UI does not promise full input-frame
processing: the camera topics may publish at 25 to 30 FPS while the inference
console intentionally samples only the newest frame at each global round.

If the YOLO service becomes slower than the configured interval, each camera
continues with one in-flight request and then samples the newest available frame.
No cross-camera global lock serializes requests; a failure or slow response for
one view does not freeze the other views.

## Failure Handling and Shutdown

- If `/health` or `/classes` is unavailable, the page and ROS subscriptions
  remain operational, inference reports offline, and health checks retry.
- Network timeouts, HTTP errors, invalid JSON, missing overlays, and invalid
  image payloads are recorded per camera with timestamps and retry on the next
  scheduled interval.
- A camera topic that disappears is marked stale while the other camera workers
  continue.
- Applying an empty target list pauses all YOLO requests but retains the latest
  images and results with stale-state labels.
- Opening or closing a browser does not control backend subscriptions or worker
  lifetime. Multiple browsers observe and edit the same global configuration.
- SIGINT and SIGTERM stop the HTTP server, worker threads, and ROS executor and
  destroy the node without leaving a listener on port 7788.

## Testing and Acceptance

Implementation follows test-driven development.

Unit tests cover:

1. Global prompt splitting, trimming, empty-item removal, and stable
   deduplication.
2. Interval validation at the `1.0` and `60.0` boundaries and rejection outside
   the range.
3. Latest-frame replacement and absence of application-level frame queues.
4. Concurrent global rounds, one-in-flight skipping, and stale config generation
   rejection.
5. YOLO health, class, success, mask, overlay, HTTP-error, timeout, and malformed
   response handling.
6. Atomic matching of source and overlay sequences.
7. Independent error and liveness state for all four camera keys.

HTTP tests cover:

1. Home page and the four camera cards.
2. Status and cached-class JSON schemas.
3. Valid target, interval, pause, and camera configuration changes plus all
   invalid configuration cases.
4. Source, overlay, and latest image responses, including no-store headers and
   missing-image `404` behavior.
5. Unknown routes and unknown camera keys.

Integration tests use fake camera frames and a fake YOLO server to prove that
four workers start in the same round, process only the latest frames, skip a busy
camera without queuing, keep image pairs matched, and isolate one camera's
failures. They do not require ROS hardware or the remote GPU service.

Live acceptance on the robot requires:

1. Port 7788 is free and the launcher starts successfully.
2. All four compressed-image topics become live in the page.
3. A multi-target tag selection produces a mask overlay for each camera that
   currently sees any selected product, with a working source-image toggle.
4. Every successful detection response contains mask polygons, and the decoded
   overlay matches the corresponding source dimensions.
5. Changing prompts discards late results from the previous prompt.
6. Changing the global interval to 3 seconds produces no more than one request
   per enabled camera per round, while individual camera switches do not stop the
   other cards.
7. Pause and resume do not stop ROS subscriptions, and stopping the process
   releases port 7788 cleanly.

Delivery verification includes Python compilation, shell syntax checking, the
targeted automated test suite, a local fake-service browser smoke test, and the
live four-camera/remote-YOLO check when the hardware and service are available.

## Non-Goals

- Processing every 25 to 30 FPS input frame without dropping frames.
- Recording images or detections to a dataset.
- Controlling robot motion or changing the collection pipeline.
- Adding user authentication, TLS, or Internet-facing deployment.
- Returning per-instance lossless binary-mask PNG files; the page displays the
  service's composite JPEG overlay and validates the returned polygon data.
- Maintaining independent prompts per camera.
- Persisting Web configuration across application restarts.

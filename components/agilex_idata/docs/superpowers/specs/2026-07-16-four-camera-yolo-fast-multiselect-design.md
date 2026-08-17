# Four-Camera YOLO Fast Multi-Select Design

## Goal

Make the four-camera YOLO console easier to operate by replacing free-form
target entry with a deployed-class multi-select dropdown, running inference as
often as every 0.2 seconds, and enabling the remote service's temporal mode so
small products such as Yakult produce masks.

## Scope

- Support selecting multiple deployed product classes from a searchable
  checkbox dropdown.
- Keep selected classes visible as removable chips.
- Default the inference interval to 0.2 seconds and accept values from 0.2 to
  60 seconds.
- Poll status and refresh result images every 200 milliseconds.
- Send a stable, camera-specific `stream_id` with temporal inference enabled.
- Reset temporal state on the first request and whenever the selected target
  set changes.
- Preserve the existing four-camera, one-request-in-flight-per-camera policy.

Free-form aliases, new server endpoints, WebSockets, camera driver changes,
and model retraining are outside this change.

## User Experience

The target control is a button-like field showing the selection count. Opening
it reveals a searchable list populated from `/api/classes`. Each deployed class
has a checkbox; checking or unchecking updates the selected chips immediately.
The menu includes a clear-all action and closes on Escape or an outside click.

Only class names advertised by the deployed YOLO service are selectable. The
existing Apply and Pause actions remain explicit so a user can prepare several
changes before starting a new inference configuration.

The interval input defaults to `0.2`, has a minimum of `0.2`, a maximum of
`60`, and a step of `0.1` seconds. Status text and per-camera metrics show the
configured decimal interval without rounding it to an integer.

## Data Flow

1. The page loads class metadata from `/api/classes` and builds the dropdown.
2. Apply sends the selected class names, interval, pause state, and enabled
   cameras to `/api/config`.
3. The scheduler offers a round every configured interval. A camera whose
   previous request is still running is skipped for that round, so requests do
   not queue.
4. Each camera request keeps its unique `request_id`, adds
   `stream_id: "four-camera-<camera>"`, and sends `temporal: true`.
5. The first request for a client, and the first request after targets change,
   sends `reset_temporal: true`; later requests send `false`.
6. The page polls `/api/status` every 200 milliseconds. A new result sequence
   changes the cache-busting image URL and loads the paired overlay or source
   JPEG.

## Failure Handling and Capacity

The existing no-overlap guard is the backpressure mechanism. If inference takes
longer than 0.2 seconds, the next offer for that camera is skipped rather than
queued. Other cameras continue independently. Existing timeout, stale-result,
health, and per-camera error displays remain in force.

The dropdown shows a loading state until classes arrive and an empty/error
state if the service provides no selectable classes. Applying with no selected
target remains invalid. The backend is the source of truth for interval
validation, with the browser providing matching validation text.

## Testing

- Core validation accepts 0.2 seconds and rejects values below 0.2.
- Scheduler tests retain the one-in-flight guarantee at short intervals.
- HTTP client tests verify stable per-camera `stream_id`, temporal mode,
  unique request IDs, and reset behavior across target changes.
- Web tests verify the searchable multi-select structure, removal/clear
  controls, 0.2-second input bounds, 200 ms polling, and API payload shape.
- Launcher and CLI tests verify the 0.2-second default.
- The complete four-camera YOLO test suite must pass before restarting the live
  process.

## Acceptance Criteria

- A user can choose multiple advertised classes without typing their names.
- Applying `Yakult` produces temporal-mode requests and visible masks when the
  deployed model returns detections.
- Four enabled cameras are offered inference rounds every 0.2 seconds with no
  per-camera request backlog.
- New results can appear in the browser within the next 200 ms polling cycle.
- Pause, per-camera enable switches, Mask/source toggles, status metrics, and
  error reporting continue to work.

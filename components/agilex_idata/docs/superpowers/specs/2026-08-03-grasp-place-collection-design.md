# Grasp and Place Two-Phase Collection Design

## Goal

Add a new collection launcher that records one logical episode as two separately
stored and independently reviewed captures:

1. grasp capture in the operator-selected stage-2 configuration directory;
2. place capture in a matching configuration directory below
   `/home/agilex/data/place`.

The existing fixed-stage and staged launchers must retain their current behavior.

## Directory and Episode Mapping

The first-phase dataset directory remains the directory passed to the launcher.
Its final path component is the configuration name. For example:

```text
first phase:  /home/agilex/data/stage2_twohand/20260803_scene1/episode7
second phase: /home/agilex/data/place/20260803_scene1/episode7
```

One place directory is created per configuration, not per episode. Both captures
use the same episode number. The logical episode advances from `N` to `N+1` only
after both captures have been saved and independently reviewed.

The place root defaults to `/home/agilex/data/place` and may be overridden with
an environment variable for testing or deployment.

## Entry Point and Compatibility

Create `scripts/collection/collect_mobile_pipeline_web_grasp_place.sh`. It is a
small wrapper around `collect_mobile_pipeline_web_staged.sh`, following the
existing fixed-stage wrapper pattern.

The wrapper:

- enables the new two-phase bridge mode;
- keeps ordinary, non-staged capture for each physical recording;
- configures the four ROS `std_msgs/msg/Bool` control topics;
- derives the place dataset directory from the first-phase configuration name;
- forwards the existing dataset directory, start index, and grade arguments;
- requires the exact first-phase configuration directory through the existing
  dataset-directory argument or `DATA_DIR`; it does not invent a scene
  configuration name.

The existing launchers do not set the two-phase switch, so their control flow
does not change.

## State Machine and Data Flow

The two-phase bridge uses these states:

```text
waiting_grasp_start
  -- /state_machine/start=true --> recording_grasp
recording_grasp
  -- /state_machine/grasping/end=true --> reviewing_grasp
reviewing_grasp
  -- accepted --> waiting_place_start
  -- discarded --> waiting_grasp_start
waiting_place_start
  -- /state_place/start=true --> recording_place
recording_place
  -- /state_machine/end=true --> reviewing_place
reviewing_place
  -- accepted --> configure grasp episode N+1, waiting_grasp_start
  -- discarded --> waiting_place_start for episode N
```

For both physical recordings, the bridge starts the collection service, waits
until `/status` confirms `capture_running=true` and `state=recording`, and only
then publishes `/data_collection/start=true`. It publishes `false` while idle,
saving, reviewing, switching directories, or handling an error.

The first capture stops and saves only on
`/state_machine/grasping/end=true`. After the operator selects A, B, or F, the
bridge reconfigures the existing collection service to the paired place path and
episode number. It then waits for `/state_place/start=true`.

The second capture stops and saves only on `/state_machine/end=true`. After its
independent A, B, or F review, the bridge reconfigures the service back to the
first-phase directory at episode `N+1`.

## Review and Discard Semantics

A, B, and F are all completed reviews and allow the flow to continue. The
existing explicit discard action is different:

- discarding the grasp capture deletes that capture, keeps episode `N`, and
  waits for a new grasp start;
- discarding the place capture leaves the already accepted grasp capture in
  place, keeps episode `N`, and waits for a new place start.

This prevents phase directories from drifting to different episode numbers.

## Event Ordering and Failure Handling

Only the topic valid for the current state has an effect. Duplicate, stale, or
out-of-order true events are ignored and recorded in the bridge status/log.
False events do not trigger transitions.

Directory switches are performed only while capture is stopped and no quality
review is pending. A failed start, save, review wait, or configuration switch
sets an error status, publishes collection-ready false, and leaves enough phase
and episode information in the control file for diagnosis. The process remains
alive so an operator can correct the underlying condition or use the manual UI.

## Implementation Shape

The ROS bridge embedded in `collect_mobile_pipeline_web_staged.sh` gains an
opt-in two-phase branch and configuration fields. Existing HTTP helpers and the
collection controller remain the source of truth for recording, saving,
conversion, QC, and review.

Small pure helpers will cover:

- deriving `/home/agilex/data/place/<configuration>`;
- constructing a collection reconfiguration payload with an explicit directory
  and episode number;
- classifying a completed review as accepted or discarded;
- deciding whether a topic event is valid for the current two-phase state.

Keeping these decisions pure makes them testable without ROS hardware.

## Testing

Automated tests will verify:

- the new wrapper exports the required opt-in mode and topics while forwarding
  existing CLI arguments;
- place directory derivation uses the final first-phase path component and
  creates no per-episode directory;
- both phases use the same episode number;
- accepted grasp review advances to place, accepted place review advances the
  logical episode, and discards retry only the discarded phase;
- out-of-order topic events do not change phase;
- legacy wrapper environment and behavior remain unchanged.

Shell syntax checks and focused Python tests will run before completion. A live
ROS/hardware recording is outside automated verification and should be exercised
with a short disposable configuration before production collection.

# Data Collection Save Success Topic Design

## Goal

Add a ROS 2 Boolean status topic to
`collect_mobile_pipeline_web_staged.sh`:

- Topic: `/data_collection/save_success`
- Type: `std_msgs/msg/Bool`
- `true`: the episode data has been saved, its A/B/F quality metadata has been
  written successfully, and the collection controller has advanced to the next
  episode.
- `false`: no episode has reached that success boundary in the current
  collection cycle.

HDF5 conversion, QC, and LeRobot conversion run outside this success boundary.

## Configuration

Expose `DATA_COLLECTION_SAVE_SUCCESS_TOPIC`, defaulting to
`/data_collection/save_success`. Document and print the resolved value alongside
the existing state-machine topic settings.

## Publisher and QoS

The existing `StateMachineBridge` owns the publisher because it already maps
collection-controller state to ROS feedback topics. The new publisher uses
`std_msgs/msg/Bool` with reliable, transient-local QoS and depth 1. A late
subscriber therefore receives the current retained state.

The bridge provides a dedicated publishing helper so save-success state is not
confused with `/data_collection/start`, whose Boolean value describes recording
readiness.

## State Transitions

The retained state starts as `false`.

Publish and retain `false` when:

- the bridge starts;
- a new collection attempt starts, including a start initiated from the web UI;
- an episode is stopping or waiting for quality review;
- the pending episode is discarded;
- start, save, status polling, or review processing fails;
- automatic collection is disabled or the bridge shuts down.

Publish and retain `true` only when collection status confirms all of the
following:

- capture is no longer running;
- quality review is no longer pending;
- `last_saved_episode` identifies the reviewed episode;
- the current episode has advanced beyond `last_saved_episode`.

The bridge tracks whether a real capture/review cycle has armed the success
signal. The first successfully read status establishes a baseline without
announcing an old success, including when the initial status request failed.
Each newly observed capture/review cycle resets the topic to `false`, arms one
future success, and then disarms after publishing `true`. This also permits a
deliberately reused episode number to produce a new success.

This status-based transition supports both state-machine-triggered saves and
web-UI-triggered saves.

The collection controller advances the episode, clears the pending review, and
writes status before scheduling HDF5 conversion or QC. The bridge can therefore
publish `true` independently of synchronous processing or a saturated
background-processing queue.

## Failure and Discard Behavior

A metadata write failure leaves quality review pending, so no success is
published. A discard clears the pending review without advancing the episode
and therefore also remains `false`. Background conversion or QC failures do not
change a previously published `true`, because those jobs are explicitly outside
the success definition.

## Verification

Add regression tests that assert:

- the configurable topic and default value are present;
- the bridge creates a reliable, transient-local Boolean publisher;
- initial, new-cycle, discard/error, disable, and shutdown paths set `false`;
- only the reviewed-and-advanced episode condition sets `true`;
- a pre-existing episode is not announced after startup or status recovery;
- a reused episode number can be announced after a new capture cycle;
- review completion is visible before conversion/QC processing starts.

Run the focused pytest file, the full project test suite, and `bash -n` on the
modified shell script.

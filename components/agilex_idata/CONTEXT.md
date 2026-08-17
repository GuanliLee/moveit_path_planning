# Data Collection

This context records robot demonstrations while preserving the existing raw episode contract for later quality processing and export.

## Language

**Raw Episode**:
One existing `episodeN` directory containing the recorded MCAP, rosbag metadata, capture health, and reviewed episode info sidecar.
_Avoid_: ALOHA episode, converted episode

**Capture Health**:
Online evidence that required sensors and robot topics were present, timely, finite, and sufficiently frequent before and during recording.
_Avoid_: QC report, dataset quality

**Collection Review**:
The operator's A/B/F decision and reasons recorded in the raw episode immediately after capture.
_Avoid_: Automated QC, offline quality pipeline

**Collection Mode**:
The workflow that controls when one or more Raw Episodes start and finish, such as single segment, fixed stage 2, five stage, or paired grasp/place.
_Avoid_: Robot profile, arm mode

**Collection Portal**:
The pre-session web page where an operator enters a raw data directory and starting episode, then launches exactly one Collection Mode. Returning to the portal changes only the browser view; it does not stop the active mode.
_Avoid_: Collection console, recorder

**Collection Console**:
The full-page operator interface of the active Collection Mode. Its controls continue to call the existing collection backend and state machine.
_Avoid_: Collection portal, replacement backend

**Robot Profile**:
The required sensors and robot capabilities for mobile bimanual or stationary bimanual collection.
_Avoid_: Collection mode

**Target Mode**:
Whether a task names one target or two targets while the robot continues to record both arms.
_Avoid_: Single-arm mode, arm mode

**Manual Collection**:
Operator- or state-machine-controlled recording run by the manual operation bridge against the deployed recorder service.

**Inference Collection**:
Inference-triggered recording whose operation bridge, runtime state, and logs are independent from Manual Collection. It deliberately uses the same deployed recorder service and ROS topic/state-machine contract, so the modes are mutually exclusive rather than assigned renamed ROS namespaces.

**Robot Control Lease**:
Exclusive ownership that prevents Manual Collection, Inference Collection, and real-robot replay from controlling or recording the shared robot simultaneously.

**Dataset QC**:
Offline conversion, validation, repair, grading synchronization, and LeRobot export performed outside the collection context.
_Avoid_: Capture health, collection review

**Replay Episode**:
The original Raw Episode number selected by the operator together with its source dataset directory. MCAP replay keeps the last-saved directory and number as one identity when a paired workflow switches its next capture directory. HDF5 replay resolves the number to the exact `episodeN/states/aligned_joints.h5`; graded LeRobot replay resolves it through `source_episode_name` rather than treating it as the grade-local parquet index.
_Avoid_: LeRobot episode index, sorted-list position

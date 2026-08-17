# Stage2 Action Periodic-Jitter Repair

## Goal and scope

Repair high-confidence periodic action corruption in every existing
`three_camera_global` ALOHA HDF5 dataset under
`/home/agilex/data/stage2_new/scene1` through `scene11`, then synchronize any
existing LeRobot derivatives. The recursive scope includes the nested
`scene8/stage2_new2/three_camera_global` batch. It excludes `four_camera`,
`three_camera_front`, and scene12, which currently has no matching HDF5 output.

The source of truth is each episode's `states/aligned_joints.h5`. Raw MCAP is
unavailable. This repair changes action values only; it does not change state,
timestamps, frame count, video, task metadata, quality grade, or episode
numbering.

Continuous all-zero action is valid data. Absolute zero, near-zero, and an
episode being stationary are never sufficient reasons to modify a frame.

## Target anomaly

The target is a short action excursion that immediately returns to the local
trajectory and repeats near 10 Hz in a stationary or near-stationary window.
Observed variants are:

1. a normally non-zero action briefly switching to a zero/default sample;
2. a held zero action briefly switching to a measured-state-like sample;
3. a held or slowly changing action briefly switching to another non-zero
   source, commonly a state-like sample.

The common signature is a periodic `A -> B -> A` source-mixing pattern, not the
numeric value of `A` or `B`.

## Detection policy

Detection operates independently on each 7D arm action. The first six joint
positions determine motion and anomaly thresholds; the gripper is carried with
an accepted arm repair but does not by itself trigger one.

An episode-side is eligible only when a 60-frame analysis window satisfies all
of:

- an 11-frame rolling-median baseline has maximum per-joint residual RMS of at
  least `0.005 rad`;
- baseline RMS velocity is at most `0.01 rad/frame`;
- the dominant residual frequency is between `8 and 12 Hz`;
- power within `peak +/- 0.5 Hz` is at least `0.75` of residual power between
  `3 and 14.5 Hz`;
- at least six reversible excursions share the same frame-modulo-3 phase, with
  consecutive supported occurrences separated by 3 or 6 frames.

An individual one- or two-frame run is repairable only when:

- it has valid anchors on both sides;
- the six-joint excursion norm from anchor interpolation is at least
  `0.005 rad`;
- at least two joints differ by at least `0.001 rad`;
- the anchor-to-anchor change divided by the interpolated interval is at most
  `0.11 rad/frame`;
- the return error is at most `0.25` of the excursion;
- exactly one source-direction hypothesis is supported.

Source direction is resolved conservatively:

- a zero/default run is eligible only when it is the minority branch in the
  analysis window;
- a non-zero state-mixing run is eligible when it is materially closer to the
  same-frame state than the interpolated action baseline, using a maximum
  distance ratio of `0.35`;
- when zero duty cycle, state distance, or competing phase hypotheses do not
  identify one direction unambiguously, the run is left unchanged.

Runs of three or more frames, boundary runs, windows with conflicting phases,
and motion above the gates are not repaired. They are recorded as unresolved.
This fail-closed rule also protects continuous zero plateaus.

Frequency and phase tests select an episode window. The actual repair mask
contains only the accepted short excursions; the rest of the window remains
unchanged.

## Repair operation

Each accepted run is replaced by linear interpolation between its two trusted
anchors. All seven values for that arm, including its gripper, are interpolated
together.

For each repaired frame:

- update the corresponding slice of `action/joint/position`;
- synchronize `action/left_effector/position` from joint index 6 or
  `action/right_effector/position` from joint index 13;
- do not modify any other dataset.

There is no whole-trajectory low-pass, notch, Savitzky-Golay, or Hampel filter.
Every action element outside the explicit repair mask must remain bitwise
unchanged.

## Tool architecture

Add a reusable core module and a thin command-line interface:

- `scripts/embodied_data_pipeline-main/quality_pipeline/action_repair.py`
  contains HDF5 loading, detector, repair-plan construction, transactional
  application, LeRobot synchronization, verification, and rollback.
- `scripts/embodied_data_pipeline-main/scripts/repair_action_jitter_hdf5.py`
  exposes `audit`, `plan`, `apply`, `verify`, and `rollback` subcommands.
- `tests/test_repair_action_jitter_hdf5.py` covers numerical behavior,
  persistence, synchronization, and rollback.

The plan is immutable and records:

- tool version and policy thresholds;
- every source and derived path;
- pre-apply SHA-256 digests;
- episode, side, frame indices, amplitudes, source-direction evidence, and
  replacement values;
- unresolved candidates and reasons;
- expected post-repair metrics.

`apply` refuses to operate when a file digest or dataset shape differs from the
approved plan.

## Backup and transaction safety

Applying a plan requires an explicit backup root. A UTC run directory under
`/home/agilex/repair_backups/action_jitter_repair/` contains:

- `plan.json`;
- `manifest.json`;
- `repair_report.json`;
- `verification.json`;
- `files/` with every original HDF5, affected Parquet file, and affected
  `episodes_stats.jsonl`, preserving paths relative to `stage2_new`.

Backups use real `copy2` copies, never hard links, and are verified against the
source SHA-256 before modification.

For HDF5, the tool:

1. clones the original file to a unique temporary file in the same directory;
2. edits only target datasets in the clone with `h5py` slice assignment;
3. verifies object structure, attributes, storage properties, unchanged
   datasets, and the exact allowed action differences;
4. flushes and `fsync`s the temporary file;
5. confirms the original digest has not changed since planning;
6. atomically installs the clone with `os.replace` and `fsync`s the parent
   directory.

If any apply or verification step fails, all files already changed in the run
are restored from verified backups through the same temporary-file and atomic
replace procedure. The manifest records `planning`, `applying`, `verified`,
`rolling_back`, or `rolled_back` state so interrupted runs can be diagnosed and
resumed only through verification.

## LeRobot synchronization

HDF5 remains authoritative. For each repaired HDF5 that appears in an existing
`meta/episode_name_mapping.json`, the tool resolves the exact grade and LeRobot
episode. Before HDF5 apply, it verifies:

- the mapping is unique;
- Parquet exists and has the same frame count;
- its first 14 action dimensions match the pre-repair HDF5 within `1e-6`;
- the action schema is fixed-size float32 and has at least 14 dimensions.

After HDF5 repair, synchronization:

- replaces only the first 14 dimensions of the Parquet `action` column with
  repaired HDF5 values converted to float32;
- preserves the remaining action dimensions and every non-action column;
- preserves Arrow schema metadata and SNAPPY compression;
- recomputes only that episode's `action` min, max, mean, standard deviation,
  and count in `meta/episodes_stats.jsonl`;
- leaves videos, mapping, tasks, episode indices, and frame indices unchanged.

Parquet and JSONL writes use verified temporary files and atomic replacement.
HDF5 files with no existing LeRobot mapping are repaired normally and reported
as having no derivative to synchronize. A planned mapped output that fails
preflight prevents the transaction from starting.

## Verification and acceptance

Automated tests must establish:

- continuous all-zero, long zero plateaus, held non-zero action, smooth ramps,
  normal steps, and non-periodic isolated changes remain unchanged;
- synthetic one- and two-frame 10 Hz zero/default and state-mixing corruption
  is detected and repaired only when all gates pass;
- ambiguous source direction, missing anchors, long runs, and fast motion are
  reported but unchanged;
- HDF5 datasets, attributes, storage properties, and values outside the repair
  mask are preserved;
- injected failures trigger complete rollback;
- LeRobot action values and episode action statistics match repaired HDF5 while
  all unrelated columns and videos remain unchanged.

Before applying to production data, a dry-run audit must show zero proposed
changes in all 154 scene10 HDF5 episodes and zero changes to known continuous
all-zero episodes.

After apply, verification reopens every scoped HDF5 and asserts:

1. all preplanned repairs equal their interpolation targets;
2. no action element outside the plan changed;
3. state, timestamps, frame count, and non-action content match backups;
4. repaired periodic-return events no longer satisfy the detector;
5. unresolved events remain listed rather than silently altered;
6. every mapped LeRobot action agrees with repaired HDF5 within float32
   tolerance and its action statistics are exact.

Existing QC reports are marked stale by the repair report. QC is rerun for
affected batches after verification, but existing manual grades are not
changed automatically.

## Out of scope

- Reconstructing ambiguous or sustained corruption without trusted anchors.
- Smoothing real robot motion or state.
- Modifying four-camera or three-camera-front datasets.
- Changing manual quality grades.
- Fixing the live Piper publisher and collection preflight; prevention at
  collection time is a separate change after the historical-data repair.

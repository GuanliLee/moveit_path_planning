# Strict HDF5 Task Source and Canonical Beverage Names Design

**Status:** Approved on 2026-07-10

## Problem

The ALOHA profile currently supplies `grasp the bottles on the table` when an
episode does not expose task metadata. HDF5-to-LeRobot conversion also accepts
`--task` and `--default-task`, so a page or command-line value can replace the
task stored in HDF5. Quality-grade conversion runs independently for A, B, C,
and F, which has allowed outputs produced at different times to retain
different task text.

The result is a LeRobot `meta/tasks.jsonl` value that may describe neither the
source HDF5 nor the actual episode.

## Goals

- Make the ALOHA HDF5 file the only task source for HDF5-to-LeRobot conversion.
- Remove the ALOHA profile fallback task.
- Stop ALOHA LeRobot conversion when an HDF5 episode has no usable task.
- Let the Web workflow ask the user for one manual task when task metadata is
  missing, write it into the missing HDF5 files first, and then convert.
- Never overwrite a non-empty HDF5 task with the manual page value.
- Verify task consistency for every A/B/C/F grade output.
- Correct stale grade outputs when the user runs “Generate LeRobot” again.
- Normalize existing task metadata under exactly these roots:
  `/home/agilex/data/stage2/hdf5_episodes` and
  `/home/agilex/data/stage2/lerobot`.
- Preserve the product category, quality grade, episode number, sensor data,
  and video data while normalizing task names.

## Non-goals

- Automatically infer task text from dataset names, quality reports, or model
  output.
- Change quality-grade assignment or episode numbering.
- Rewrite existing LeRobot directories merely by starting the Web server. A
  user-triggered LeRobot generation run is required.
- Change G2 task-source behavior unless a shared helper must preserve existing
  G2 behavior.
- Create data for canonical products that are not present in the two requested
  roots.

## Canonical Beverage Vocabulary

The following 19 product names are independent canonical values. Their order,
capitalization, spacing, and punctuation are fixed:

1. `Yakult`
2. `Guangming Probiotic Milk`
3. `Coca-Cola`
4. `COSTA Peach Oolong`
5. `Taro Milk`
6. `Robuk Velvet Latte`
7. `Yili Peach Yogurt`
8. `HK Orange Fanta`
9. `Aojiru`
10. `NEVER Coconut Latte`
11. `Daily C Orange Juice`
12. `Daily C Grape Juice`
13. `COSTA Grape Jasmine`
14. `Wanglaoji`
15. `AD Calcium Milk`
16. `Yili Strawberry Yogurt`
17. `Dahongpao Milk Tea`
18. `Sprite`
19. `Royal Coconut`

No entry is a parent category for another entry. Normalization changes only a
name variant within an already-known dataset category; it never reclassifies a
dataset by fuzzy matching its current task text.

The canonical task sentence is:

```text
Grasp <canonical beverage name> with the left hand.
```

For example:

```text
Grasp Daily C Grape Juice with the left hand.
```

Dataset identity or an explicit user selection determines the beverage name.
An unknown or ambiguous dataset slug is reported for manual selection rather
than guessed.

## Existing-data Audit Snapshot

The read-only audit on 2026-07-10 found:

- 1,012 HDF5 episodes across 13 dataset directories;
- no HDF5 episode with a completely missing root task;
- 414 HDF5 tasks with a non-canonical product name or command sentence;
- 58 HDF5/sidecar disagreements;
- 1,014 existing LeRobot grade episodes across 42 A/B/C/F task files;
- approximately 476 LeRobot episodes resolving to a non-canonical task;
- one or more stale LeRobot episodes that no longer have an exact one-to-one
  count match with the current HDF5 root; migration reports these as orphans
  but does not delete them.

Only five of the 19 canonical products are present in the requested roots:
`NEVER Coconut Latte`, `Daily C Orange Juice`, `Daily C Grape Juice`,
`HK Orange Fanta`, and `Wanglaoji`.

Observed variants map by dataset identity as follows:

| Dataset identity | Observed task variants | Canonical task |
|---|---|---|
| `nevercoffee` | `grasp the bottles on the table`, `Grasp NEVER Coconut Latte with the left hand.` | `Grasp NEVER Coconut Latte with the left hand.` |
| `daily-c-orange-juice` | `Grasp Orange Juice with the left hand.`, `Grasp Daily C Orange Juice with the left hand.` | `Grasp Daily C Orange Juice with the left hand.` |
| `daily-c-grape-juice` | `Grasp Grape Juice with the left hand.`, `grasp the bottles on the table`, `Target: Grape Juice. Pick the Grape Juice from the shelf and place it into the cart.`, `Grasp Daily C Grape Juice with the left hand.` | `Grasp Daily C Grape Juice with the left hand.` |
| `hk-orange-fanta` | `Grasp Fanta with the left hand.` | `Grasp HK Orange Fanta with the left hand.` |
| `wanglaoji` | `grasp the bottles on the table`, `Grasp Wanglaoji with the left hand.` | `Grasp Wanglaoji with the left hand.` |

## Canonical ALOHA Task

For ALOHA conversion, the canonical task is read from
`states/aligned_joints.h5`. Accepted HDF5 metadata is checked in this order:

1. non-empty root attribute `task`;
2. the first non-empty item in root attribute `tasks_json`;
3. non-empty root attribute `text`.

Whitespace around a value is removed. Sidecar JSON, robot-profile defaults,
dataset directory names, Web request parameters, and LeRobot metadata are not
task sources for an HDF5 file that already exists.

The converter copies this canonical value to the LeRobot episode task,
`tasks`, and `full_instructions_en` metadata so those representations cannot
disagree.

## Missing-task Interaction

Before grade commands are created, the Web pipeline scans every selected ALOHA
HDF5 episode.

- If every episode has a canonical task, conversion proceeds and the page task
  input is not allowed to override any value.
- If one or more episodes are missing a task and the page task input is empty,
  the job stops before creating grade outputs. The error lists the missing
  episode names and tells the user to enter a task in the task-text field and
  retry.
- If one or more episodes are missing a task and the page task input is
  non-empty, that value is written only to the missing HDF5 root `task`
  attributes. The corresponding `meta/episode_meta.json` files are updated
  with matching `task`, `tasks`, and `full_instructions_en` fields for QC and
  replay compatibility. Existing non-empty HDF5 tasks remain unchanged.

HDF5 writes and sidecar writes must fail the job with the affected path in the
error message. Sidecar updates use a temporary file followed by replacement.

## Converter Behavior

- Remove `tasks.default` from the ALOHA profile.
- ALOHA conversion must not consume a profile default.
- The Web grade commands must not pass `--task` to the LeRobot converter.
- Direct ALOHA conversion with `--task` or `--default-task` must be rejected
  with guidance to write the task into HDF5 first. Existing G2 compatibility
  may retain those options when the active profile is not ALOHA.
- A missing canonical ALOHA HDF5 task raises an error naming the HDF5 file and
  accepted attributes. It must never emit an empty or fallback LeRobot task.

MCAP-to-HDF5 conversion may still use explicit user task text because that
stage writes the value into HDF5. With the ALOHA profile default removed, an
MCAP episode that has neither instruction metadata nor explicit task text must
fail and direct the user to provide task text.

## Quality-grade Consistency

The A/B/C/F split changes only the subset and numbering of episodes. It must
not change their tasks.

Before conversion, one canonical-task map is built from the source HDF5 root:

```text
source HDF5 episode -> exact canonical task
```

Every grade command uses that same source root and an episode-index filter.
After all grade commands finish, a validation step checks each grade mapping,
Parquet `task_index`, and `meta/tasks.jsonl` against the source map.

- For each episode, resolving its Parquet `task_index` through that grade's
  `tasks.jsonl` must produce the exact canonical HDF5 task.
- If all source HDF5 episodes share one task, every generated grade directory
  must expose exactly that same single task.
- If source HDF5 episodes contain multiple tasks, a grade may contain only the
  subset used by its episodes, but every episode must still resolve to its own
  HDF5 task.
- Any missing, extra, or mismatched task fails the job and reports the grade,
  episode, HDF5 task, and LeRobot task.

The current Web grade command uses `--overwrite`. Therefore rerunning “Generate
LeRobot” rebuilds grade outputs and removes stale fallback tasks before the
post-conversion check runs.

## Existing-data Migration

A dedicated migration command handles the two approved roots. It supports a
read-only audit mode and an explicit apply mode.

For each recognized HDF5 dataset, apply mode:

1. derives the canonical beverage from an explicit dataset-slug map;
2. writes the canonical sentence to the HDF5 root `task` attribute;
3. removes conflicting task meaning by synchronizing HDF5 `tasks_json` when it
   exists, and a `text` attribute only when it is a plain task string rather
   than a structured JSON payload;
4. writes matching `task`, `tasks`, and `full_instructions_en` fields to
   `meta/episode_meta.json`;
5. leaves all non-task HDF5 datasets, attributes, and sidecar fields intact.

For each recognized LeRobot A/B/C/F directory, apply mode:

1. rewrites `meta/tasks.jsonl` to one canonical task row for these current
   single-product datasets;
2. rewrites each Parquet `task_index` to that row's index without changing any
   other column values or schema;
3. synchronizes task fields in `meta/episodes.jsonl` and
   `meta/episode_name_mapping.json` when present;
4. refreshes task counts in `meta/info.json` when present;
5. never changes quality grades, episode indices, frame counts, media, state,
   or action values.

Before apply mode changes a value, it writes
`/home/agilex/data/stage2/task-migration-<timestamp>.json`, a migration
manifest containing the file path, field, original value, and new value. The
manifest is sufficient to restore task attributes, JSON/JSONL fields, and
Parquet `task_index` values without copying large sensor or video files. JSON,
JSONL, and Parquet outputs use temporary files followed by atomic replacement.

Unknown dataset identities, unreadable files, mixed product identities, and
ambiguous mappings are hard failures before mutation. Orphan LeRobot episodes
are normalized from their recognized dataset identity and reported, but not
deleted or moved.

After mutation, the migration performs a fresh full audit. Success requires:

- every recognized HDF5 and sidecar task to equal its canonical sentence;
- every LeRobot episode's Parquet `task_index` to resolve to the same canonical
  sentence as its recognized source dataset;
- every A/B/C/F directory for one dataset to expose the same canonical task;
- zero occurrences of the removed fallback sentence in task metadata under
  the two approved roots.

## User-facing Changes

- Rename the task input help text to explain that it fills missing HDF5 task
  metadata and does not override existing HDF5 tasks.
- Replace “will use profile default” status text with “task must come from
  HDF5; missing values require manual input.”
- Show missing episode names in the job error.
- Log how many HDF5 files received a manual task and how many retained existing
  tasks.

The launcher option `--task-text` remains available as a page preset. Its new
meaning is “manual value used only for missing HDF5 tasks.”

## Components and Files

- `scripts/embodied_data_pipeline-main/robot_profiles/aloha.yaml`
  removes the default task.
- `scripts/embodied_data_pipeline-main/quality_pipeline/episode_io.py`
  exposes canonical HDF5 task metadata without allowing a sidecar to outrank
  HDF5.
- `scripts/embodied_data_pipeline-main/quality_pipeline/task_names.py`
  owns the exact 19-name vocabulary, dataset-slug map, and canonical sentence
  formatter.
- `scripts/embodied_data_pipeline-main/lerobot_conversion/scripts/convert_hdf5_to_lerobot_v2.py`
  enforces strict ALOHA task resolution and rejects fallback/override paths.
- `scripts/embodied_data_pipeline-main/scripts/pipeline_web_app.py`
  performs missing-task fill, grade command construction, user messaging, and
  post-conversion grade validation.
- `scripts/embodied_data_pipeline-main/scripts/normalize_aloha_task_metadata.py`
  audits, applies, verifies, and rolls back task-only metadata migrations for
  the two approved data roots.
- `scripts/collection/collect_mobile_pipeline_qc_web.sh`
  documents the revised `--task-text` semantics.
- Tests cover canonical resolution, missing-task behavior, non-overwrite
  behavior, 19-name validation, migration dry-run/apply behavior, rollback
  manifests, grade consistency, and generated command arguments.

## Testing Strategy

Tests are written before implementation and must demonstrate these failures
against the current code:

1. ALOHA profile has no task fallback.
2. An HDF5 task wins even when sidecar metadata contains a different task.
3. Missing HDF5 task blocks conversion with actionable episode names.
4. Manual task input fills only missing HDF5 files.
5. Grade conversion commands do not contain `--task`.
6. A/B/C/F outputs for a single-task source all resolve to exactly the same
   HDF5 task.
7. A mixed-task source resolves every grade episode to its own HDF5 task.
8. A stale or mismatched grade task causes validation to fail.
9. The canonical vocabulary contains exactly the approved 19 names.
10. Known dataset slugs produce the exact canonical sentence while unknown
    slugs fail without mutation.
11. Migration updates only task metadata and preserves all other HDF5,
    sidecar, JSONL, Parquet, and grade fields.
12. Audit mode reports the current affected counts without writing files.

Existing focused Web-pipeline tests and shell/Python syntax checks run after
the new tests. The repository's known unrelated full-suite dependency and
baseline failures are reported separately rather than hidden.

## Acceptance Criteria

- The string `grasp the bottles on the table` is absent from the ALOHA profile
  and cannot be produced as an ALOHA fallback.
- ALOHA HDF5-to-LeRobot conversion cannot proceed without a canonical HDF5
  task.
- Manual task input is persisted to missing HDF5 files before conversion and
  cannot replace existing HDF5 tasks.
- Every generated grade episode resolves to the exact task stored in its
  source HDF5.
- Rerunning grade conversion repairs previously stale grade outputs through a
  full overwrite and passes the consistency validator.
- Existing data under the two approved roots uses only canonical task
  sentences for recognized datasets after migration.
- The migration manifest records every changed task value and supports exact
  task-metadata rollback.
- Categories absent from the existing data remain absent; normalization does
  not fabricate episodes or categories.

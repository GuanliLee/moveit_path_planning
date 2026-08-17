# HDF5 and LeRobot Quality-Grade Consistency Design

**Status:** Approved on 2026-07-13

## Problem

The Web pipeline currently has two independent quality-grade paths. The final
grade shown by the Web QC report determines whether an episode is converted
into the A, B, C, or F LeRobot dataset. The HDF5-to-LeRobot converter, however,
copies `quality_grade` only from the episode sidecar
`meta/episode_meta.json`. A grade that exists only in the QC report, or an A
grade synthesized by the Web status view, therefore selects the correct grade
directory without appearing in LeRobot `meta/episodes.jsonl`.

Existing-output validation checks files, episode counts, frames, tasks, and
feature layout, but it does not require a quality grade. Resume behavior can
therefore preserve old rows without `quality_grade`, producing both completely
missing and mixed-schema `episodes.jsonl` files.

## Goals

- Make the final quality grade displayed by the Web QC report the authoritative
  grade for every episode.
- Persist that grade in each HDF5 episode's existing
  `meta/episode_meta.json`; do not add an HDF5 binary attribute.
- Persist the same grade in every corresponding LeRobot
  `meta/episodes.jsonl` and `meta/episode_name_mapping.json` record.
- Validate every HDF5 and LeRobot episode, failing the Web job on a missing,
  invalid, or inconsistent grade.
- Repair existing metadata without re-encoding Parquet data or videos.
- Preserve every unrelated sidecar, mapping, and LeRobot episode field.

## Non-goals

- Change how QC assigns A, B, C, or F.
- Store `quality_grade` in `states/aligned_joints.h5`.
- Change episode numbering, task text, Parquet content, sensor data, or video
  data.
- Make the generic HDF5-to-LeRobot converter read Web QC report files
  directly.

## Authoritative Grade

For Web-driven conversion, the authoritative grade is the normalized final
`quality_grade` in the same finalized QC/status row used by the page. Accepted
values are exactly `A`, `B`, `C`, and `F`.

The existing status rule remains unchanged:

1. use an explicit manual/collection grade when present;
2. use `F` for a collection failure;
3. otherwise use `A`.

Every discovered HDF5 episode must have exactly one finalized status row. A
missing row, duplicate episode identity, or invalid grade is a hard error
before LeRobot conversion starts.

## Data Flow

When an operator saves an A/B/C/F grade in the Web QC page, the save request
first resolves exactly one HDF5 episode and atomically writes the final grade
to that episode's `meta/episode_meta.json`. The existing
`manual_failure_annotations.json` overlay is then retained and updated for
backward compatibility with existing Web readers. A successful save response
therefore guarantees that the HDF5 sidecar already contains the displayed
grade before any LeRobot job can be started.

The Web LeRobot workflow performs these stages in order:

1. Finalize the QC rows that the page displays.
2. Build an exact map from HDF5 episode identity to final grade.
3. Atomically synchronize the grade into every episode's
   `meta/episode_meta.json`.
4. Generate the renumber plan and A/B/C/F subsets from the same map.
5. Run the HDF5-to-LeRobot converter for each non-empty subset while explicitly
   supplying that subset's grade.
6. Synchronize grade metadata in both LeRobot metadata files.
7. Validate HDF5 sidecars and all generated/existing LeRobot grade records
   against the original final-grade map.

One map is reused throughout the run. No later stage independently re-derives
or defaults a grade.

## HDF5 Sidecar Synchronization

Before conversion, each HDF5 episode sidecar is updated atomically. Existing
JSON fields are preserved. The synchronized fields are:

- `quality_grade`: the authoritative `A`, `B`, `C`, or `F` value;
- `manual_quality_grade`: the same value, for compatibility with existing
  replay and Web readers;
- `manual_failure`: `true` only for `F`, otherwise `false`.

Existing reason labels, reason codes, review notes, task metadata, source
paths, and frame metadata remain unchanged. An unreadable or non-object
sidecar is a hard error naming the affected file. A missing sidecar is created
with the three synchronized fields.

The manual-grade save endpoint uses the same sidecar synchronization helper as
the pre-conversion workflow. It requires one exact HDF5 episode-name match; a
missing or duplicate episode, invalid grade, or malformed sidecar fails the
save instead of updating only the compatibility annotation file. The
pre-conversion synchronization remains in place as an idempotent safety net
for older annotations and data created before this behavior existed.

## Converter Contract

The converter gains an optional explicit quality-grade argument accepting only
`A`, `B`, `C`, or `F`.

- Web grade commands always pass the grade for their selected subset.
- When supplied, the explicit value is authoritative and is written into the
  episode instruction metadata used to patch `meta/episodes.jsonl`.
- If the source sidecar contains a different valid grade, conversion fails
  instead of silently overwriting the conflict.
- Direct converter usage without an explicit grade retains sidecar-based
  behavior, but a present invalid grade fails rather than being copied.

This keeps the converter independent of the Web QC report layout while making
the Web-to-converter boundary explicit.

## LeRobot Metadata Synchronization

After each grade conversion, every LeRobot episode is resolved through
`meta/episode_name_mapping.json` to its source HDF5 episode. The workflow
atomically writes the authoritative grade to:

- the mapping record's `quality_grade`;
- the matching `meta/episodes.jsonl` row's `quality_grade`.

The grade dataset's top-level mapping metadata also records its grade. The
patch operation preserves row order by `episode_index` and preserves all
unrelated fields. Missing mapping records, duplicate episode indices, and
unresolvable source episodes are hard failures.

Because this step changes only JSON/JSONL metadata, it is also the repair path
for existing datasets. It must not open or rewrite Parquet or video files.

## Validation

The final validation covers every episode, not just aggregate counts.

For each HDF5 episode:

- a sidecar exists and is a JSON object;
- `quality_grade` and `manual_quality_grade` both equal the Web final grade;
- `manual_failure` is true exactly when the final grade is `F`.

For each LeRobot episode:

- a mapping record and `episodes.jsonl` row both exist;
- both records contain a valid `quality_grade`;
- both grades equal each other, the source HDF5 sidecar grade, the Web final
  grade, and the A/B/C/F dataset directory;
- there are no extra LeRobot episodes without a source-map entry.

Any error fails the job with dataset grade, episode index, source episode/path,
expected grade, and actual value. Successful generation therefore guarantees
that every page-visible HDF5 episode and every generated LeRobot episode has
one consistent grade.

## Existing-Data Repair

The metadata synchronization and validation logic is exposed as a callable
workflow step that can run on existing A/B/C/F datasets. It uses the current
Web QC final-grade map and existing LeRobot source mapping. It repairs missing
or stale grade fields in HDF5 sidecars, LeRobot mappings, and
`meta/episodes.jsonl`, then runs the same final validation.

The repair is idempotent. A second run produces no changes. It does not rebuild
LeRobot datasets and does not modify Parquet, media, tasks, episode indices, or
frame counts.

## Testing

Automated tests cover:

- default A from the finalized Web status being persisted to a previously
  grade-less sidecar;
- an operator grade save immediately updating the matching HDF5 sidecar while
  preserving unrelated fields and the compatibility annotation;
- an operator grade save rejecting missing, duplicate, or malformed HDF5
  episode metadata without reporting success;
- explicit B, C, and F values and correct `manual_failure` behavior;
- preservation of unrelated sidecar fields;
- missing and malformed sidecars;
- explicit converter grade propagation and sidecar-grade conflicts;
- complete and mixed missing-grade LeRobot metadata repair;
- mapping/`episodes.jsonl` mismatch and duplicate-index failures;
- validation across HDF5, mapping, `episodes.jsonl`, and grade directory;
- idempotent repair without touching Parquet or video files.

## Components

- `scripts/embodied_data_pipeline-main/scripts/pipeline_web_app.py` owns the
  authoritative Web final-grade map, HDF5 sidecar synchronization, grade
  command construction, existing-metadata repair, and end-to-end validation.
- `scripts/embodied_data_pipeline-main/lerobot_conversion/scripts/convert_hdf5_to_lerobot_v2.py`
  accepts and validates an explicit grade and writes it to each processed
  LeRobot episode row.
- Tests under `tests/` exercise the synchronization, converter contract,
  repair, and validation behavior.

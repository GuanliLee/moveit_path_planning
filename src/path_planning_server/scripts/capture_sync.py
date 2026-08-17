"""Pure helpers for selecting a fresh, synchronized RGB-D capture pair."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Iterable, Optional


@dataclass(frozen=True)
class TimedMessageSample:
    """One received ROS message with process-local arrival metadata."""

    message: Any
    sequence: int
    arrival_monotonic_s: float
    receipt_ros_s: float


@dataclass(frozen=True)
class MatchedSensorPair:
    """A selected RGB/depth pair and its synchronization diagnostics."""

    rgb: TimedMessageSample
    depth: TimedMessageSample
    rgb_stamp_s: float
    depth_stamp_s: float
    stamp_skew_s: float
    arrival_skew_s: float
    arrival_age_s: float
    matched_pair_count: int


def message_stamp_seconds(message: Any) -> float:
    """Return a positive finite ROS header timestamp in seconds."""

    stamp = message.header.stamp
    seconds = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
    if not math.isfinite(seconds) or seconds <= 0.0:
        raise ValueError("camera message has an invalid or zero header timestamp")
    return seconds


def _eligible_samples(
    samples: Iterable[TimedMessageSample],
    *,
    minimum_sequence: int,
    barrier_monotonic_s: float,
) -> list[TimedMessageSample]:
    eligible = []
    for sample in samples:
        if (
            sample.sequence > minimum_sequence
            and sample.arrival_monotonic_s >= barrier_monotonic_s
        ):
            eligible.append(sample)
    return eligible


def select_synced_pair(
    rgb_samples: Iterable[TimedMessageSample],
    depth_samples: Iterable[TimedMessageSample],
    *,
    minimum_rgb_sequence: int,
    minimum_depth_sequence: int,
    barrier_monotonic_s: float,
    now_monotonic_s: float,
    max_header_skew_s: float,
    max_arrival_skew_s: float,
    max_arrival_age_s: float,
    discard_pairs_after_barrier: int,
) -> Optional[MatchedSensorPair]:
    """Select the newest fresh pair after a capture barrier.

    Header timestamps are used only to associate RGB with aligned depth.  They
    are deliberately not used as an absolute freshness clock: RealSense
    ``global_time`` can drift relative to the host ROS clock.  Freshness and
    the post-settle barrier therefore use ``time.monotonic()`` arrival times.
    """

    rgb = _eligible_samples(
        rgb_samples,
        minimum_sequence=minimum_rgb_sequence,
        barrier_monotonic_s=barrier_monotonic_s,
    )
    depth = _eligible_samples(
        depth_samples,
        minimum_sequence=minimum_depth_sequence,
        barrier_monotonic_s=barrier_monotonic_s,
    )
    if not rgb or not depth:
        return None

    rgb.sort(key=lambda sample: (sample.arrival_monotonic_s, sample.sequence))
    depth.sort(key=lambda sample: (sample.arrival_monotonic_s, sample.sequence))

    def candidate_for(
        rgb_sample: TimedMessageSample,
        depth_sample: TimedMessageSample,
    ):
        try:
            rgb_stamp_s = message_stamp_seconds(rgb_sample.message)
            depth_stamp_s = message_stamp_seconds(depth_sample.message)
        except ValueError:
            return None
        stamp_skew_s = abs(rgb_stamp_s - depth_stamp_s)
        arrival_skew_s = abs(
            rgb_sample.arrival_monotonic_s - depth_sample.arrival_monotonic_s
        )
        if not (
            stamp_skew_s <= max_header_skew_s
            and arrival_skew_s <= max_arrival_skew_s
        ):
            return None
        return (
            stamp_skew_s,
            arrival_skew_s,
            max(rgb_sample.arrival_monotonic_s, depth_sample.arrival_monotonic_s),
            rgb_sample,
            depth_sample,
            rgb_stamp_s,
            depth_stamp_s,
        )

    def is_better(first: tuple, second: tuple) -> bool:
        """Prefer max pair count, min total skew, then the newer pair set."""

        def score(pairs: tuple) -> tuple:
            return (
                len(pairs),
                -sum(pair[0] for pair in pairs),
                -sum(pair[1] for pair in pairs),
                pairs[-1][2] if pairs else -math.inf,
            )

        return score(first) > score(second)

    # Ordered dynamic programming prevents cross-frame associations and, unlike
    # a greedy closest-stamp choice, maximizes the number of usable pairs first.
    # With 30-entry queues this O(N*M) matcher is negligible in the callback path.
    best: list[list[tuple]] = [
        [tuple() for _ in range(len(depth) + 1)]
        for _ in range(len(rgb) + 1)
    ]
    for rgb_index in range(1, len(rgb) + 1):
        for depth_index in range(1, len(depth) + 1):
            selected = best[rgb_index - 1][depth_index]
            skip_depth = best[rgb_index][depth_index - 1]
            if is_better(skip_depth, selected):
                selected = skip_depth
            candidate = candidate_for(
                rgb[rgb_index - 1],
                depth[depth_index - 1],
            )
            if candidate is not None:
                with_pair = best[rgb_index - 1][depth_index - 1] + (candidate,)
                if is_better(with_pair, selected):
                    selected = with_pair
            best[rgb_index][depth_index] = selected

    matched = best[len(rgb)][len(depth)]
    discard_count = max(0, int(discard_pairs_after_barrier))
    if len(matched) <= discard_count:
        return None

    selected = matched[-1]
    arrival_age_s = now_monotonic_s - min(
        selected[3].arrival_monotonic_s,
        selected[4].arrival_monotonic_s,
    )
    # Older post-barrier pairs only prove that the camera pipeline has flushed;
    # they need not remain fresh while waiting for the configured discard count.
    # Freshness applies to the newest pair that will actually be consumed.
    if not 0.0 <= arrival_age_s <= max_arrival_age_s:
        return None

    return MatchedSensorPair(
        rgb=selected[3],
        depth=selected[4],
        rgb_stamp_s=selected[5],
        depth_stamp_s=selected[6],
        stamp_skew_s=selected[0],
        arrival_skew_s=selected[1],
        arrival_age_s=arrival_age_s,
        matched_pair_count=len(matched),
    )

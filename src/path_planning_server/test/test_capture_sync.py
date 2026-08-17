from dataclasses import dataclass

import pytest

from capture_sync import TimedMessageSample, select_synced_pair


@dataclass
class _Stamp:
    sec: int
    nanosec: int


@dataclass
class _Header:
    stamp: _Stamp


@dataclass
class _Message:
    header: _Header


def _sample(sequence: int, stamp_s: float, arrival_s: float) -> TimedMessageSample:
    sec = int(stamp_s)
    nanosec = int(round((stamp_s - sec) * 1.0e9))
    return TimedMessageSample(
        message=_Message(_Header(_Stamp(sec, nanosec))),
        sequence=sequence,
        arrival_monotonic_s=arrival_s,
        receipt_ros_s=1000.0 + arrival_s,
    )


def _select(rgb, depth, **overrides):
    options = {
        "minimum_rgb_sequence": 10,
        "minimum_depth_sequence": 20,
        "barrier_monotonic_s": 5.0,
        "now_monotonic_s": 5.12,
        "max_header_skew_s": 0.005,
        "max_arrival_skew_s": 0.050,
        "max_arrival_age_s": 0.150,
        "discard_pairs_after_barrier": 2,
    }
    options.update(overrides)
    return select_synced_pair(rgb, depth, **options)


def test_selects_third_new_pair_after_barrier() -> None:
    rgb = [
        _sample(10, 20.000, 4.99),
        _sample(11, 20.033, 5.02),
        _sample(12, 20.066, 5.05),
        _sample(13, 20.099, 5.08),
    ]
    depth = [
        _sample(20, 20.000, 4.991),
        _sample(21, 20.033, 5.021),
        _sample(22, 20.066, 5.052),
        _sample(23, 20.099, 5.082),
    ]

    selected = _select(rgb, depth)

    assert selected is not None
    assert selected.rgb.sequence == 13
    assert selected.depth.sequence == 23
    assert selected.matched_pair_count == 3


def test_matches_queued_pair_when_latest_messages_do_not_match() -> None:
    rgb = [
        _sample(11, 30.000, 5.01),
        _sample(12, 30.033, 5.04),
        _sample(13, 30.066, 5.07),
        _sample(14, 30.099, 5.10),
    ]
    depth = [
        _sample(21, 30.000, 5.012),
        _sample(22, 30.033, 5.042),
        _sample(23, 30.066, 5.072),
        _sample(24, 31.000, 5.105),
    ]

    selected = _select(rgb, depth)

    assert selected is not None
    assert selected.rgb.sequence == 13
    assert selected.depth.sequence == 23


@pytest.mark.parametrize(
    ("header_skew_s", "arrival_skew_s", "expected"),
    ((0.004999, 0.049999, True), (0.005001, 0.001, False), (0.0, 0.050001, False)),
)
def test_sync_threshold_boundaries(
    header_skew_s: float,
    arrival_skew_s: float,
    expected: bool,
) -> None:
    rgb = [_sample(11, 40.000, 5.01)]
    depth = [_sample(21, 40.000 + header_skew_s, 5.01 + arrival_skew_s)]

    selected = _select(
        rgb,
        depth,
        discard_pairs_after_barrier=0,
        now_monotonic_s=5.10,
    )

    assert (selected is not None) is expected


def test_rejects_stale_or_pre_barrier_messages() -> None:
    rgb = [_sample(11, 50.000, 4.99), _sample(12, 50.033, 5.01)]
    depth = [_sample(21, 50.000, 4.991), _sample(22, 50.033, 5.011)]

    assert _select(
        rgb,
        depth,
        discard_pairs_after_barrier=0,
        now_monotonic_s=5.30,
    ) is None


def test_rejects_zero_header_stamp() -> None:
    rgb = [_sample(11, 0.0, 5.01)]
    depth = [_sample(21, 0.0, 5.011)]

    assert _select(
        rgb,
        depth,
        discard_pairs_after_barrier=0,
    ) is None


def test_age_limit_uses_older_arrival_in_pair() -> None:
    rgb = [_sample(11, 60.000, 5.00)]
    depth = [_sample(21, 60.000, 5.10)]

    assert _select(
        rgb,
        depth,
        discard_pairs_after_barrier=0,
        max_arrival_skew_s=0.20,
        max_arrival_age_s=0.15,
        now_monotonic_s=5.16,
    ) is None


def test_discarded_pairs_may_age_out_before_selecting_fresh_pair() -> None:
    rgb = [
        _sample(11, 70.000, 5.010),
        _sample(12, 70.167, 5.177),
        _sample(13, 70.334, 5.344),
    ]
    depth = [
        _sample(21, 70.000, 5.012),
        _sample(22, 70.167, 5.179),
        _sample(23, 70.334, 5.346),
    ]

    selected = _select(rgb, depth, now_monotonic_s=5.360)

    assert selected is not None
    assert selected.rgb.sequence == 13
    assert selected.depth.sequence == 23
    assert selected.arrival_age_s == pytest.approx(0.016)


def test_matcher_maximizes_pair_count_before_minimizing_skew() -> None:
    # Greedy closest-skew matching chooses RGB1<->Depth1 first and leaves only
    # one pair. Ordered maximum-cardinality matching correctly returns two.
    rgb = [_sample(11, 20.000, 5.000), _sample(12, 20.001, 5.010)]
    depth = [_sample(21, 20.001, 5.011), _sample(22, 20.006, 5.020)]

    selected = _select(
        rgb,
        depth,
        discard_pairs_after_barrier=1,
        now_monotonic_s=5.05,
    )

    assert selected is not None
    assert selected.matched_pair_count == 2
    assert selected.rgb.sequence == 12
    assert selected.depth.sequence == 22

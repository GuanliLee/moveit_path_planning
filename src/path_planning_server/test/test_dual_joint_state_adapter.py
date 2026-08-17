import importlib.util
from pathlib import Path

import pytest


SCRIPT = (
    Path(__file__).parents[1] / "scripts" / "dual_joint_state_adapter.py"
)
SPEC = importlib.util.spec_from_file_location("dual_joint_state_adapter", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)

JOINT_NAMES = list(MODULE.JOINT_NAMES)


def test_joint_state_publish_rate_is_180hz():
    assert MODULE.JOINT_STATE_PUBLISH_RATE_HZ == 180.0


def update(merger, side, stamp, velocity=(), effort=(), names=JOINT_NAMES):
    position = [float(index) for index in range(1, 9)]
    return merger.update(side, names, position, velocity, effort, stamp)


def test_merges_complete_fresh_states_with_prefixes():
    merger = MODULE.DualJointStateMerger(state_timeout=1.0)
    assert update(merger, "left", 10.0) is None

    merged = update(merger, "right", 10.5)

    assert merged.name == tuple(
        [f"left_joint{index}" for index in range(1, 9)]
        + [f"right_joint{index}" for index in range(1, 9)]
    )
    assert merged.position == tuple(float(index) for index in range(1, 9)) * 2
    assert len(merged.position) == 16


def test_reorders_values_to_canonical_joint_order():
    merger = MODULE.DualJointStateMerger()
    reversed_names = list(reversed(JOINT_NAMES))
    reversed_positions = list(reversed(range(1, 9)))
    merger.update("left", reversed_names, reversed_positions, (), (), 1.0)

    merged = update(merger, "right", 1.0)

    assert merged.position[:8] == tuple(float(index) for index in range(1, 9))


@pytest.mark.parametrize(
    ("names", "position"),
    [
        (JOINT_NAMES[:-1], list(range(1, 8))),
        (JOINT_NAMES, list(range(1, 8))),
        (JOINT_NAMES[:-1] + ["joint7"], list(range(1, 9))),
    ],
)
def test_rejects_invalid_names_or_position(names, position):
    with pytest.raises(ValueError):
        MODULE.validate_arm_state(names, position, (), (), 0.0)


def test_requires_both_states_to_be_within_timeout():
    merger = MODULE.DualJointStateMerger(state_timeout=1.0)
    update(merger, "left", 1.0)

    assert update(merger, "right", 2.1) is None
    assert update(merger, "left", 2.1) is not None


def test_velocity_and_effort_merge_only_when_complete_on_both_sides():
    complete = list(range(8))
    merger = MODULE.DualJointStateMerger()
    update(merger, "left", 1.0, velocity=complete, effort=complete)
    merged = update(merger, "right", 1.0, velocity=complete[:7], effort=complete)

    assert merged.velocity == ()
    assert merged.effort == tuple(float(index) for index in complete) * 2

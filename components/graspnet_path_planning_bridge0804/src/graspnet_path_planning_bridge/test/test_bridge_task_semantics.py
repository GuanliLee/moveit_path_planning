import pytest

from graspnet_path_planning_bridge.bridge_node import (
    SUPPORTED_TASK_TYPES,
    _keeps_grasp_ellipsoid,
)


def test_carry_is_a_supported_bridge_task() -> None:
    assert "CARRY" in SUPPORTED_TASK_TYPES


@pytest.mark.parametrize(
    "task_type",
    ("GRASP", "LIFT", "CARRY", "PLACE"),
)
def test_transport_tasks_keep_grasp_ellipsoid(task_type: str) -> None:
    assert _keeps_grasp_ellipsoid(task_type)


@pytest.mark.parametrize("task_type", ("MOVE", "RETURN_HOME"))
def test_non_transport_tasks_clear_grasp_ellipsoid(task_type: str) -> None:
    assert not _keeps_grasp_ellipsoid(task_type)

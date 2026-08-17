from pathlib import Path

import numpy as np
import pytest
import yaml

from geometry_msgs.msg import PoseStamped

from grasp_bridge_state_machine import (
    GraspBridgeStateMachine,
    _PLACE_TRANSPORT_STEP_ORDER,
    _bridge_task_type_for_step,
    _capture_joint_target_for_profile,
    _elevated_place_waypoint,
    _return_home_step,
)


@pytest.mark.parametrize(
    "step",
    ("RETREAT_AFTER_GRASP", "PLACE_WAYPOINT", "PLACE_XY", "PLACE_ORIENT"),
)
def test_place_transport_steps_keep_attached_grasp_geometry(step: str) -> None:
    assert _bridge_task_type_for_step(step) == "CARRY"


def test_place_transport_orients_before_horizontal_motion() -> None:
    assert _PLACE_TRANSPORT_STEP_ORDER == (
        "PLACE_LIFT",
        "PLACE_ORIENT",
        "PLACE_XY",
    )


@pytest.mark.parametrize(
    ("step", "expected"),
    (
        ("OPEN", "MOVE"),
        ("PRE_GRASP", "MOVE"),
        ("GRASP_APPROACH", "MOVE"),
        ("LIFT_AFTER_GRASP", "LIFT"),
        ("PLACE_LIFT", "LIFT"),
        ("GRASP", "GRASP"),
        ("PLACE", "PLACE"),
        ("RETURN_HOME", "RETURN_HOME"),
    ),
)
def test_other_steps_preserve_existing_bridge_semantics(
    step: str,
    expected: str,
) -> None:
    assert _bridge_task_type_for_step(step) == expected


def test_return_home_step_has_no_gripper_command() -> None:
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = "world"

    task_type, pose, gripper_command, gripper_opening = _return_home_step(
        initial_pose
    )

    assert task_type == "RETURN_HOME"
    assert pose is initial_pose
    assert not gripper_command
    assert gripper_opening == 0.0


def test_restock_right_capture_selects_configured_six_axis_target() -> None:
    restock = {
        "left": None,
        "right": (1.0, 0.4, -0.8, -0.2, 0.9, 0.1),
    }
    pick = {"left": None, "right": None}

    assert _capture_joint_target_for_profile(
        "grasp", "right", restock, pick
    ) == restock["right"]
    assert _capture_joint_target_for_profile(
        "pick", "right", restock, pick
    ) is None


def test_place_waypoint_is_eight_centimeters_above_final_pose() -> None:
    final_pose = PoseStamped()
    final_pose.header.frame_id = "world"
    final_pose.pose.position.x = 0.489322903785962
    final_pose.pose.position.y = -0.020938377004554136
    final_pose.pose.position.z = 0.30619623218684644
    final_pose.pose.orientation.x = -0.11241760487545066
    final_pose.pose.orientation.y = 0.6525535082516751
    final_pose.pose.orientation.z = 0.17339098968769825
    final_pose.pose.orientation.w = 0.7290211009824191

    waypoint = _elevated_place_waypoint(final_pose, 0.08)

    assert final_pose.pose.position.z == pytest.approx(0.30619623218684644)
    assert waypoint.header.frame_id == "world"
    assert waypoint.pose.position.x == pytest.approx(final_pose.pose.position.x)
    assert waypoint.pose.position.y == pytest.approx(final_pose.pose.position.y)
    assert waypoint.pose.position.z == pytest.approx(0.38619623218684643)
    assert waypoint.pose.orientation == final_pose.pose.orientation


class _NullLogger:
    def info(self, _message: str) -> None:
        pass


class _PoseHelper:
    def get_logger(self) -> _NullLogger:
        return _NullLogger()

    def _copy_pose(self, pose: PoseStamped) -> PoseStamped:
        return GraspBridgeStateMachine._copy_pose(self, pose)


def test_retracted_pose_moves_against_approach_axis() -> None:
    grasp_pose = PoseStamped()
    grasp_pose.pose.position.x = 0.50
    grasp_pose.pose.position.y = -0.20
    grasp_pose.pose.position.z = 0.40

    retreat = GraspBridgeStateMachine._retracted_pose_from_axis(
        _PoseHelper(),
        grasp_pose,
        np.asarray([0.0, 1.0, 0.0]),
        0.06,
        log_label="test_retreat",
        min_z_m=0.0,
    )

    assert retreat.pose.position.x == pytest.approx(0.50)
    assert retreat.pose.position.y == pytest.approx(-0.26)
    assert retreat.pose.position.z == pytest.approx(0.40)


def test_right_arm_config_uses_six_centimeter_pregrasp_and_retreat() -> None:
    config_path = (
        Path(__file__).resolve().parents[1]
        / "config"
        / "grasp_bridge_state_machine.right.yaml"
    )
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    parameters = config["grasp_bridge_state_machine"]["ros__parameters"]

    assert parameters["pre_grasp_offset_m"] == pytest.approx(0.06)
    assert parameters["retreat_after_grasp"] is False
    assert parameters["retreat_after_grasp_offset_m"] == pytest.approx(0.06)
    assert parameters["lift_after_grasp"] is True
    assert parameters["lift_offset_m"] == pytest.approx(0.03)
    assert parameters["place_lift_before_place"] is False
    assert parameters["place_lift_height_m"] == pytest.approx(0.0)
    assert parameters["place_descend_offset_m"] == pytest.approx(0.08)
    assert parameters["planning_retry_attempts"] == 2
    assert parameters["joint_plan_service_name"] == "/plan_to_joints"
    assert parameters["right_capture_joint_enabled"] is True
    assert parameters["right_capture_joint_positions"] == pytest.approx([
        1.096756612,
        0.376842732,
        -0.773397184,
        -0.213252900,
        0.928927888,
        0.106338624,
    ])

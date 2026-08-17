from pathlib import Path

import numpy as np
import pytest
import yaml

from geometry_msgs.msg import PoseStamped

from grasp_bridge_state_machine import (
    GraspBridgeStateMachine,
    _PLACE_TRANSPORT_STEP_ORDER,
    _bridge_task_type_for_step,
    _build_ik_orientation_candidates,
    _candidate_indexes_for_step,
    _capture_joint_target_for_profile,
    _elevated_place_waypoint,
    _is_no_ik_failure,
    _matrix_from_pose,
    _pose_with_local_orientation_offset,
    _return_home_step,
    _validated_ik_fallback_degrees,
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


def _arbitrary_grasp_pose() -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = "world"
    pose.pose.position.x = 0.41
    pose.pose.position.y = -0.17
    pose.pose.position.z = 0.36
    quaternion = np.asarray([0.2, -0.3, 0.1, 0.9], dtype=np.float64)
    quaternion /= np.linalg.norm(quaternion)
    pose.pose.orientation.x = float(quaternion[0])
    pose.pose.orientation.y = float(quaternion[1])
    pose.pose.orientation.z = float(quaternion[2])
    pose.pose.orientation.w = float(quaternion[3])
    return pose


def test_local_y_fallback_keeps_opening_axis_and_changes_approach_axis() -> None:
    original = _arbitrary_grasp_pose()
    rotated = _pose_with_local_orientation_offset(original, "y", 5.0)
    original_matrix = _matrix_from_pose(original)
    rotated_matrix = _matrix_from_pose(rotated)

    assert rotated_matrix[:3, 3] == pytest.approx(original_matrix[:3, 3])
    assert rotated_matrix[:3, 1] == pytest.approx(original_matrix[:3, 1])
    assert float(np.dot(rotated_matrix[:3, 2], original_matrix[:3, 2])) == pytest.approx(
        np.cos(np.deg2rad(5.0))
    )


def test_local_z_fallback_rotates_opening_axis_with_full_tcp_pose() -> None:
    original = _arbitrary_grasp_pose()
    rotated = _pose_with_local_orientation_offset(original, "z", -5.0)
    original_matrix = _matrix_from_pose(original)
    rotated_matrix = _matrix_from_pose(rotated)

    assert rotated_matrix[:3, 3] == pytest.approx(original_matrix[:3, 3])
    assert rotated_matrix[:3, 2] == pytest.approx(original_matrix[:3, 2])
    assert float(np.dot(rotated_matrix[:3, 1], original_matrix[:3, 1])) == pytest.approx(
        np.cos(np.deg2rad(5.0))
    )
    assert not np.allclose(rotated_matrix[:3, 1], original_matrix[:3, 1])
    rotated_quaternion = np.asarray(
        [
            rotated.pose.orientation.x,
            rotated.pose.orientation.y,
            rotated.pose.orientation.z,
            rotated.pose.orientation.w,
        ]
    )
    assert np.linalg.norm(rotated_quaternion) == pytest.approx(1.0)


def test_ik_fallback_candidate_order_is_original_then_y_then_z() -> None:
    candidates = _build_ik_orientation_candidates(
        _arbitrary_grasp_pose(),
        [3.0, -3.0, 5.0, -5.0, 8.0, -8.0],
        [3.0, -3.0, 5.0, -5.0],
    )

    assert [candidate.name for candidate in candidates] == [
        "original",
        "local_y_+3deg",
        "local_y_-3deg",
        "local_y_+5deg",
        "local_y_-5deg",
        "local_y_+8deg",
        "local_y_-8deg",
        "local_z_+3deg",
        "local_z_-3deg",
        "local_z_+5deg",
        "local_z_-5deg",
    ]


def test_grasp_candidate_lock_is_reused_by_coupled_steps() -> None:
    assert _candidate_indexes_for_step("PRE_GRASP", 11, None) == tuple(range(11))
    assert _candidate_indexes_for_step("GRASP", 11, 4) == (4,)
    assert _candidate_indexes_for_step("RETREAT_AFTER_GRASP", 11, 4) == (4,)
    assert _candidate_indexes_for_step("PLACE", 1, 4) == (0,)


def test_only_explicit_no_ik_detail_matches_ik_fallback_condition() -> None:
    assert _is_no_ik_failure("Target pose has no IK solution")
    assert _is_no_ik_failure("PLANNING FAILED: target pose has NO IK SOLUTION")
    assert not _is_no_ik_failure("Planning failed due to collision")
    assert not _is_no_ik_failure("Trajectory execution failed")


def test_ik_fallback_angles_are_bounded_and_deduplicated() -> None:
    assert _validated_ik_fallback_degrees(
        [3.0, -3.0, 3.0],
        "angles",
    ) == (3.0, -3.0)
    with pytest.raises(ValueError, match="must not exceed"):
        _validated_ik_fallback_degrees([16.0], "angles")


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
    assert parameters["grasp_ik_fallback_enabled"] is True
    assert parameters["grasp_ik_fallback_profiles"] == ["grasp"]
    assert parameters["grasp_ik_fallback_local_y_degrees"] == pytest.approx(
        [3.0, -3.0, 5.0, -5.0, 8.0, -8.0]
    )
    assert parameters["grasp_ik_fallback_local_z_degrees"] == pytest.approx(
        [3.0, -3.0, 5.0, -5.0]
    )

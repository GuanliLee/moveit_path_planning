from pathlib import Path

import numpy as np
import pytest
import yaml

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

from grasp_bridge_state_machine import (
    GraspBridgeStateMachine,
    _PLACE_ORIENTATION_TARGET_STEPS,
    _PLACE_TRANSPORT_STEP_ORDER,
    _arm_joint_positions_from_state,
    _bridge_task_type_for_step,
    _build_ik_orientation_candidates,
    _candidate_indexes_for_step,
    _capture_joint_target_for_profile,
    _elevated_place_waypoint,
    _grasp_approach_vertical_deviation_degrees,
    _is_no_ik_failure,
    _matrix_from_pose,
    _normalized_target_key,
    _object_upright_tilt_degrees,
    _place_completion_steps,
    _place_pose_with_grasp_orientation_compensation,
    _place_pose_with_upright_axis_compensation,
    _pose_with_local_orientation_offset,
    _return_home_step,
    _rotation_aligning_vectors,
    _upright_target_is_selected,
    _upright_object_axis_in_tcp,
    _validated_ik_fallback_degrees,
    _world_negative_x_retreat_pose,
)


@pytest.mark.parametrize(
    "step",
    (
        "RETREAT_AFTER_GRASP",
        "PLACE_WAYPOINT",
        "PLACE_XY",
        "PLACE_ORIENT",
        "PLACE_APPROACH",
    ),
)
def test_place_transport_steps_keep_attached_grasp_geometry(step: str) -> None:
    assert _bridge_task_type_for_step(step) == "CARRY"


def test_place_transport_orients_before_horizontal_motion() -> None:
    assert _PLACE_TRANSPORT_STEP_ORDER == (
        "PLACE_LIFT",
        "PLACE_ORIENT",
        "PLACE_XY",
    )


def test_all_explicit_place_orientation_targets_receive_grasp_delta() -> None:
    assert _PLACE_ORIENTATION_TARGET_STEPS == {
        "PLACE_ORIENT",
        "PLACE_WAYPOINT",
        "PLACE_APPROACH",
        "PLACE",
    }


@pytest.mark.parametrize(
    ("step", "expected"),
    (
        ("OPEN", "MOVE"),
        ("PRE_GRASP", "MOVE"),
        ("GRASP_APPROACH", "MOVE"),
        ("RETREAT_AFTER_PLACE", "MOVE"),
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


def test_place_completion_retreats_before_returning_to_photo_position() -> None:
    final_place_pose = PoseStamped()
    final_place_pose.header.frame_id = "world"
    photo_pose = PoseStamped()
    photo_pose.header.frame_id = "world"

    steps = _place_completion_steps(
        final_place_pose,
        0.08,
        verify_upright_before_release=True,
        retreat_after_place=True,
        return_home_pose=photo_pose,
    )

    assert [step[0] for step in steps] == [
        "PLACE_APPROACH",
        "VERIFY_PLACE_UPRIGHT",
        "PLACE",
        "RETREAT_AFTER_PLACE",
        "RETURN_HOME",
    ]
    assert steps[0] == ("PLACE_APPROACH", final_place_pose, False, 0.0)
    assert steps[1] == ("VERIFY_PLACE_UPRIGHT", None, False, 0.0)
    assert steps[2] == ("PLACE", final_place_pose, True, 0.08)
    assert steps[3] == ("RETREAT_AFTER_PLACE", None, False, 0.0)
    assert steps[4] == ("RETURN_HOME", photo_pose, False, 0.0)


def test_place_completion_can_preserve_legacy_direct_release() -> None:
    final_place_pose = PoseStamped()

    steps = _place_completion_steps(
        final_place_pose,
        0.08,
        verify_upright_before_release=False,
        retreat_after_place=False,
        return_home_pose=None,
    )

    assert steps == [("PLACE", final_place_pose, True, 0.08)]


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


def test_guarded_release_extracts_live_six_axis_joint_target() -> None:
    joint_state = JointState()
    joint_state.name = [
        "right_joint4",
        "left_joint1",
        "right_joint1",
        "right_joint6",
        "right_joint2",
        "right_joint5",
        "right_joint3",
        "right_joint7",
    ]
    joint_state.position = [0.4, -1.0, 0.1, 0.6, 0.2, 0.5, 0.3, 0.08]

    assert _arm_joint_positions_from_state(joint_state, "right") == pytest.approx(
        (0.1, 0.2, 0.3, 0.4, 0.5, 0.6)
    )


def test_guarded_release_rejects_incomplete_joint_state() -> None:
    joint_state = JointState()
    joint_state.name = ["right_joint1"]
    joint_state.position = [0.1]

    with pytest.raises(RuntimeError, match="joint_state_missing:right_joint2"):
        _arm_joint_positions_from_state(joint_state, "right")


def test_restock_final_pose_is_raised_two_centimeters_before_waypoint() -> None:
    nominal_pose = PoseStamped()
    nominal_pose.header.frame_id = "world"
    nominal_pose.pose.position.x = 0.489322903785962
    nominal_pose.pose.position.y = -0.020938377004554136
    nominal_pose.pose.position.z = 0.30619623218684644
    nominal_pose.pose.orientation.x = -0.11241760487545066
    nominal_pose.pose.orientation.y = 0.6525535082516751
    nominal_pose.pose.orientation.z = 0.17339098968769825
    nominal_pose.pose.orientation.w = 0.7290211009824191

    final_pose = _elevated_place_waypoint(nominal_pose, 0.02)
    waypoint = _elevated_place_waypoint(final_pose, 0.08)

    assert nominal_pose.pose.position.z == pytest.approx(0.30619623218684644)
    assert final_pose.pose.position.z == pytest.approx(0.32619623218684644)
    assert waypoint.header.frame_id == "world"
    assert waypoint.pose.position.x == pytest.approx(final_pose.pose.position.x)
    assert waypoint.pose.position.y == pytest.approx(final_pose.pose.position.y)
    assert waypoint.pose.position.z == pytest.approx(0.40619623218684643)
    assert waypoint.pose.orientation == final_pose.pose.orientation


def test_post_place_retreat_moves_eight_centimeters_along_world_negative_x() -> None:
    current_pose = PoseStamped()
    current_pose.header.frame_id = "world"
    current_pose.pose.position.x = 0.520677872605286
    current_pose.pose.position.y = 0.076696553529590
    current_pose.pose.position.z = 0.301053788167778
    current_pose.pose.orientation.x = 0.016259139819811
    current_pose.pose.orientation.y = 0.725175006149608
    current_pose.pose.orientation.z = -0.090586573306525
    current_pose.pose.orientation.w = 0.682386198252000

    retreat = _world_negative_x_retreat_pose(current_pose, 0.08)

    assert current_pose.pose.position.x == pytest.approx(0.520677872605286)
    assert retreat.header.frame_id == "world"
    assert retreat.pose.position.x == pytest.approx(0.440677872605286)
    assert retreat.pose.position.y == pytest.approx(current_pose.pose.position.y)
    assert retreat.pose.position.z == pytest.approx(current_pose.pose.position.z)
    assert retreat.pose.orientation == current_pose.pose.orientation


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


@pytest.mark.parametrize(
    ("local_axis", "degrees"),
    (("y", 5.0), ("z", -3.0)),
)
def test_place_compensation_restores_nominal_object_transform(
    local_axis: str,
    degrees: float,
) -> None:
    original_grasp = _arbitrary_grasp_pose()
    selected_grasp = _pose_with_local_orientation_offset(
        original_grasp,
        local_axis,
        degrees,
    )
    nominal_place = _arbitrary_grasp_pose()
    nominal_place.pose.position.x = 0.52
    nominal_place.pose.position.y = 0.08
    nominal_place.pose.position.z = 0.30
    place_quaternion = np.asarray(
        [0.0163, 0.7252, -0.0906, 0.6824],
        dtype=np.float64,
    )
    place_quaternion /= np.linalg.norm(place_quaternion)
    nominal_place.pose.orientation.x = float(place_quaternion[0])
    nominal_place.pose.orientation.y = float(place_quaternion[1])
    nominal_place.pose.orientation.z = float(place_quaternion[2])
    nominal_place.pose.orientation.w = float(place_quaternion[3])

    compensated_place = _place_pose_with_grasp_orientation_compensation(
        nominal_place,
        original_grasp,
        selected_grasp,
    )

    object_at_grasp = np.eye(4, dtype=np.float64)
    object_at_grasp[:3, 3] = np.asarray([0.31, -0.02, 0.08])
    nominal_object_at_place = (
        _matrix_from_pose(nominal_place)
        @ np.linalg.inv(_matrix_from_pose(original_grasp))
        @ object_at_grasp
    )
    compensated_object_at_place = (
        _matrix_from_pose(compensated_place)
        @ np.linalg.inv(_matrix_from_pose(selected_grasp))
        @ object_at_grasp
    )

    assert compensated_place.pose.position == nominal_place.pose.position
    assert compensated_object_at_place == pytest.approx(nominal_object_at_place)


def _pose_with_quaternion(quaternion: tuple[float, float, float, float]) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = "world"
    pose.pose.position.x = 0.52
    pose.pose.position.y = 0.08
    pose.pose.position.z = 0.30
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]
    return pose


def test_upright_compensation_corrects_latest_original_grasp_case() -> None:
    actual_grasp_tcp = _pose_with_quaternion(
        (-0.3493, 0.7510, 0.2134, 0.5180)
    )
    nominal_place = _pose_with_quaternion(
        (-0.1124, 0.6526, 0.1734, 0.7290)
    )
    object_axis_tcp = _upright_object_axis_in_tcp(actual_grasp_tcp)

    compensated_place, correction_degrees = (
        _place_pose_with_upright_axis_compensation(
            nominal_place,
            object_axis_tcp,
        )
    )

    assert _object_upright_tilt_degrees(
        nominal_place,
        object_axis_tcp,
    ) == pytest.approx(29.54, abs=0.05)
    assert correction_degrees == pytest.approx(29.54, abs=0.05)
    assert _object_upright_tilt_degrees(
        compensated_place,
        object_axis_tcp,
    ) == pytest.approx(0.0, abs=1e-6)
    assert compensated_place.pose.position == nominal_place.pose.position

    nominal_rotation = _matrix_from_pose(nominal_place)[:3, :3]
    compensated_rotation = _matrix_from_pose(compensated_place)[:3, :3]
    world_correction = compensated_rotation @ nominal_rotation.T
    # The full basis is transformed, including the TCP opening direction Y.
    assert compensated_rotation[:, 1] == pytest.approx(
        world_correction @ nominal_rotation[:, 1]
    )
    assert not np.allclose(compensated_rotation[:, 1], nominal_rotation[:, 1])


@pytest.mark.parametrize(
    ("local_axis", "degrees"),
    (("y", 0.0), ("y", 5.0), ("z", -5.0)),
)
def test_upright_compensation_levels_original_and_fallback_grasps(
    local_axis: str,
    degrees: float,
) -> None:
    actual_grasp_tcp = _arbitrary_grasp_pose()
    if degrees:
        actual_grasp_tcp = _pose_with_local_orientation_offset(
            actual_grasp_tcp,
            local_axis,
            degrees,
        )
    nominal_place = _pose_with_quaternion(
        (0.0163, 0.7252, -0.0906, 0.6824)
    )
    object_axis_tcp = _upright_object_axis_in_tcp(actual_grasp_tcp)

    compensated_place, _ = _place_pose_with_upright_axis_compensation(
        nominal_place,
        object_axis_tcp,
    )

    assert _object_upright_tilt_degrees(
        compensated_place,
        object_axis_tcp,
    ) == pytest.approx(0.0, abs=1e-6)


def test_vector_alignment_handles_opposite_vertical_axes() -> None:
    rotation = _rotation_aligning_vectors([0.0, 0.0, -1.0], [0.0, 0.0, 1.0])

    assert rotation @ np.asarray([0.0, 0.0, -1.0]) == pytest.approx(
        [0.0, 0.0, 1.0]
    )
    assert rotation @ rotation.T == pytest.approx(np.eye(3))
    assert np.linalg.det(rotation) == pytest.approx(1.0)


@pytest.mark.parametrize(
    ("approach_axis", "expected_deviation_deg"),
    (
        ((1.0, 0.0, 0.0), 0.0),
        ((1.0, 0.0, -1.0), 45.0),
        ((0.0, 0.0, -1.0), 90.0),
    ),
)
def test_side_grasp_deviation_is_measured_from_the_horizontal_plane(
    approach_axis: tuple[float, float, float],
    expected_deviation_deg: float,
) -> None:
    assert _grasp_approach_vertical_deviation_degrees(
        approach_axis
    ) == pytest.approx(expected_deviation_deg)


def test_upright_target_wildcard_selects_all_current_and_future_products() -> None:
    selected = {_normalized_target_key("*")}

    assert _upright_target_is_selected("AD Calcium Milk", selected)
    assert _upright_target_is_selected("Aojiru", selected)
    assert _upright_target_is_selected("Future Upright Product", selected)


def test_explicit_upright_target_selection_remains_case_and_space_insensitive() -> None:
    selected = {_normalized_target_key("HK Orange Fanta")}

    assert _upright_target_is_selected("  HK   ORANGE fanta ", selected)
    assert not _upright_target_is_selected("AD Calcium Milk", selected)


def test_upright_place_compensation_is_enabled_for_right_grasp_flow() -> None:
    config_path = (
        Path(__file__).resolve().parents[1]
        / "config"
        / "grasp_bridge_state_machine.right.yaml"
    )
    parameters = yaml.safe_load(config_path.read_text(encoding="utf-8"))[
        "grasp_bridge_state_machine"
    ]["ros__parameters"]

    assert parameters["compensate_place_orientation_from_grasp_delta"] is True
    assert parameters["place_upright_axis_compensation_enabled"] is True
    assert parameters["place_upright_axis_compensation_profiles"] == ["grasp"]
    assert parameters["place_upright_axis_compensation_arms"] == ["left", "right"]
    assert parameters["place_upright_axis_compensation_targets"] == ["*"]
    assert parameters[
        "place_upright_side_grasp_max_vertical_deviation_deg"
    ] == pytest.approx(45.0)
    assert parameters["place_upright_tilt_limit_deg"] == pytest.approx(3.5)


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
    assert parameters["restock_place_final_z_offset_m"] == pytest.approx(0.02)
    assert parameters["place_descend_offset_m"] == pytest.approx(0.08)
    assert parameters["retreat_after_place"] is True
    assert parameters["retreat_after_place_offset_m"] == pytest.approx(0.08)
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

import ast
from pathlib import Path

import numpy as np
import pytest
import yaml

from handeye_transform import (
    compose_capture_frame_extrinsics,
    compose_world_from_handeye_camera,
    transform_pose_delta,
    world_from_link6_offset,
)


PACKAGE_ROOT = Path(__file__).parents[1]
STATE_MACHINE = PACKAGE_ROOT / "scripts" / "grasp_bridge_state_machine.py"
CONFIG = PACKAGE_ROOT / "config" / "grasp_bridge_state_machine.right.yaml"


def _translation(x: float, y: float, z: float) -> np.ndarray:
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, 3] = (x, y, z)
    return matrix


def _function_source(function_name: str) -> str:
    tree = ast.parse(STATE_MACHINE.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name == function_name:
            return ast.unparse(node)
    raise AssertionError(f"function not found: {function_name}")


def test_handeye_chain_uses_calibration_parent_not_planning_tcp() -> None:
    world_from_link6 = np.asarray(
        [
            [0.0, -1.0, 0.0, 0.40],
            [1.0, 0.0, 0.0, -0.20],
            [0.0, 0.0, 1.0, 0.50],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    handeye_parent_from_camera = _translation(-0.078, 0.005, -0.032)

    actual = compose_world_from_handeye_camera(
        world_from_link6,
        handeye_parent_from_camera,
        0.0593,
    )
    expected = world_from_link6 @ _translation(0.0, 0.0, 0.0593) @ handeye_parent_from_camera
    wrong_planning_tcp_chain = (
        world_from_link6 @ _translation(0.0, 0.0, 0.1358) @ handeye_parent_from_camera
    )

    np.testing.assert_allclose(actual, expected, atol=1e-12)
    np.testing.assert_allclose(
        np.linalg.norm(actual[:3, 3] - wrong_planning_tcp_chain[:3, 3]),
        0.0765,
        atol=1e-12,
    )


def test_link6_offset_is_applied_in_link6_local_axis() -> None:
    world_from_link6 = np.asarray(
        [
            [0.0, 0.0, 1.0, 1.0],
            [0.0, 1.0, 0.0, 2.0],
            [-1.0, 0.0, 0.0, 3.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )

    result = world_from_link6_offset(world_from_link6, 0.0593)

    np.testing.assert_allclose(result[:3, 3], [1.0593, 2.0, 3.0], atol=1e-12)


def test_capture_extrinsics_split_world_and_arm_base_frames() -> None:
    world_from_link6 = np.asarray(
        [
            [0.0, -1.0, 0.0, 0.40],
            [1.0, 0.0, 0.0, -0.20],
            [0.0, 0.0, 1.0, 0.50],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    arm_base_from_world = np.asarray(
        [
            [1.0, 0.0, 0.0, 0.10],
            [0.0, 0.0, -1.0, 0.30],
            [0.0, 1.0, 0.0, -0.05],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    handeye_parent_from_camera = np.asarray(
        [
            [0.0, 0.0, 1.0, -0.078],
            [1.0, 0.0, 0.0, 0.005],
            [0.0, 1.0, 0.0, -0.032],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )

    result = compose_capture_frame_extrinsics(
        world_from_link6,
        arm_base_from_world,
        handeye_parent_from_camera,
        0.0593,
        0.1358,
    )
    expected_world_parent = world_from_link6 @ _translation(0.0, 0.0, 0.0593)
    expected_world_planning_tcp = world_from_link6 @ _translation(0.0, 0.0, 0.1358)
    expected_world_camera = expected_world_parent @ handeye_parent_from_camera

    np.testing.assert_allclose(
        result["world_from_handeye_parent"], expected_world_parent, atol=1e-12
    )
    np.testing.assert_allclose(
        result["world_from_camera"], expected_world_camera, atol=1e-12
    )
    np.testing.assert_allclose(
        result["world_from_planning_tcp"], expected_world_planning_tcp, atol=1e-12
    )
    np.testing.assert_allclose(
        result["base_from_handeye_parent"],
        arm_base_from_world @ expected_world_parent,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        result["base_from_camera"],
        arm_base_from_world @ expected_world_camera,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        result["base_from_planning_tcp"],
        arm_base_from_world @ expected_world_planning_tcp,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        result["planning_tcp_from_camera"],
        np.linalg.inv(expected_world_planning_tcp) @ expected_world_camera,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        np.linalg.inv(result["base_from_handeye_parent"])
        @ result["base_from_camera"],
        handeye_parent_from_camera,
        atol=1e-12,
    )


@pytest.mark.parametrize(
    ("base_from_world_y", "expected_delta_y"),
    ((0.3, 0.3), (-0.3, -0.3)),
)
def test_arm_base_capture_values_apply_world_to_base_offset(
    base_from_world_y: float,
    expected_delta_y: float,
) -> None:
    world_from_link6 = _translation(0.039, -0.263, 0.3087)
    arm_base_from_world = _translation(0.0, base_from_world_y, 0.0)
    handeye_parent_from_camera = _translation(-0.007, -0.025, 0.081)

    result = compose_capture_frame_extrinsics(
        world_from_link6,
        arm_base_from_world,
        handeye_parent_from_camera,
        0.0593,
        0.1358,
    )

    assert result["base_from_handeye_parent"][1, 3] == pytest.approx(
        result["world_from_handeye_parent"][1, 3] + expected_delta_y
    )
    assert result["base_from_camera"][1, 3] == pytest.approx(
        result["world_from_camera"][1, 3] + expected_delta_y
    )


def test_camera_goal_world_round_trip_does_not_add_arm_base_offset() -> None:
    world_from_link6 = _translation(0.039, -0.263, 0.3087)
    arm_base_from_world = _translation(0.0, 0.3, 0.0)
    handeye_parent_from_camera = _translation(-0.007, -0.025, 0.081)
    camera_from_goal = _translation(0.053, 0.113, 0.417)
    result = compose_capture_frame_extrinsics(
        world_from_link6,
        arm_base_from_world,
        handeye_parent_from_camera,
        0.0593,
        0.1358,
    )

    world_from_goal = result["world_from_camera"] @ camera_from_goal
    base_from_goal = result["base_from_camera"] @ camera_from_goal

    np.testing.assert_allclose(
        np.linalg.inv(arm_base_from_world) @ base_from_goal,
        world_from_goal,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        np.linalg.inv(result["base_from_camera"]) @ base_from_goal,
        np.linalg.inv(result["world_from_camera"]) @ world_from_goal,
        atol=1e-12,
    )


def test_transform_pose_delta_reports_translation_and_rotation() -> None:
    first = np.eye(4, dtype=np.float64)
    second = np.asarray(
        [
            [0.0, -1.0, 0.0, 0.001],
            [1.0, 0.0, 0.0, 0.002],
            [0.0, 0.0, 1.0, 0.002],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )

    translation_m, rotation_deg = transform_pose_delta(first, second)

    assert translation_m == pytest.approx(0.003)
    assert rotation_deg == pytest.approx(90.0)


def test_runtime_parameters_keep_handeye_and_planning_tcp_separate() -> None:
    parameters = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))[
        "grasp_bridge_state_machine"
    ]["ros__parameters"]

    assert parameters["link6_to_handeye_parent_m"] == 0.0593
    assert parameters["tcp_to_link6_m"] == 0.1358


def test_capture_sync_configuration_is_fail_closed_after_settle() -> None:
    parameters = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))[
        "grasp_bridge_state_machine"
    ]["ros__parameters"]
    execution_source = _function_source("_execute_named")
    capture_source = _function_source("_capture_sensor_snapshot")
    dynamic_source = _function_source("_dynamic_extrinsics_for_121")

    assert parameters["sensor_require_post_open_frames"] is True
    assert parameters["sensor_discard_pairs_after_barrier"] == 2
    assert parameters["sensor_max_stamp_skew_s"] == 0.005
    assert parameters["sensor_max_arrival_skew_s"] == 0.15
    assert parameters["sensor_max_arrival_age_s"] == 0.15
    assert parameters["capture_stationary_translation_m"] == 0.002
    assert parameters["capture_stationary_rotation_deg"] == 1.0
    assert parameters["capture_tf_max_age_s"] == 0.20
    assert parameters["capture_tf_advance_timeout_s"] == 0.50
    assert parameters["sensor_discard_pairs_after_barrier"] < parameters["sensor_queue_size"]
    assert execution_source.index("self._sleep_interruptibly(open_settle_s)") < execution_source.index(
        "self._capture_sensor_snapshot"
    )
    assert "barrier_monotonic_s = time.monotonic()" in capture_source
    assert "transform_pose_delta" in capture_source
    assert "after_tf_stamp_s > before_tf_stamp_s" in capture_source
    assert "tf_advance_deadline" in capture_source
    assert "latest_receipt_ros_s - 0.05" in capture_source
    assert "capture_stamp" not in dynamic_source


def test_image_payload_uses_imported_stamp_helper() -> None:
    tree = ast.parse(STATE_MACHINE.read_text(encoding="utf-8"))
    imports = {
        alias.name
        for node in tree.body
        if isinstance(node, ast.ImportFrom) and node.module == "capture_sync"
        for alias in node.names
    }
    image_payload_source = _function_source("_image_payload")

    assert "message_stamp_seconds" in imports
    assert "message_stamp_seconds(msg)" in image_payload_source
    assert "_message_stamp_seconds" not in image_payload_source


def test_camera_paths_cannot_reuse_planning_tcp_parameter() -> None:
    for function_name in (
        "_world_camera_from_link6",
        "_offset_camera_grasp_z_in_world",
        "_camera_tcp_pose_to_world_pose",
    ):
        source = _function_source(function_name)
        assert "_tcp_to_link6_m" not in source

    assert "_link6_to_handeye_parent_m" in _function_source("_world_camera_from_link6")
    capture_source = _function_source("_dynamic_extrinsics_for_121")
    assert "_link6_to_handeye_parent_m" in capture_source
    assert "_tcp_to_link6_m" in capture_source


def test_121_state_pose_matches_handeye_parent() -> None:
    payload_source = _function_source("_build_inference_payload")
    planning_tcp_source = _function_source("_current_tcp_pose_world")

    assert 'extrinsics[\'base_from_handeye_parent\']' in payload_source
    assert 'extrinsics[\'world_from_handeye_parent\']' in payload_source
    assert 'extrinsics[\'base_from_planning_tcp\']' not in payload_source
    assert 'extrinsics[\'world_from_planning_tcp\']' not in payload_source
    assert "_current_handeye_parent_pose_world" not in payload_source
    assert "_current_tcp_pose_world" not in payload_source
    assert "_tcp_to_link6_m" in planning_tcp_source


def test_121_base_fields_and_camera_return_use_distinct_frames() -> None:
    payload_source = _function_source("_build_inference_payload")
    capture_source = _function_source("_dynamic_extrinsics_for_121")
    execution_source = _function_source("_execute_named")

    assert 'extrinsics[\'base_from_camera\']' in payload_source
    assert "capture['base_from_camera']" in capture_source
    assert "capture['world_from_camera']" in capture_source
    assert 'capture_extrinsics[\'world_from_camera\']' in execution_source
    assert 'capture_extrinsics[\'base_from_camera\']' not in execution_source


def test_remote_gripper_end_pose_is_forwarded_without_rebase() -> None:
    execution_source = _function_source("_execute_named")

    assert "_handeye_parent_pose_to_planning_tcp" not in execution_source
    assert "rebase_link6_control_point_pose" not in execution_source
    assert "planning_grasp_tcp_from_121" in execution_source


def test_grasp_uses_candidate_axis_pregrasp_before_final_target() -> None:
    parameters = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))[
        "grasp_bridge_state_machine"
    ]["ros__parameters"]
    execution_source = _function_source("_execute_named")

    assert parameters["pre_grasp_before_grasp"] is True
    assert parameters["pre_grasp_offset_m"] == 0.06
    assert "_retracted_pose_from_axis" in execution_source
    assert "grasp_approach_axes_world" in execution_source
    assert execution_source.index('steps.append((\'PRE_GRASP\'') < execution_source.index(
        'steps.append((\'GRASP\''
    )


def test_pre_close_snapshot_uses_same_synchronized_capture_path_when_enabled() -> None:
    parameters = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))[
        "grasp_bridge_state_machine"
    ]["ros__parameters"]
    execution_source = _function_source("_execute_named")

    assert isinstance(parameters["pre_close_snapshot_enabled"], bool)
    assert parameters["pre_close_snapshot_settle_s"] == 0.3
    assert execution_source.index('steps.append((\'GRASP_APPROACH\'') < execution_source.index(
        'steps.append((\'GRASP\''
    )
    assert 'task_type == \'GRASP_APPROACH\'' in execution_source
    assert '_save_pre_close_snapshot' in execution_source
    assert "self._capture_sensor_snapshot" in _function_source("_save_pre_close_snapshot")


def test_live_rviz_target_uses_planning_tcp_and_inverse_link6_offset() -> None:
    marker_source = _function_source("_build_target_marker_array")
    execution_source = _function_source("_execute_named")
    parameters = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))[
        "grasp_bridge_state_machine"
    ]["ros__parameters"]

    assert "link6_from_tcp[2, 3] = -float(tcp_to_link6_m)" in marker_source
    assert "target_matrix @ link6_from_tcp" in marker_source
    assert parameters["right_target_marker_topic"] == (
        "/rviz/right/planning_tcp_target_markers"
    )
    assert execution_source.index("pre_grasp_poses = []") < execution_source.index(
        "self._publish_target_markers"
    )
    assert "'pending'" in execution_source
    assert "'failed'" in execution_source
    assert "'success'" in execution_source

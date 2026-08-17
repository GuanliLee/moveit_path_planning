from pathlib import Path

import yaml


SERVICE_FILE = (
    Path(__file__).parents[2]
    / 'path_planning_interfaces'
    / 'srv'
    / 'PlanToPose.srv'
)
JOINT_SERVICE_FILE = (
    Path(__file__).parents[2]
    / "path_planning_interfaces"
    / "srv"
    / "PlanToJoints.srv"
)
SERVER_SOURCE = Path(__file__).parents[1] / 'src' / 'path_planning_node.cpp'
PLANNER_CONFIG = Path(__file__).parents[1] / 'config' / 'planner.yaml'


def service_fields(service_file=SERVICE_FILE):
    request, response = service_file.read_text(encoding='utf-8').split('---')

    def fields(section):
        return [
            line.strip()
            for line in section.splitlines()
            if line.strip() and not line.lstrip().startswith('#')
        ]

    return fields(request), fields(response)


def test_plan_to_pose_exposes_optional_gripper_command():
    request, response = service_fields()

    assert request == [
        'string arm_name',
        'uint32 scene_id',
        'geometry_msgs/PoseStamped target_pose',
        'bool gripper_command',
        'float64 gripper_opening_m',
        'bool keep_grasp_ellipsoid',
    ]
    assert response == [
        'bool success',
        'string message',
        'moveit_msgs/RobotTrajectory trajectory',
    ]


def test_plan_to_joints_exposes_exact_six_axis_target():
    request, response = service_fields(JOINT_SERVICE_FILE)

    assert request == [
        "string arm_name",
        "uint32 scene_id",
        "float64[6] joint_positions",
        "bool gripper_command",
        "float64 gripper_opening_m",
        "bool keep_grasp_ellipsoid",
    ]
    assert response == [
        "bool success",
        "string message",
        "moveit_msgs/RobotTrajectory trajectory",
    ]

    source = SERVER_SOURCE.read_text(encoding="utf-8")
    assert 'create_service<PlanToJoints>(' in source
    assert '"/plan_to_joints"' in source
    assert "setJointValueTarget(target_joints)" in source


def test_planning_start_state_preserves_attached_collision_objects():
    source = SERVER_SOURCE.read_text(encoding='utf-8')

    assert 'move_group->setStartStateToCurrentState();' in source
    assert 'move_group->setStartState(*current_state);' not in source


def test_final_place_release_uses_dedicated_slow_duration():
    source = SERVER_SOURCE.read_text(encoding='utf-8')
    config = yaml.safe_load(PLANNER_CONFIG.read_text(encoding='utf-8'))
    parameters = config['path_planning_server']['ros__parameters']

    assert parameters['gripper_move_duration'] == 1.0
    assert parameters['gripper_release_duration'] == 3.0
    assert 'release_grasp_ellipsoid_after_gripper ?' in source
    assert 'gripper_release_duration_ : gripper_move_duration_' in source

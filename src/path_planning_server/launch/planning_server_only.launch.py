"""Start only the path-planning service against an existing MoveIt stack."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    package_share = get_package_share_directory("path_planning_server")
    moveit_config = (
        MoveItConfigsBuilder(
            "agilex_dual_piper_rviz",
            package_name="path_planning_server",
        )
        .robot_description(file_path="generated/dual_piper.urdf")
        .robot_description_semantic(file_path="config/dual_piper.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl"],
            load_all=False,
        )
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml",
            moveit_manage_controllers=False,
        )
        .planning_scene_monitor(
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    scene_id = LaunchConfiguration("scene_id")
    execute_trajectory = LaunchConfiguration("execute_trajectory")

    planning_server = Node(
        package="path_planning_server",
        executable="path_planning_server_node",
        name="path_planning_server",
        output="screen",
        parameters=[
            f"{package_share}/config/planner.yaml",
            f"{package_share}/config/grasp_ellipsoid.yaml",
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {
                "default_scene_id": ParameterValue(scene_id, value_type=int),
                "execute_trajectory": ParameterValue(
                    execute_trajectory,
                    value_type=bool,
                ),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "scene_id",
            default_value="1",
            description="Static planning scene loaded by the service.",
        ),
        DeclareLaunchArgument(
            "execute_trajectory",
            default_value="true",
            description="Execute successful plans through Piper trajectory actions.",
        ),
        planning_server,
    ])

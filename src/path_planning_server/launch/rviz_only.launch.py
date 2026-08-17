"""Start only RViz against the already-running MoveIt stack."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
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

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="path_planning_rviz",
        output="screen",
        arguments=["-d", f"{package_share}/config/moveit.rviz"],
        parameters=[
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )
    return LaunchDescription([rviz])

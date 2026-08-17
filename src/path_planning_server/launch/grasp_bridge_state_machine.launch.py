from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("path_planning_server")
    config_file = LaunchConfiguration("config_file")

    state_machine = Node(
        package="path_planning_server",
        executable="grasp_bridge_state_machine",
        name="grasp_bridge_state_machine",
        output="screen",
        parameters=[config_file],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=f"{package_share}/config/grasp_bridge_state_machine.right.yaml",
            description="State machine parameter file.",
        ),
        state_machine,
    ])

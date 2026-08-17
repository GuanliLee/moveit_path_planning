from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("graspnet_path_planning_bridge")
    config_file = LaunchConfiguration("config_file")

    bridge = Node(
        package="graspnet_path_planning_bridge",
        executable="bridge_node",
        name="graspnet_path_planning_bridge",
        output="screen",
        parameters=[config_file],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=f"{package_share}/config/bridge.yaml",
            description="Bridge parameter file.",
        ),
        bridge,
    ])

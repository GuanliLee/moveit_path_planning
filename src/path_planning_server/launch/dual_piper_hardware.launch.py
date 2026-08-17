"""Run the dual-Piper planning service against two physical arms."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    planning_share = get_package_share_directory("path_planning_server")
    piper_share = get_package_share_directory("piper")

    can_left_port = LaunchConfiguration("can_left_port")
    can_right_port = LaunchConfiguration("can_right_port")
    auto_enable = LaunchConfiguration("auto_enable")
    gripper_val_mutiple = LaunchConfiguration("gripper_val_mutiple")
    scene_id = LaunchConfiguration("scene_id")
    use_rviz = LaunchConfiguration("use_rviz")

    drivers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(piper_share, "launch", "start_two_piper.launch.py")
        ),
        launch_arguments={
            "can_left_port": can_left_port,
            "can_right_port": can_right_port,
            "auto_enable": auto_enable,
            "gripper_exist": "true",
            "gripper_val_mutiple": gripper_val_mutiple,
        }.items(),
    )

    left_bridge = Node(
        package="piper_trajectory_bridge",
        executable="piper_trajectory_bridge",
        namespace="left",
        name="trajectory_bridge",
        output="screen",
        parameters=[{
            "has_gripper": True,
            "command_topic": "/joint_ctrl_cmd_left",
            "state_topic": "/joint_states_left",
            "joint_state_topic": "joint_states",
            "command_rate": 180.0,
            "hold_gripper_command": True,
            "gripper_hold_publish_rate": 0.0,
        }],
    )

    right_bridge = Node(
        package="piper_trajectory_bridge",
        executable="piper_trajectory_bridge",
        namespace="right",
        name="trajectory_bridge",
        output="screen",
        parameters=[{
            "has_gripper": True,
            "command_topic": "/joint_ctrl_cmd_right",
            "state_topic": "/joint_states_right",
            "joint_state_topic": "joint_states",
            "command_rate": 180.0,
            "hold_gripper_command": True,
            "gripper_hold_publish_rate": 0.0,
        }],
    )

    state_adapter = Node(
        package="path_planning_server",
        executable="dual_joint_state_adapter",
        name="dual_joint_state_adapter",
        output="screen",
    )

    planning = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                planning_share,
                "launch",
                "path_planning_service.launch.py",
            )
        ),
        launch_arguments={
            "scene_id": scene_id,
            "use_rviz": use_rviz,
            "use_virtual_joint_states": "false",
            "execute_trajectory": "true",
        }.items(),
    )

    handeye_camera_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                planning_share,
                "launch",
                "handeye_camera_tf.launch.py",
            )
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "can_left_port",
            default_value="can_left",
            description="SocketCAN interface connected to the left Piper.",
        ),
        DeclareLaunchArgument(
            "can_right_port",
            default_value="can_right",
            description="SocketCAN interface connected to the right Piper.",
        ),
        DeclareLaunchArgument(
            "auto_enable",
            default_value="true",
            description="Enable both Piper arms when their drivers start.",
        ),
        DeclareLaunchArgument(
            "gripper_val_mutiple",
            default_value="2",
            description="Gripper feedback/command angle multiplier.",
        ),
        DeclareLaunchArgument(
            "scene_id",
            default_value="1",
            description="Static planning scene loaded by the service.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Start RViz alongside the hardware service.",
        ),
        drivers,
        left_bridge,
        right_bridge,
        state_adapter,
        handeye_camera_tf,
        planning,
    ])

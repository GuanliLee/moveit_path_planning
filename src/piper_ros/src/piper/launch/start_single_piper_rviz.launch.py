import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    log_level = 'warn'
    # Get the path to the piper_description package
    piper_description_path = os.path.join(
        get_package_share_directory('piper_description'),
        'launch',
        'piper_with_gripper',
        'display_urdf_follow.launch.py'
    )

    # Define launch parameters
    can_port_arg = DeclareLaunchArgument(
        'can_port',
        default_value='can0',
        description='CAN port for the robot arm'
    )

    auto_enable_arg = DeclareLaunchArgument(
        'auto_enable',
        default_value='false',
        description='Enable robot arm automatically'
    )

    # Include display_xacro.launch.py
    display_xacro_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(piper_description_path)
    )

    gripper_exist_arg = DeclareLaunchArgument(
        'gripper_exist',
        default_value='true',
        description='Gripper existence flag'
    )

    gripper_val_mutiple_arg = DeclareLaunchArgument(
        'gripper_val_mutiple',
        default_value='2',
        description='Scale URDF finger travel to the Piper total opening.'
    )
    command_topic_arg = DeclareLaunchArgument(
        'command_topic',
        default_value='joint_ctrl_single',
        description='JointState command topic.'
    )

    # Define the robot arm node
    piper_ctrl_node = Node(
        package='piper',
        executable='piper_single_ctrl',
        name='piper_ctrl_single_node',
        output='screen',
        parameters=[
            {'can_port': LaunchConfiguration('can_port')},
            {'auto_enable': LaunchConfiguration('auto_enable')},
            {'gripper_val_mutiple': LaunchConfiguration('gripper_val_mutiple')},
            {'gripper_exist': LaunchConfiguration('gripper_exist')}
        ],
        ros_arguments=['--log-level', log_level],
        remappings=[
            ('joint_ctrl_single', LaunchConfiguration('command_topic')),
            ('joint_states_single', '/joint_states'),
        ]
    )

    # Return the LaunchDescription object containing all the above elements
    return LaunchDescription([
        can_port_arg,
        auto_enable_arg,
        display_xacro_launch,
        gripper_exist_arg,
        gripper_val_mutiple_arg,
        command_topic_arg,
        piper_ctrl_node
    ])

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

os.environ['RCUTILS_COLORIZED_OUTPUT'] = '1'


def generate_launch_description():
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal).'
    )
    # Declare the launch arguments
    can_port_arg = DeclareLaunchArgument(
        'can_port',
        default_value='can0',
        description='CAN port to be used by the Piper node.'
    )
    auto_enable_arg = DeclareLaunchArgument(
        'auto_enable',
        default_value='false',
        description='Automatically enable the Piper node.'
    )

    gripper_exist_arg = DeclareLaunchArgument(
        'gripper_exist',
        default_value='true',
        description='Whether the arm has a gripper.'
    )

    gripper_val_mutiple_arg = DeclareLaunchArgument(
        'gripper_val_mutiple',
        default_value='2',
        description='Scale URDF finger travel to the Piper total opening.'
    )
    command_topic_arg = DeclareLaunchArgument(
        'command_topic',
        default_value='joint_ctrl_single',
        description='JointState command topic. Never remap feedback /joint_states here.'
    )
    feedback_topic_arg = DeclareLaunchArgument(
        'feedback_topic',
        default_value='/joint_states',
        description='Measured joint-state topic published by the Piper driver.'
    )

    # Define the node
    piper_node = Node(
        package='piper',
        executable='piper_single_ctrl',
        name='piper_ctrl_single_node',
        output='screen',
        ros_arguments=['--log-level', LaunchConfiguration('log_level')],
        parameters=[{
            'can_port': LaunchConfiguration('can_port'),
            'auto_enable': LaunchConfiguration('auto_enable'),
            'gripper_val_mutiple': LaunchConfiguration('gripper_val_mutiple'),
            'gripper_exist': LaunchConfiguration('gripper_exist'),
        }],
        remappings=[
            ('joint_ctrl_single', LaunchConfiguration('command_topic')),
            ('joint_states_single', LaunchConfiguration('feedback_topic')),
        ]
    )

    # Return the LaunchDescription
    return LaunchDescription([
        log_level_arg,
        can_port_arg,
        auto_enable_arg,
        gripper_exist_arg,
        gripper_val_mutiple_arg,
        command_topic_arg,
        feedback_topic_arg,
        piper_node
    ])

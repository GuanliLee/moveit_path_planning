import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

os.environ['RCUTILS_COLORIZED_OUTPUT'] = '1'   # 强制彩色日志


def generate_launch_description():
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal).'
    )
    # Declare the launch arguments
    can_left_port_arg = DeclareLaunchArgument(
        'can_left_port',
        default_value='can0',
        description='CAN left port to be used by the Piper node.'
    )
    can_right_port_arg = DeclareLaunchArgument(
        'can_right_port',
        default_value='can1',
        description='CAN right port to be used by the Piper node.'
    )

    auto_enable_arg = DeclareLaunchArgument(
        'auto_enable',
        default_value='false',
        description='Automatically enable the Piper node.'
    )

    rviz_ctrl_flag_arg = DeclareLaunchArgument(
        'rviz_ctrl_flag',
        default_value='false',
        description='Start rviz flag.'
    )

    gripper_exist_arg = DeclareLaunchArgument(
        'gripper_exist',
        default_value='true',
        description='gripper'
    )

    gripper_val_mutiple_arg = DeclareLaunchArgument(
        'gripper_val_mutiple',
        default_value='1',
        description='gripper'
    )

    # Define the node
    piper_left_node = Node(
        package='piper',
        executable='piper_single_ctrl',
        name='piper_left_ctrl_node',
        output='screen',
        ros_arguments=['--log-level', LaunchConfiguration('log_level')],
        parameters=[{
            'can_port': ParameterValue(
                LaunchConfiguration('can_left_port'), value_type=str),
            'auto_enable': ParameterValue(
                LaunchConfiguration('auto_enable'), value_type=bool),
            'rviz_ctrl_flag': ParameterValue(
                LaunchConfiguration('rviz_ctrl_flag'), value_type=bool),
            'gripper_exist': ParameterValue(
                LaunchConfiguration('gripper_exist'), value_type=bool),
            'gripper_val_mutiple': ParameterValue(
                LaunchConfiguration('gripper_val_mutiple'), value_type=int),
        }],
        remappings=[
            ('enable_srv', '/left/enable_srv'),
            ('enable_flag', '/left/enable_flag'),
            # 控制
            ('pos_cmd', '/pos_cmd_left'),
            ('joint_ctrl_single', '/joint_ctrl_cmd_left'),
            # 反馈
            ('joint_states_single', '/joint_states_left'),
            ('joint_states_feedback', '/joint_left'),
            ('joint_ctrl', '/joint_states_ctrl_left'),
            ('arm_status', '/arm_status_left'),
            ('end_pose', '/end_pose_left'),
            ('end_pose_stamped', '/end_pose_stamped_left'),
        ]
    )

    piper_right_node = Node(
        package='piper',
        executable='piper_single_ctrl',
        name='piper_right_ctrl_node',
        output='screen',
        ros_arguments=['--log-level', LaunchConfiguration('log_level')],
        parameters=[{
            'can_port': ParameterValue(
                LaunchConfiguration('can_right_port'), value_type=str),
            'auto_enable': ParameterValue(
                LaunchConfiguration('auto_enable'), value_type=bool),
            'rviz_ctrl_flag': ParameterValue(
                LaunchConfiguration('rviz_ctrl_flag'), value_type=bool),
            'gripper_exist': ParameterValue(
                LaunchConfiguration('gripper_exist'), value_type=bool),
            'gripper_val_mutiple': ParameterValue(
                LaunchConfiguration('gripper_val_mutiple'), value_type=int),
        }],
        remappings=[
            ('enable_srv', '/right/enable_srv'),
            ('enable_flag', '/right/enable_flag'),
            # 控制
            ('pos_cmd', '/pos_cmd_right'),
            ('joint_ctrl_single', '/joint_ctrl_cmd_right'),
            # 反馈
            ('joint_states_single', '/joint_states_right'),
            ('joint_states_feedback', '/joint_right'),
            ('joint_ctrl', '/joint_states_ctrl_right'),
            ('arm_status', '/arm_status_right'),
            ('end_pose', '/end_pose_right'),
            ('end_pose_stamped', '/end_pose_stamped_right'),
        ]
    )

    # Return the LaunchDescription
    return LaunchDescription([
        log_level_arg,
        can_left_port_arg,
        can_right_port_arg,
        auto_enable_arg,
        rviz_ctrl_flag_arg,
        gripper_exist_arg,
        gripper_val_mutiple_arg,
        piper_left_node,
        piper_right_node,
    ])

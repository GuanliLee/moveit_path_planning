import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('data_tools')

    declared_arguments = [
        DeclareLaunchArgument('useService', default_value='false'),
        DeclareLaunchArgument('datasetDir', default_value='/home/agilex/data'),
        DeclareLaunchArgument('episodeIndex', default_value='0'),
        DeclareLaunchArgument('paramsFile', default_value=os.path.join(share_dir, 'config', 'aloha_mobile_data_params.yaml')),
        DeclareLaunchArgument('hz', default_value='20'),
        DeclareLaunchArgument('timeout', default_value='2'),
        DeclareLaunchArgument('useTopicStamp', default_value='false'),
    ]

    parameter_file = LaunchConfiguration('paramsFile')
    use_service = LaunchConfiguration('useService')
    dataset_dir = LaunchConfiguration('datasetDir')
    episode_index = LaunchConfiguration('episodeIndex')
    hz = LaunchConfiguration('hz')
    timeout = LaunchConfiguration('timeout')
    use_topic_stamp = LaunchConfiguration('useTopicStamp')

    return LaunchDescription(declared_arguments + [
        Node(
            package='data_tools',
            executable='record_mcap.py',
            name='data_capture_to_mcap',
            output='screen',
            parameters=[
                {
                    'paramsFile': parameter_file,
                    'useService': use_service,
                    'datasetDir': dataset_dir,
                    'episodeIndex': episode_index,
                    'hz': hz,
                    'timeout': timeout,
                    'useTopicStamp': use_topic_stamp,
                }
            ],
        ),
    ])

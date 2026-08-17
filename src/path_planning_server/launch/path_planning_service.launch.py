from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
    use_rviz = LaunchConfiguration("use_rviz")
    use_virtual_joint_states = LaunchConfiguration("use_virtual_joint_states")
    execute_trajectory = LaunchConfiguration("execute_trajectory")

    static_world_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_dual_piper",
        output="screen",
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0",
            "--qx", "0",
            "--qy", "0",
            "--qz", "0",
            "--qw", "1",
            "--frame-id", "world",
            "--child-frame-id", "rviz_piper_world",
        ],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="dual_piper_joint_state_publisher",
        output="screen",
        condition=IfCondition(use_virtual_joint_states),
        parameters=[{
            "rate": 30,
            "source_list": ["/path_planning_test_joint_states"],
        }],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="dual_piper_robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"publish_frequency": 30.0},
        ],
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "allow_trajectory_execution": False,
            },
        ],
    )

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
                "default_scene_id": ParameterValue(
                    scene_id,
                    value_type=int,
                ),
                "execute_trajectory": ParameterValue(
                    execute_trajectory,
                    value_type=bool,
                ),
            },
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="path_planning_rviz",
        output="screen",
        condition=IfCondition(use_rviz),
        arguments=[
            "-d",
            f"{package_share}/config/moveit.rviz",
        ],
        parameters=[
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("scene_id", default_value="1"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument(
            "use_virtual_joint_states",
            default_value="true",
            description="Publish virtual joint states for RViz-only validation.",
        ),
        DeclareLaunchArgument(
            "execute_trajectory",
            default_value="false",
            description="Execute successful plans through Piper trajectory actions.",
        ),
        static_world_transform,
        joint_state_publisher,
        robot_state_publisher,
        move_group,
        planning_server,
        rviz,
    ])

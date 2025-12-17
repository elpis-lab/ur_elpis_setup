from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Complete system launch file that brings together:
    - MoveIt move_group
    - MoveIt RViz
    - Environment setup (table, ceiling)
    """
    
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "config_file",
            default_value=os.path.join(
                get_package_share_directory("setup_environment"),
                "config",
                "environment_config.yaml"
            ),
            description="Path to environment configuration file",
        )
    )
    
    # Initialize arguments
    launch_rviz = LaunchConfiguration("launch_rviz")
    config_file = LaunchConfiguration("config_file")

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution([FindPackageShare("ur_elpis_moveit_config"), "launch"]),
                "/move_group.launch.py",
            ]
        ),
    )
    
    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution([FindPackageShare("ur_elpis_moveit_config"), "launch"]),
                "/moveit_rviz.launch.py",
            ]
        ),
        condition=IfCondition(launch_rviz),
    )
    
    setup_environment_node = Node(
        package="setup_environment",
        executable="setup_environment_node",
        name="setup_environment_node",
        output="screen",
        parameters=[{
            "config_file": config_file
        }],
    )
    
    # Delay setup_environment_node to start after move_group (wait 5 seconds for move_group to initialize)
    delayed_setup_environment = TimerAction(
        period=5.0,
        actions=[setup_environment_node]
    )
    
    return LaunchDescription(
        declared_arguments + [
            move_group_launch,
            moveit_rviz_launch,
            delayed_setup_environment,
        ]
    )


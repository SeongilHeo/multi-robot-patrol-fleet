from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")

    patrol_manager = Node(
        package="mini_dogu_patrol",
        executable="patrol_manager",
        name="patrol_manager",
        output="screen",
        parameters=[params_file],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("mini_dogu_patrol"),
                "config",
                "patrol.yaml",
            ]),
            description="Path to the patrol parameter file",
        ),
        patrol_manager,
    ])
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_namespace = LaunchConfiguration("robot_namespace")
    frame_id = LaunchConfiguration("frame_id")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")

    patrol_manager = Node(
        package="patrol",
        executable="patrol_manager",
        namespace=robot_namespace,
        name="patrol_manager",
        output="screen",
        parameters=[
            params_file,
            {
                "robot_namespace": robot_namespace,
                "frame_id": frame_id,
                "use_sim_time": use_sim_time,
                "autostart": autostart,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_namespace",
            default_value="robot1",
        ),
        DeclareLaunchArgument(
            "frame_id",
            default_value="robot1/map",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "autostart",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("patrol"),
                "config",
                "patrol.yaml",
            ]),
        ),
        patrol_manager,
    ])
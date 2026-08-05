import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    mapping_share = get_package_share_directory("mini_dogu_mapping")

    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_slam_launch = os.path.join(
        mapping_share,
        "launch",
        "robot_slam.launch.py",
    )

    robot1_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_slam_launch),
        launch_arguments={
            "namespace": "robot1",
            "use_sim_time": use_sim_time,
            "map_frame": "robot1/map",
            "odom_frame": "robot1/odom",
            "base_frame": "robot1/base_link",
            "scan_topic": "scan",
        }.items(),
    )

    robot2_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_slam_launch),
        launch_arguments={
            "namespace": "robot2",
            "use_sim_time": use_sim_time,
            "map_frame": "robot2/map",
            "odom_frame": "robot2/odom",
            "base_frame": "robot2/base_link",
            "scan_topic": "scan",
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
        ),
        robot1_slam,
        robot2_slam,
    ])
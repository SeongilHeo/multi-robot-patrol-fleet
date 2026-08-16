import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory("bringup")
    mapping_share = get_package_share_directory("mapping")

    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_description_launch = os.path.join(
        bringup_share,
        "launch",
        "multi_robot.launch.py",
    )

    multi_robot_slam_launch = os.path.join(
        mapping_share,
        "launch",
        "multi_robot_slam.launch.py",
    )

    map_frames_launch = os.path.join(
        mapping_share,
        "launch",
        "map_frames.launch.py",
    )

    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_description_launch),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    multi_robot_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(multi_robot_slam_launch),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    map_frames = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(map_frames_launch),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
        ),
        robot_description,
        multi_robot_slam,
        map_frames,
    ])
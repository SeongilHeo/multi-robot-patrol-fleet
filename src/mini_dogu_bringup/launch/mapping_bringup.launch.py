import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bringup_share = get_package_share_directory("mini_dogu_bringup")
    mapping_share = get_package_share_directory("mini_dogu_mapping")

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

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_description_launch),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(multi_robot_slam_launch),
        ),
    ])
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    mapping_share = get_package_share_directory("mini_dogu_mapping")

    robot_slam_launch = os.path.join(
        mapping_share,
        "launch",
        "robot_slam.launch.py",
    )

    robot1_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_slam_launch),
        launch_arguments={
            "namespace": "robot1",
            "use_sim_time": "false",
        }.items(),
    )

    robot2_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_slam_launch),
        launch_arguments={
            "namespace": "robot2",
            "use_sim_time": "true",
        }.items(),
    )

    return LaunchDescription([
        robot1_slam,
        robot2_slam,
    ])
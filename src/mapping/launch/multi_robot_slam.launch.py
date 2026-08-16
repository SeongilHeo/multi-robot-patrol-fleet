import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


ROBOT_IDS = [
    "robot1",
    "robot2",
    "robot3",
]


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    mapping_share = get_package_share_directory(
        "mapping"
    )

    robot_slam_launch = os.path.join(
        mapping_share,
        "launch",
        "robot_slam.launch.py",
    )

    slam_actions = []

    for robot_id in ROBOT_IDS:
        slam_actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    robot_slam_launch
                ),
                launch_arguments={
                    "namespace": robot_id,
                    "use_sim_time": use_sim_time,
                    "map_frame": f"{robot_id}/map",
                    "odom_frame": f"{robot_id}/odom",
                    "base_frame": f"{robot_id}/base_link",
                    "scan_topic": "scan",
                }.items(),
            )
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
        ),
        *slam_actions,
    ])
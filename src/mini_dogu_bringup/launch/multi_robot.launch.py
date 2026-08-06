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
    "robot4",
]


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    bringup_share = get_package_share_directory(
        "mini_dogu_bringup"
    )

    robot_launch_file = os.path.join(
        bringup_share,
        "launch",
        "robot.launch.py",
    )

    robot_actions = []

    for robot_id in ROBOT_IDS:
        robot_actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    robot_launch_file
                ),
                launch_arguments={
                    "namespace": robot_id,
                    "frame_prefix": f"{robot_id}/",
                    "use_sim_time": use_sim_time,
                }.items(),
            )
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
        ),
        *robot_actions,
    ])
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
    autostart = LaunchConfiguration("autostart")

    navigation_share = get_package_share_directory(
        "mini_dogu_navigation"
    )

    robot_navigation_launch = os.path.join(
        navigation_share,
        "launch",
        "robot_navigation.launch.py",
    )

    navigation_actions = []

    for robot_id in ROBOT_IDS:
        params_file = os.path.join(
            navigation_share,
            "config",
            f"{robot_id}_nav2_params.yaml",
        )

        navigation_actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    robot_navigation_launch
                ),
                launch_arguments={
                    "namespace": robot_id,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "params_file": params_file,
                }.items(),
            )
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
        ),

        DeclareLaunchArgument(
            "autostart",
            default_value="false",
        ),

        *navigation_actions,
    ])
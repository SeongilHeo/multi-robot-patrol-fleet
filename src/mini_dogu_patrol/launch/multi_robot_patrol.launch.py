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

    patrol_share = get_package_share_directory(
        "mini_dogu_patrol"
    )

    patrol_launch_file = os.path.join(
        patrol_share,
        "launch",
        "patrol.launch.py",
    )

    patrol_actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                patrol_launch_file
            ),
            launch_arguments={
                "robot_namespace": robot_id,
                "frame_id": f"{robot_id}/map",
                "use_sim_time": use_sim_time,
                "autostart": autostart,
            }.items(),
        )
        for robot_id in ROBOT_IDS
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
        ),

        DeclareLaunchArgument(
            "autostart",
            default_value="false",
        ),

        *patrol_actions,
    ])

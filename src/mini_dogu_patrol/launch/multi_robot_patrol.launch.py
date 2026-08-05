import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


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

    robot1_patrol = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            patrol_launch_file
        ),
        launch_arguments={
            "robot_namespace": "robot1",
            "frame_id": "robot1/map",
            "use_sim_time": use_sim_time,
            "autostart": autostart,
        }.items(),
    )

    robot2_patrol = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            patrol_launch_file
        ),
        launch_arguments={
            "robot_namespace": "robot2",
            "frame_id": "robot2/map",
            "use_sim_time": use_sim_time,
            "autostart": autostart,
        }.items(),
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

        robot1_patrol,
        robot2_patrol,
    ])
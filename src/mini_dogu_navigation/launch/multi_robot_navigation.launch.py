import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


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

    robot1_params = os.path.join(
        navigation_share,
        "config",
        "robot1_nav2_params.yaml",
    )

    robot2_params = os.path.join(
        navigation_share,
        "config",
        "robot2_nav2_params.yaml",
    )

    robot1_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            robot_navigation_launch
        ),
        launch_arguments={
            "namespace": "robot1",
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "params_file": robot1_params,
        }.items(),
    )

    # Start slightly later to separate the initialization load and logs.
    robot2_navigation = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    robot_navigation_launch
                ),
                launch_arguments={
                    "namespace": "robot2",
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "params_file": robot2_params,
                }.items(),
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
        ),

        DeclareLaunchArgument(
            "autostart",
            default_value="true",
        ),

        robot1_navigation,
        robot2_navigation,
    ])
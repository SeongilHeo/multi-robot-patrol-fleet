import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


ROBOT_IDS = [
    "robot1",
    "robot2",
    "robot3",
]


def create_navigation_actions(context):
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    start_interval_seconds = float(
        LaunchConfiguration(
            "start_interval_seconds"
        ).perform(context)
    )

    navigation_share = get_package_share_directory(
        "navigation"
    )

    robot_navigation_launch = os.path.join(
        navigation_share,
        "launch",
        "robot_navigation.launch.py",
    )

    navigation_actions = []

    for index, robot_id in enumerate(ROBOT_IDS):
        params_file = os.path.join(
            navigation_share,
            "config",
            f"{robot_id}_nav2_params.yaml",
        )

        robot_navigation = IncludeLaunchDescription(
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

        navigation_actions.append(
            TimerAction(
                period=index * start_interval_seconds,
                actions=[
                    robot_navigation,
                ],
            )
        )

    return navigation_actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
        ),

        DeclareLaunchArgument(
            "autostart",
            default_value="false",
        ),

        DeclareLaunchArgument(
            "start_interval_seconds",
            default_value="45.0",
            description=(
                "Wall-clock gap between starting each robot's Nav2 "
                "stack. A robot launched while earlier robots' stacks "
                "are still settling (costmaps, BT construction, SLAM) "
                "competes for the same CPU and can time out its own "
                "lifecycle transitions. Raise this on smaller/shared "
                "hosts if a robot repeatedly gets stuck at "
                "lifecycle=waiting long after its map/tf are ready."
            ),
        ),

        OpaqueFunction(
            function=create_navigation_actions,
        ),
    ])

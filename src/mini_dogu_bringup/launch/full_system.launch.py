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
    start_navigation = LaunchConfiguration("start_navigation")
    start_patrol = LaunchConfiguration("start_patrol")

    sim_share = get_package_share_directory("mini_dogu_sim")
    bringup_share = get_package_share_directory("mini_dogu_bringup")
    navigation_share = get_package_share_directory(
        "mini_dogu_navigation"
    )
    patrol_share = get_package_share_directory("mini_dogu_patrol")
    merge_map_share = get_package_share_directory("merge_map")

    simulation_launch = os.path.join(
        sim_share,
        "launch",
        "multi_robot_sim.launch.py",
    )

    system_bringup_launch = os.path.join(
        bringup_share,
        "launch",
        "system_bringup.launch.py",
    )

    merge_map_launch = os.path.join(
        merge_map_share,
        "launch",
        "merge_map_launch.py",
    )

    navigation_launch = os.path.join(
        navigation_share,
        "launch",
        "multi_robot_navigation.launch.py",
    )

    patrol_launch = os.path.join(
        patrol_share,
        "launch",
        "multi_robot_patrol.launch.py",
    )

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simulation_launch),
    )

    system_bringup = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    system_bringup_launch
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                }.items(),
            )
        ],
    )

    map_merge = TimerAction(
        period=7.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    merge_map_launch
                ),
                launch_arguments={
                    "robot_count": "2",
                    "frame_id": "merged_map",
                    "use_sim_time": use_sim_time,
                }.items(),
            )
        ],
    )

    navigation = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    patrol_launch
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "autostart": start_navigation,
                }.items(),
            )
        ],
    )

    patrol = TimerAction(
        period=13.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    patrol_launch
                ),
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock",
        ),

        DeclareLaunchArgument(
            "start_navigation",
            default_value="true",
            description="Automatically activate robot1 Nav2",
        ),

        DeclareLaunchArgument(
            "start_patrol",
            default_value="false",
            description=(
                "Reserved for future automatic patrol startup"
            ),
        ),

        simulation,
        system_bringup,
        map_merge,
        navigation,
        patrol,
    ])
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bringup_share = get_package_share_directory("bringup")
    fleet_share = get_package_share_directory("fleet")

    use_sim_time = LaunchConfiguration("use_sim_time")

    mapping_bringup_launch = os.path.join(
        bringup_share,
        "launch",
        "mapping_bringup.launch.py",
    )

    fleet_bringup_launch = os.path.join(
        fleet_share,
        "launch",
        "fleet_bringup.launch.py",
    )

    mapping_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mapping_bringup_launch),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    fleet_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fleet_bringup_launch),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
        ),

        mapping_bringup,
        fleet_bringup,
    ])
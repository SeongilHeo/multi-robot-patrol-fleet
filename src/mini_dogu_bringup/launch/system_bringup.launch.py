import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bringup_share = get_package_share_directory("mini_dogu_bringup")
    fleet_share = get_package_share_directory("mini_dogu_fleet")

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
    )

    fleet_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fleet_bringup_launch),
    )

    return LaunchDescription([
        mapping_bringup,
        fleet_bringup,
    ])
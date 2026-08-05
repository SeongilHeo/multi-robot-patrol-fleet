import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    navigation_share = get_package_share_directory(
        "mini_dogu_navigation"
    )

    nav2_bringup_share = get_package_share_directory(
        "nav2_bringup"
    )

    params_file = os.path.join(
        navigation_share,
        "config",
        "nav2_params.yaml",
    )

    navigation_launch = os.path.join(
        nav2_bringup_share,
        "launch",
        "navigation_launch.py",
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch),
        launch_arguments={
            "namespace": namespace,
            "use_namespace": "true",
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "params_file": params_file,
            "use_composition": "false",
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "namespace",
            default_value="robot1",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
        ),
        nav2,
    ])
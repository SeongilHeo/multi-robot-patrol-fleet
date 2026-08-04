import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    mini_dogu_sim_share = get_package_share_directory("mini_dogu_sim")

    world_file = os.path.join(
        mini_dogu_sim_share,
        "worlds",
        "patrol_world.sdf",
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_sim_share,
                "launch",
                "gz_server.launch.py",
            )
        ),
        launch_arguments={
            "world_sdf_file": world_file,
        }.items(),
    )

    return LaunchDescription([
        gazebo_server,
    ])

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    mini_dogu_sim_share = get_package_share_directory("mini_dogu_sim")

    world_file = os.path.join(
        mini_dogu_sim_share,
        "worlds",
        "patrol_world.sdf",
    )

    model_path = os.path.join(
        mini_dogu_sim_share,
        "models",
    )

    existing_resource_path = os.environ.get(
        "GZ_SIM_RESOURCE_PATH",
        "",
    )

    gazebo_resource_path = (
        model_path
        if not existing_resource_path
        else model_path + os.pathsep + existing_resource_path
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
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=gazebo_resource_path,
        ),
        gazebo_server,
    ])
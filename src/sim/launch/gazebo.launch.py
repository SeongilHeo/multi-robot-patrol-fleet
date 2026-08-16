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
    sim_share = get_package_share_directory("sim")

    world_file = os.path.join(
        sim_share,
        "worlds",
        "patrol_world.sdf",
    )

    model_path = os.path.join(
        sim_share,
        "models",
    )

    existing_resource_path = os.environ.get(
        "GZ_SIM_RESOURCE_PATH",
        "",
    )

    resource_path = (
        model_path
        if not existing_resource_path
        else model_path + os.pathsep + existing_resource_path
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_sim_share,
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            "gz_args": f"-r {world_file}",
        }.items(),
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            resource_path,
        ),
        gazebo,
    ])
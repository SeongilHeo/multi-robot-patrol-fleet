import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bridge_config = os.path.join(
        get_package_share_directory("mini_dogu_sim"),
        "config",
        "bridge.yaml",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gazebo_bridge",
        output="screen",
        parameters=[
            {
                "config_file": bridge_config,
            }
        ],
    )

    return LaunchDescription([
        bridge,
    ])
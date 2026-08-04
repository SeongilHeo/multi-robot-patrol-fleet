import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")

    params_file = os.path.join(
        get_package_share_directory("mini_dogu_mapping"),
        "config",
        "slam_params.yaml",
    )

    slam_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        namespace=namespace,
        name="slam_toolbox",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": use_sim_time,
            },
        ],
        remappings=[
            ("scan", "scan"),
            ("map", "map"),
            ("map_metadata", "map_metadata"),
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
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
        slam_node,
    ])
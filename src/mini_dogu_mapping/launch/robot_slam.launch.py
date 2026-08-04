import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")

    map_frame = LaunchConfiguration("map_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")
    scan_topic = LaunchConfiguration("scan_topic")

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
                "map_frame": map_frame,
                "odom_frame": odom_frame,
                "base_frame": base_frame,
                "scan_topic": scan_topic,
            },
        ],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
            ("scan", scan_topic),
            ("map", "map"),
            ("map_metadata", "map_metadata"),
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

        DeclareLaunchArgument(
            "map_frame",
            default_value="robot1/map",
        ),

        DeclareLaunchArgument(
            "odom_frame",
            default_value="robot1/odom",
        ),

        DeclareLaunchArgument(
            "base_frame",
            default_value="robot1/base_link",
        ),

        DeclareLaunchArgument(
            "scan_topic",
            default_value="scan",
        ),

        slam_node,
    ])
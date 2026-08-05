from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    frame_id = LaunchConfiguration("frame_id")
    robot_count = LaunchConfiguration("robot_count")
    use_sim_time = LaunchConfiguration("use_sim_time")
    tf_timeout_seconds = LaunchConfiguration(
        "tf_timeout_seconds"
    )

    merge_map_node = Node(
        package="merge_map",
        executable="merge_map",
        name="merge_map",
        output="screen",
        parameters=[
            {
                "frame_id": frame_id,
                "robot_count": robot_count,
                "use_sim_time": use_sim_time,
                "tf_timeout_seconds": tf_timeout_seconds,
            }
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "frame_id",
            default_value="merged_map",
            description="Frame used by the merged grid",
        ),

        DeclareLaunchArgument(
            "robot_count",
            default_value="2",
            description="Number of robot maps to merge",
        ),

        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock",
        ),

        DeclareLaunchArgument(
            "tf_timeout_seconds",
            default_value="0.5",
            description="Timeout for each map-frame TF lookup",
        ),

        merge_map_node,
    ])
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    robot1_map_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="robot1_map_static_tf",
        output="screen",
        arguments=[
            "--x", "-2.0",
            "--y", "0.0",
            "--z", "0.0",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", "merged_map",
            "--child-frame-id", "robot1/map",
        ],
    )

    robot2_map_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="robot2_map_static_tf",
        output="screen",
        arguments=[
            "--x", "2.0",
            "--y", "0.0",
            "--z", "0.0",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "3.14159265",
            "--frame-id", "merged_map",
            "--child-frame-id", "robot2/map",
        ],
    )

    return LaunchDescription([
        robot1_map_tf,
        robot2_map_tf,
    ])
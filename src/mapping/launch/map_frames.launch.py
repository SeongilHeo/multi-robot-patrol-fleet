from launch import LaunchDescription
from launch_ros.actions import Node


MAP_TRANSFORMS = [
    {
        "robot_id": "robot1",
        "x": "-10.0",
        "y": "0.0",
        "yaw": "0.0",
    },
    {
        "robot_id": "robot2",
        "x": "10.0",
        "y": "0.0",
        "yaw": "3.14159265",
    },
    {
        "robot_id": "robot3",
        "x": "0.0",
        "y": "6.0",
        "yaw": "-1.57079633",
    },
]


def generate_launch_description():
    transform_nodes = []

    for transform in MAP_TRANSFORMS:
        robot_id = transform["robot_id"]

        transform_nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name=f"{robot_id}_map_static_tf",
                output="screen",
                arguments=[
                    "--x", transform["x"],
                    "--y", transform["y"],
                    "--z", "0.0",
                    "--roll", "0.0",
                    "--pitch", "0.0",
                    "--yaw", transform["yaw"],
                    "--frame-id", "merged_map",
                    "--child-frame-id", f"{robot_id}/map",
                ],
            )
        )

    return LaunchDescription(transform_nodes)
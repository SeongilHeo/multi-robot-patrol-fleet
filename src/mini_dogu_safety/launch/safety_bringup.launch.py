from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ROBOT_IDS = [
    "robot1",
    "robot2",
    "robot3",
]


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    safety_gates = [
        Node(
            package="mini_dogu_safety",
            executable="safety_gate_node",
            namespace=robot_id,
            name="safety_gate",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                }
            ],
        )
        for robot_id in ROBOT_IDS
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
        ),

        *safety_gates,
    ])

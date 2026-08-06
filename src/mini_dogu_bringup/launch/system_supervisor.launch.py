from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    supervisor = Node(
        package="mini_dogu_bringup",
        executable="system_readiness_supervisor",
        name="system_readiness_supervisor",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_ids": [
                    "robot1",
                    "robot2",
                    "robot3",
                    "robot4",
                ],
                "readiness_check_period_seconds": 1.0,
                "startup_retry_period_seconds": 5.0,
                "tf_timeout_seconds": 0.2,
            }
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
        ),
        supervisor,
    ])
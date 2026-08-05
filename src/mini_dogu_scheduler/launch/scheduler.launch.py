from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    minimum_battery_percentage = LaunchConfiguration(
        "minimum_battery_percentage"
    )

    scheduler = Node(
        package="mini_dogu_scheduler",
        executable="mission_scheduler_node",
        name="mission_scheduler",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "fleet_state_topic": "/fleet/state",
                "assign_mission_service": "/fleet/assign_mission",
                "minimum_battery_percentage":
                    minimum_battery_percentage,
            }
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "minimum_battery_percentage",
            default_value="20.0",
        ),
        scheduler,
    ])
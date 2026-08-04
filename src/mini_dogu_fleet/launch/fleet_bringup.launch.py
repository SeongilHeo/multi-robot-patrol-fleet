from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    fleet_manager = Node(
        package="mini_dogu_fleet",
        executable="fleet_manager_node",
        name="fleet_manager",
        output="screen",
        parameters=[
            {
                "robot_ids": ["robot1", "robot2"],
                "heartbeat_timeout_seconds": 3.0,
            }
        ],
    )

    robot1_heartbeat = Node(
        package="mini_dogu_fleet",
        executable="robot_heartbeat_node",
        namespace="robot1",
        name="robot_heartbeat",
        output="screen",
        parameters=[
            {
                "robot_id": "robot1",
                "battery_percentage": 90.0,
                "state": 1,
                "current_mission": "",
            }
        ],
    )

    robot2_heartbeat = Node(
        package="mini_dogu_fleet",
        executable="robot_heartbeat_node",
        namespace="robot2",
        name="robot_heartbeat",
        output="screen",
        parameters=[
            {
                "robot_id": "robot2",
                "battery_percentage": 80.0,
                "state": 2,
                "current_mission": "mapping_zone_b",
            }
        ],
    )

    return LaunchDescription([
        fleet_manager,
        robot1_heartbeat,
        robot2_heartbeat,
    ])
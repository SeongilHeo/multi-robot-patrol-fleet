from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    robot_configs = [
        ("robot1", 90.0),
        ("robot2", 80.0),
        ("robot3", 85.0),
        ("robot4", 75.0),
    ]

    fleet_manager = Node(
        package="mini_dogu_fleet",
        executable="fleet_manager_node",
        name="fleet_manager",
        output="screen",
        parameters=[
            {
                "robot_ids": [
                    robot_id
                    for robot_id, _ in robot_configs
                ],
                "heartbeat_timeout_seconds": 3.0,
                "offline_grace_seconds": 20.0,
                "auto_estop_after_offline_seconds": 0.0,
                "patrol_robot": "robot1",
                "mission_db_path": "/tmp/mini_dogu_fleet.db",
                "incident_auto_dispatch_min_severity": 2,
                "incident_auto_dispatch_priority": 100,
            }
        ],
    )

    heartbeat_nodes = []

    for robot_id, battery_percentage in robot_configs:
        heartbeat_nodes.append(
            Node(
                package="mini_dogu_fleet",
                executable="robot_heartbeat_node",
                namespace=robot_id,
                name="robot_heartbeat",
                output="screen",
                parameters=[
                    {
                        "robot_id": robot_id,
                        "battery_percentage": battery_percentage,
                        "default_state": 1,
                        "default_mission": "",
                    }
                ],
            )
        )

    return LaunchDescription([
        fleet_manager,
        *heartbeat_nodes,
    ])
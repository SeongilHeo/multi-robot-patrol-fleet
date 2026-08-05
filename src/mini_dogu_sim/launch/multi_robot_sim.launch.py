import os
import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


WORLD_NAME = "patrol_world"

ROBOTS = [
    {
        "name": "robot1",
        "x": -2.0,
        "y": 0.0,
        "z": 0.0,
        "yaw": 0.0,
    },
    {
        "name": "robot2",
        "x": 2.0,
        "y": 0.0,
        "z": 0.0,
        "yaw": 3.14159265,
    },
]


def generate_robot_sdf(template_path: str, robot_name: str) -> str:
    template = Path(template_path).read_text(encoding="utf-8")

    rendered = template.replace(
        "@ROBOT_NAME@",
        robot_name,
    )

    output_path = os.path.join(
        tempfile.gettempdir(),
        f"mini_dogu_{robot_name}.sdf",
    )

    Path(output_path).write_text(
        rendered,
        encoding="utf-8",
    )

    return output_path


def create_robot_actions(context):
    del context

    mini_dogu_sim_share = get_package_share_directory(
        "mini_dogu_sim"
    )

    ros_gz_sim_share = get_package_share_directory(
        "ros_gz_sim"
    )

    template_path = os.path.join(
        mini_dogu_sim_share,
        "models",
        "mini_dogu",
        "model.sdf.template",
    )

    spawn_launch = os.path.join(
        ros_gz_sim_share,
        "launch",
        "gz_spawn_model.launch.py",
    )

    actions = []

    for index, robot in enumerate(ROBOTS):
        robot_name = robot["name"]

        rendered_sdf = generate_robot_sdf(
            template_path,
            robot_name,
        )

        spawn_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(spawn_launch),
            launch_arguments={
                "world": WORLD_NAME,
                "file": rendered_sdf,
                "entity_name": robot_name,
                "x": str(robot["x"]),
                "y": str(robot["y"]),
                "z": str(robot["z"]),
                "roll": "0.0",
                "pitch": "0.0",
                "yaw": str(robot["yaw"]),
            }.items(),
        )

        robot_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name=f"{robot_name}_bridge",
            output="screen",
            arguments=[
                (
                    f"/{robot_name}/cmd_vel"
                    "@geometry_msgs/msg/Twist"
                    "]gz.msgs.Twist"
                ),
                (
                    f"/{robot_name}/odom"
                    "@nav_msgs/msg/Odometry"
                    "[gz.msgs.Odometry"
                ),
                (
                    f"/{robot_name}/tf"
                    "@tf2_msgs/msg/TFMessage"
                    "[gz.msgs.Pose_V"
                ),
                (
                    f"/{robot_name}/scan"
                    "@sensor_msgs/msg/LaserScan"
                    "[gz.msgs.LaserScan"
                ),
            ],
        )

        # Gazebo가 먼저 시작된 뒤 순서대로 spawn한다.
        actions.append(
            TimerAction(
                period=2.0 + index,
                actions=[
                    spawn_robot,
                    robot_bridge,
                ],
            )
        )

    return actions


def generate_launch_description():
    ros_gz_sim_share = get_package_share_directory(
        "ros_gz_sim"
    )

    mini_dogu_sim_share = get_package_share_directory(
        "mini_dogu_sim"
    )

    world_file = os.path.join(
        mini_dogu_sim_share,
        "worlds",
        "patrol_world.sdf",
    )

    model_path = os.path.join(
        mini_dogu_sim_share,
        "models",
    )

    existing_resource_path = os.environ.get(
        "GZ_SIM_RESOURCE_PATH",
        "",
    )

    resource_path = (
        model_path
        if not existing_resource_path
        else model_path
        + os.pathsep
        + existing_resource_path
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_sim_share,
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            "gz_args": f"-r {world_file}",
        }.items(),
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=resource_path,
        ),
        gazebo,
        clock_bridge,
        OpaqueFunction(
            function=create_robot_actions,
        ),
    ])
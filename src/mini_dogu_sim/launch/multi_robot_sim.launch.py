import os
import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


WORLD_NAME = "industrial_facility"

ROBOTS = [
    {
        "name": "robot1",
        "x": -10.0,
        "y": 0.0,
        "z": 0.0,
        "yaw": 0.0,
    },
    {
        "name": "robot2",
        "x": 10.0,
        "y": 0.0,
        "z": 0.0,
        "yaw": 3.14159265,
    },
    {
        "name": "robot3",
        "x": 0.0,
        "y": 6.0,
        "z": 0.0,
        "yaw": -1.57079633,
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
    spawn_initial_delay_seconds = float(
        LaunchConfiguration(
            "spawn_initial_delay_seconds"
        ).perform(context)
    )

    spawn_stagger_seconds = float(
        LaunchConfiguration(
            "spawn_stagger_seconds"
        ).perform(context)
    )

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

        lidar_frame_alias = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"{robot_name}_lidar_frame_alias",
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                }
            ],
            arguments=[
                "--x", "0",
                "--y", "0",
                "--z", "0",
                "--roll", "0",
                "--pitch", "0",
                "--yaw", "0",
                "--frame-id", f"{robot_name}/lidar_link",
                "--child-frame-id", f"{robot_name}/base_link/lidar",
            ],
        )

        actions.append(
            TimerAction(
                period=(
                    spawn_initial_delay_seconds +
                    (spawn_stagger_seconds * index)
                ),
                actions=[
                    spawn_robot,
                    lidar_frame_alias,
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
        "industrial_facility.sdf",
    )

    model_path = os.path.join(
        mini_dogu_sim_share,
        "models",
    )
    
    bridge_launch_file = os.path.join(
        mini_dogu_sim_share,
        "launch",
        "bridge.launch.py",
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

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            bridge_launch_file
        ),
        launch_arguments={
            "use_sim_time": "true",
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "spawn_initial_delay_seconds",
            default_value="12.0",
            description=(
                "Wall-clock delay before the first robot is spawned, "
                "giving Gazebo time to load the world and advertise "
                "its create service. Fixed delays race against however "
                "long the world takes to come up (GPU driver warm-up, "
                "world size, host load), so raise this if robots are "
                "still missing from the entity tree after startup."
            ),
        ),
        DeclareLaunchArgument(
            "spawn_stagger_seconds",
            default_value="4.0",
            description=(
                "Additional delay between each robot's spawn, on top "
                "of spawn_initial_delay_seconds."
            ),
        ),
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=resource_path,
        ),
        gazebo,
        bridge,
        OpaqueFunction(
            function=create_robot_actions,
        ),
    ])
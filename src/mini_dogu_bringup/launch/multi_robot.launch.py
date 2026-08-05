import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory("mini_dogu_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_launch = os.path.join(
        bringup_share,
        "launch",
        "robot.launch.py",
    )

    robot1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_launch),
        launch_arguments={
            "namespace": "robot1",
            "frame_prefix": "robot1/",
            "use_sim_time": use_sim_time,
        }.items(),
    )

    robot2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_launch),
        launch_arguments={
            "namespace": "robot2",
            "frame_prefix": "robot2/",
            "use_sim_time": use_sim_time,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
        ),
        robot1,
        robot2,
    ])
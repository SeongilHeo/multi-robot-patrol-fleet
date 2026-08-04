import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution


def generate_launch_description():
    bringup_share = get_package_share_directory("mini_dogu_bringup")

    robot_launch = os.path.join(
        bringup_share,
        "launch",
        "robot.launch.py",
    )

    robot1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_launch),
        launch_arguments={
            "namespace": TextSubstitution(text="robot1"),
            "frame_prefix": TextSubstitution(text="robot1/"),
            "use_sim_time": TextSubstitution(text="false"),
        }.items(),
    )

    robot2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_launch),
        launch_arguments={
            "namespace": TextSubstitution(text="robot2"),
            "frame_prefix": TextSubstitution(text="robot2/"),
            "use_sim_time": TextSubstitution(text="false"),
        }.items(),
    )

    return LaunchDescription([
        robot1,
        robot2,
    ])

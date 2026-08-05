import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    frame_prefix = LaunchConfiguration("frame_prefix")
    use_sim_time = LaunchConfiguration("use_sim_time")

    urdf_file = os.path.join(
        get_package_share_directory("mini_dogu_description"),
        "urdf",
        "mini_dogu.urdf.xacro",
    )

    robot_description = ParameterValue(
        Command([
            "xacro ",
            urdf_file,
            " prefix:=",
            frame_prefix,
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "namespace",
            default_value="robot1",
        ),

        DeclareLaunchArgument(
            "frame_prefix",
            default_value="robot1/",
        ),

        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=namespace,
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }],
        ),
    ])

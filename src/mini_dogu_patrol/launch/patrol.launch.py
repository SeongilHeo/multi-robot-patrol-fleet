from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare

from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    robot_namespace = LaunchConfiguration("robot_namespace")
    frame_id = LaunchConfiguration("frame_id")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=robot_namespace,
            param_rewrites={
                "robot_namespace": robot_namespace,
                "frame_id": frame_id,
                "use_sim_time": use_sim_time,
                "autostart": autostart,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    patrol_manager = Node(
        package="mini_dogu_patrol",
        executable="patrol_manager",
        namespace=robot_namespace,
        name="patrol_manager",
        output="screen",
        parameters=[configured_params],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_namespace",
            default_value="robot1",
            description="Robot ROS namespace",
        ),

        DeclareLaunchArgument(
            "frame_id",
            default_value="robot1/map",
            description="Frame used for patrol waypoints",
        ),

        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock",
        ),

        DeclareLaunchArgument(
            "autostart",
            default_value="false",
            description="Start patrol automatically",
        ),

        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("mini_dogu_patrol"),
                "config",
                "patrol.yaml",
            ]),
            description="Patrol parameter file",
        ),

        patrol_manager,
    ])
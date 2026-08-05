from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushROSNamespace
from launch_ros.parameter_descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    log_level = LaunchConfiguration("log_level")

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={
                "use_sim_time": use_sim_time,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    managed_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
    ]

    navigation_nodes = GroupAction([
        PushROSNamespace(namespace),

        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[configured_params],
            arguments=[
                "--ros-args",
                "--log-level",
                log_level,
            ],
        ),

        Node(
            package="nav2_smoother",
            executable="smoother_server",
            name="smoother_server",
            output="screen",
            parameters=[configured_params],
            arguments=[
                "--ros-args",
                "--log-level",
                log_level,
            ],
        ),

        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[configured_params],
            arguments=[
                "--ros-args",
                "--log-level",
                log_level,
            ],
        ),

        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[configured_params],
            arguments=[
                "--ros-args",
                "--log-level",
                log_level,
            ],
        ),

        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[configured_params],
            arguments=[
                "--ros-args",
                "--log-level",
                log_level,
            ],
        ),

        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            output="screen",
            parameters=[configured_params],
            arguments=[
                "--ros-args",
                "--log-level",
                log_level,
            ],
        ),

        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "bond_timeout": 4.0,
                    "attempt_respawn_reconnection": True,
                    "node_names": managed_nodes,
                }
            ],
            arguments=[
                "--ros-args",
                "--log-level",
                log_level,
            ],
        ),
    ])

    return LaunchDescription([
        SetEnvironmentVariable(
            name="RCUTILS_LOGGING_BUFFERED_STREAM",
            value="1",
        ),

        DeclareLaunchArgument(
            "namespace",
            default_value="robot1",
            description="ROS namespace for the robot",
        ),

        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock from /clock",
        ),

        DeclareLaunchArgument(
            "autostart",
            default_value="false",
            description="Automatically configure and activate Nav2 nodes",
        ),

        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("mini_dogu_navigation"),
                "config",
                "robot1_nav2_params.yaml",
            ]),
            description="Absolute path to the Nav2 parameter file",
        ),

        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="Logging level",
        ),

        navigation_nodes,
    ])
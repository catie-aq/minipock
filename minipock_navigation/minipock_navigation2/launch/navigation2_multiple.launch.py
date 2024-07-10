#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString


def parse_config(context, *args, **kwargs):
    """
    Generate the launch description for the application.

    :param context: The context of the launch.
    :return: A LaunchDescription object.
    """
    start_rviz = LaunchConfiguration("start_rviz").perform(context)
    use_sim_time = IfCondition(LaunchConfiguration("use_sim_time")).evaluate(context)
    autostart = IfCondition(LaunchConfiguration("autostart")).evaluate(context)
    use_composition = LaunchConfiguration("use_composition").perform(context)
    use_respawn = IfCondition(LaunchConfiguration("use_respawn")).evaluate(context)
    bringup = IfCondition(LaunchConfiguration("bringup")).evaluate(context)

    map_yaml_file = LaunchConfiguration(
        "map_yaml_file",
        default=PathJoinSubstitution(
            [FindPackageShare("minipock_navigation2"), "map", "map.yaml"]
        ),
    )
    nav2_launch_file_dir = PathJoinSubstitution(
        [
            FindPackageShare("nav2_bringup"),
            "launch",
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("minipock_navigation2"), "rviz", "navigation2_namespaced.rviz"]
    )

    robots = [
        {"name": "minipock0"},
        {"name": "minipock1"},
    ]

    namespaced_rviz_config_file = ReplaceString(
        source_file=rviz_config_file, replacements={"<robot_namespace>": ("/", robots[0]["name"])}
    )

    default_bt_xml_filename = PathJoinSubstitution(
        [
            FindPackageShare("nav2_bt_navigator"),
            "behavior_trees",
            "navigate_w_replanning_and_recovery.xml",
        ]
    )

    minipock_bringup = None
    if bringup:
        minipock_bringup_file_dir = PathJoinSubstitution(
            [
                FindPackageShare("minipock_bringup"),
                "launch",
            ]
        )
        minipock_bringup = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([minipock_bringup_file_dir, "/bringup.launch.py"]),
            launch_arguments={
                "use_sim_time": use_sim_time,
            }.items(),
        )

    map_server_node, amcl_nodes, lifecycle_manager_localization_node = launch_localization(
        robots, use_sim_time, autostart, use_respawn, map_yaml_file
    )

    navigation_nodes = launch_navigation(robots, use_sim_time, autostart, use_respawn)

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", namespaced_rviz_config_file],
        output="screen",
        condition=IfCondition(start_rviz),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    launch_actions = [
        map_server_node,
        amcl_nodes,
        lifecycle_manager_localization_node,
        navigation_nodes,
        rviz_node,
    ]
    if minipock_bringup is not None:
        launch_actions.append(minipock_bringup)

    return launch_actions


def launch_localization(robots, use_sim_time, autostart, use_respawn, map_yaml_file):
    """
    Generate the launch description for localization.

    :param robots: list of robots containing their names
    :param use_sim_time: use simulation time
    :param autostart: autostart the nodes
    :param use_respawn: respawn the nodes
    :param map_yaml_file: path to the map yaml file
    :return: list of launch actions
    """
    launch_map_server = LaunchDescription(
        [
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"topic_name": "/map"},
                    {"frame-id": "map"},
                    {"yaml_filename": map_yaml_file},
                ],
            ),
        ]
    )

    localization_lifecycle_nodes = ["map_server"]

    launch_amcl = LaunchDescription()
    for robot in robots:
        namespace = robot["name"]
        params_file = LaunchConfiguration(
            "params_file",
            default=PathJoinSubstitution(
                [FindPackageShare("minipock_navigation2"), "param", "minipock_multi.yaml"]
            ),
        )
        params_file = ReplaceString(
            source_file=params_file,
            replacements={
                "<robot_namespace>": namespace,
                "<use_sim_time>": str(use_sim_time),
            },
        )
        launch_amcl.add_action(
            Node(
                namespace=namespace,
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                output="screen",
                parameters=[
                    params_file,
                ],
            )
        )
        localization_lifecycle_nodes.append(f"{namespace}/amcl")

    launch_lifecycle_manager_localization = LaunchDescription(
        [
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                parameters=[
                    {"autostart": autostart},
                    {"node_names": localization_lifecycle_nodes},
                    {"use_sim_time": use_sim_time},
                    {"bond_timeout": 0.0},
                ],
            ),
        ]
    )
    return [launch_map_server, launch_amcl, launch_lifecycle_manager_localization]


def launch_navigation(robots, use_sim_time, autostart, use_respawn):
    """
    Generate the launch description for navigation.

    :param robots: list of robots containing their names
    :param use_sim_time: use simulation time
    :param autostart: autostart the nodes
    :param use_respawn: respawn the nodes
    :return: list of launch actions
    """
    launch_navigation = LaunchDescription()
    for robot in robots:
        namespace = robot["name"]
        params_file = LaunchConfiguration(
            "params_file",
            default=PathJoinSubstitution(
                [FindPackageShare("minipock_navigation2"), "param", "minipock_multi.yaml"]
            ),
        )
        params_file = ReplaceString(
            source_file=params_file,
            replacements={
                "<robot_namespace>": namespace,
                "<use_sim_time>": str(use_sim_time),
            },
        )
        navigation_lifecycle_nodes = [
            f"{namespace}/controller_server",
            f"{namespace}/smoother_server",
            f"{namespace}/planner_server",
            f"{namespace}/behavior_server",
            f"{namespace}/velocity_smoother",
        ]
        launch_navigation.add_action(
            Node(
                namespace=namespace,
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[params_file, {"use_sim_time": use_sim_time}],
            )
        )
        launch_navigation.add_action(
            Node(
                namespace=namespace,
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[params_file, {"use_sim_time": use_sim_time}],
            )
        )
        launch_navigation.add_action(
            Node(
                namespace=namespace,
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[params_file, {"use_sim_time": use_sim_time}],
            )
        )
        launch_navigation.add_action(
            Node(
                namespace=namespace,
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[params_file, {"use_sim_time": use_sim_time}],
            )
        )
        launch_navigation.add_action(
            Node(
                namespace=namespace,
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[params_file, {"use_sim_time": use_sim_time}],
            )
        )
        launch_navigation.add_action(
            Node(
                namespace=namespace,
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[params_file, {"use_sim_time": use_sim_time}],
            )
        )
        launch_navigation.add_action(
            Node(
                namespace=namespace,
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[params_file, {"use_sim_time": use_sim_time}],
            )
        )
        launch_navigation.add_action(
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name=f"lifecycle_manager_navigation_{namespace}",
                output="screen",
                parameters=[
                    {"autostart": autostart},
                    {"node_names": navigation_lifecycle_nodes},
                    {"use_sim_time": use_sim_time},
                    {"bond_timeout": 0.0},
                ],
            ),
        )
    return launch_navigation


def generate_launch_description():
    """
    Generate the launch description for spawning an object in a simulation.

    :return: LaunchDescription object.
    """
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_rviz", default_value="true", description="Whether execute rviz2"
            ),
            DeclareLaunchArgument(
                "use_sim_time", default_value="false", description="Set use_sim_time"
            ),
            DeclareLaunchArgument(
                "bringup",
                default_value="true",
                description="Whether to bringup minipock",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="false",
                description="Automatically startup the nav2 stack",
            ),
            DeclareLaunchArgument(
                "use_composition",
                default_value="False",
                description="Whether to use composed bringup",
            ),
            DeclareLaunchArgument(
                "use_respawn",
                default_value="false",
                description="Whether to respawn if a node crashes. \
                Applied when composition is disabled.",
            ),
            OpaqueFunction(function=parse_config),
        ]
    )

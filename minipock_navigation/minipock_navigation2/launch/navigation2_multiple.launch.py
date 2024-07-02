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

    :return: A LaunchDescription object.
    """
    start_rviz = LaunchConfiguration("start_rviz").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    autostart = IfCondition(LaunchConfiguration("autostart")).evaluate(context)
    use_composition = LaunchConfiguration("use_composition").perform(context)
    use_respawn = LaunchConfiguration("use_respawn").perform(context)
    bringup = IfCondition(LaunchConfiguration("bringup")).evaluate(context)

    map_yaml_file = LaunchConfiguration(
        "map_yaml_file",
        default=PathJoinSubstitution(
            [FindPackageShare("minipock_navigation2"), "map", "map.yaml"]
        ),
    )
    amcl_params_file = LaunchConfiguration(
        "amcl_params_file",
        default=PathJoinSubstitution(
            [FindPackageShare("minipock_navigation2"), "param", "amcl_minipock0.yaml"]
        ),
    )
    params_file = LaunchConfiguration(
        "params_file",
        default=PathJoinSubstitution(
            [FindPackageShare("minipock_navigation2"), "param", "minipock0.yaml"]
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

    namespace = "minipock0"
    namespaced_rviz_config_file = ReplaceString(
        source_file=rviz_config_file, replacements={"<robot_namespace>": ("/", namespace)}
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

    launch_map_server = LaunchDescription(
        [
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[
                    {"use_sim_time": True},
                    {"topic_name": "/map"},
                    {"frame-id": "map"},
                    {"yaml_filename": map_yaml_file},
                ],
            ),
        ]
    )
    launch_amcl = LaunchDescription(
        [
            Node(
                namespace=namespace,
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                output="screen",
                parameters=[amcl_params_file],
            ),
        ]
    )

    launch_lifecycle_manager_localization = LaunchDescription(
        [
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                parameters=[
                    {"autostart": autostart},
                    {"node_names": ["map_server", "minipock0/amcl"]},
                    {"use_sim_time": True},
                    {"bond_timeout": 0.0},
                ],
            ),
        ]
    )

    launch_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", namespaced_rviz_config_file],
        output="screen",
        condition=IfCondition(start_rviz),
        parameters=[{"use_sim_time": IfCondition(use_sim_time).evaluate(context)}],
    )

    launch_actions = [
        launch_map_server,
        launch_amcl,
        launch_lifecycle_manager_localization,
        launch_rviz,
    ]
    if minipock_bringup is not None:
        launch_actions.append(minipock_bringup)

    return launch_actions


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

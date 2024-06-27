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
    autostart = LaunchConfiguration("autostart").perform(context)
    use_composition = LaunchConfiguration("use_composition").perform(context)
    use_respawn = LaunchConfiguration("use_respawn").perform(context)
    bringup = IfCondition(LaunchConfiguration("bringup")).evaluate(context)

    map_yaml_file = LaunchConfiguration(
        "map_yaml_file",
        default=PathJoinSubstitution(
            [FindPackageShare("minipock_navigation2"), "map", "map.yaml"]
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
            source_file=rviz_config_file,
            replacements={'<robot_namespace>': ('/', namespace)})
    
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

    launch_actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, "/bringup_launch.py"]),
            launch_arguments={
                "map": map_yaml_file,
                "use_sim_time": use_sim_time,
                "params_file": params_file,
                "default_bt_xml_filename": default_bt_xml_filename,
                "autostart": autostart,
                "use_composition": use_composition,
                "use_respawn": use_respawn,
                "use_namespace": "true",
                "namespace": "minipock0",
            }.items(),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", namespaced_rviz_config_file],
            output="screen",
            condition=IfCondition(start_rviz),
            parameters=[{"use_sim_time": IfCondition(use_sim_time).evaluate(context)}],
        ),
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
                default_value="True",
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

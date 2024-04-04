#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Generate the launch description for the application.

    :return: A LaunchDescription object.
    """
    start_rviz = LaunchConfiguration("start_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")

    map_yaml_file = LaunchConfiguration(
        "map_yaml_file",
        default=PathJoinSubstitution([FindPackageShare("minipock_navigation2"), "map", "map.yaml"]),
    )
    params_file = LaunchConfiguration(
        "params_file",
        default=PathJoinSubstitution(
            [FindPackageShare("minipock_navigation2"), "param", "minipock.yaml"]
        ),
    )

    nav2_launch_file_dir = PathJoinSubstitution(
        [
            FindPackageShare("nav2_bringup"),
            "launch",
        ]
    )

    minipock_bringup_file_dir = PathJoinSubstitution(
        [
            FindPackageShare("minipock_bringup"),
            "launch",
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("minipock_navigation2"), "rviz", "navigation2.rviz"]
    )

    default_bt_xml_filename = PathJoinSubstitution(
        [
            FindPackageShare("nav2_bt_navigator"),
            "behavior_trees",
            "navigate_w_replanning_and_recovery.xml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_rviz", default_value="true", description="Whether execute rviz2"
            ),
            DeclareLaunchArgument(
                "use_sim_time", default_value="true", description="Set use_sim_time"
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
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(
            #         [minipock_bringup_file_dir, "/bringup.launch.py"]
            #     ),
            # ),
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
                }.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config_file],
                output="screen",
                condition=IfCondition(start_rviz),
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )

"""Simulate the fleet of a fleet file: Gazebo world, robots, shared localization and Nav2."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import Action, LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

from minipock_bringup.config import load_config, resolve_config_path
from minipock_bringup.launch_args import include, pick, robot_launch_arguments
from minipock_bringup.localization import localization_actions

USE_SIM_TIME = True


def _fleet(context: LaunchContext) -> list[Action]:
    """Return the world and robot actions of the fleet file given by `config`."""
    cfg = load_config(resolve_config_path(LaunchConfiguration("config").perform(context)))
    nav2_share = Path(get_package_share_directory("minipock_navigation2"))
    robots = [
        robot_launch_arguments(robot, cfg, use_sim_time=USE_SIM_TIME) for robot in cfg.robots
    ]
    actions: list[Action] = [
        include(
            "minipock_gz",
            "world.launch.py",
            {"world": cfg.world, "headless": LaunchConfiguration("headless").perform(context)},
        ),
        *localization_actions(
            [(args["namespace"], args["params_file"]) for args in robots],
            str(nav2_share / "map" / f"{cfg.map}.yaml"),
            use_sim_time=USE_SIM_TIME,
        ),
    ]
    actions += [
        GroupAction(
            [
                include(
                    "minipock_description",
                    "description.launch.py",
                    pick(args, "mode", "sim_index"),
                ),
                include("minipock_gz", "robot.launch.py", pick(args, "x", "y", "yaw")),
                include("minipock_navigation2", "robot.launch.py", pick(args, "params_file")),
            ],
            scoped=True,
        )
        for args in robots
    ]
    actions.append(
        Node(
            condition=IfCondition(LaunchConfiguration("rviz")),
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", str(nav2_share / "rviz" / f"{cfg.rviz}.rviz")],
        )
    )
    return actions


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description of the simulated fleet.

    The fleet file is read once; every node gets `use_sim_time: true`.

    Returns:
        The launch description. Arguments: `config` (path, or name of a file installed in
        `minipock_bringup/config`), `rviz` and `headless` (Gazebo server only).
    """
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value="minipocks.yaml",
                description="Fleet file: path, or name of a file of minipock_bringup/config",
            ),
            DeclareLaunchArgument("rviz", default_value="true", description="Start RViz"),
            DeclareLaunchArgument(
                "headless", default_value="false", description="Run the Gazebo server only"
            ),
            SetParameter(name="use_sim_time", value=USE_SIM_TIME),
            OpaqueFunction(function=_fleet),
        ]
    )

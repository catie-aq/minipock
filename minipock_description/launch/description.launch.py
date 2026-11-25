"""Publish the robot description of one robot, built with xacro at launch time."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description of `robot_state_publisher` for one robot.

    Returns:
        The launch description. Arguments: `namespace` (without trailing slash), `mode`
        (`differential` or `holonomic`) and `use_sim_time` (`true` or `false`).
    """
    share = Path(get_package_share_directory("minipock_description"))
    namespace = LaunchConfiguration("namespace")
    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                str(share / "urdf" / "minipock.urdf.xacro"),
                " namespace:=",
                namespace,
                "/ mode:=",
                LaunchConfiguration("mode"),
            ]
        ),
        value_type=str,
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="minipock_0"),
            DeclareLaunchArgument("mode", default_value="differential"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace=namespace,
                output="both",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                    }
                ],
            ),
        ]
    )

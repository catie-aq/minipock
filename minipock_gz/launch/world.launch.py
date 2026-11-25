"""Start Gazebo Sim with a MiniPock world and the clock bridge."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from minipock_gz import bridges


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description of the simulated world.

    Returns:
        The launch description. Arguments: `world` (file name without `.sdf`) and `headless`
        (`true` runs the server only, with headless rendering).
    """
    gz_share = Path(get_package_share_directory("minipock_gz"))
    description_share = Path(get_package_share_directory("minipock_description"))
    gz_sim = str(Path(get_package_share_directory("ros_gz_sim")) / "launch" / "gz_sim.launch.py")
    world = [str(gz_share / "worlds") + "/", LaunchConfiguration("world"), ".sdf"]
    headless = LaunchConfiguration("headless")
    clock = bridges.clock()
    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="minipock_world"),
            DeclareLaunchArgument("headless", default_value="false"),
            AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", str(gz_share / "worlds")),
            # Vendored models of the office world, referenced as `model://<name>`.
            AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", str(gz_share / "models")),
            # Models are referenced as `model://minipock_description/...`.
            AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", str(description_share.parent)),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_sim),
                launch_arguments={"gz_args": ["-r ", *world]}.items(),
                condition=UnlessCondition(headless),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_sim),
                launch_arguments={"gz_args": ["-r -s --headless-rendering ", *world]}.items(),
                condition=IfCondition(headless),
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                output="screen",
                arguments=[clock.argument()],
                remappings=[clock.remapping()],
            ),
        ]
    )

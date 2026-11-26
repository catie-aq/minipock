"""Spawn one MiniPock in the running Gazebo world and bridge its topics."""

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from minipock_gz import bridges

# Height of the model origin (base_footprint, at base_link height) at spawn. The wheel contact
# plane is 0.0975 m below base_link: spawning lower puts the wheels and the chassis inside the
# floor mesh, where the physics leaves them sunk (chassis rubbing the floor). 2.5 mm drop.
SPAWN_HEIGHT = "0.1"


def _robot(context: LaunchContext) -> list[Node]:
    """Return the spawn and bridge nodes of the robot named by the launch arguments."""
    namespace = LaunchConfiguration("namespace").perform(context)
    robot_bridges = bridges.robot_bridges(namespace)
    return [
        Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=[
                "-topic",
                f"/{namespace}/robot_description",
                "-name",
                namespace,
                "-x",
                LaunchConfiguration("x").perform(context),
                "-y",
                LaunchConfiguration("y").perform(context),
                "-z",
                SPAWN_HEIGHT,
                "-Y",
                LaunchConfiguration("yaw").perform(context),
            ],
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name=f"{namespace}_bridge",
            output="screen",
            arguments=[bridge.argument() for bridge in robot_bridges],
            remappings=[bridge.remapping() for bridge in robot_bridges],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description spawning one robot.

    Returns:
        The launch description. Arguments: `namespace` (without trailing slash), `x`, `y` and
        `yaw`. The robot description must already be published on
        `/<namespace>/robot_description`.
    """
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="minipock_0"),
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            OpaqueFunction(function=_robot),
        ]
    )

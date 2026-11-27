"""Shape the firmware topics of one real robot into `odom`, `scan`, TF and joint states."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description of the host-side nodes of one real robot.

    The robot description must already be published on `/<namespace>/robot_description`.

    Returns:
        The launch description. Arguments: `namespace` (without trailing slash) and
        `use_sim_time`.
    """
    namespace = LaunchConfiguration("namespace")
    use_sim_time = {
        "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    }
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="minipock_0"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="minipock_bringup",
                executable="raw_data_transformer",
                namespace=namespace,
                output="screen",
                parameters=[use_sim_time],
            ),
            # The firmware publishes no joint states: the wheels are drawn at rest.
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                namespace=namespace,
                output="screen",
                parameters=[use_sim_time],
            ),
        ]
    )

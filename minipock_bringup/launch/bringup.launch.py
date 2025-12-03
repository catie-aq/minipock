import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from minipock_description import model, config


def parse_config(context, *args, **kwargs):
    """
    The `parse_config` method is used to parse a configuration in the specified context.

    It retrieves the robot name using the LaunchConfiguration class and then performs the necessary
    operations

    :param context: The context object containing information for parsing the config.
    :param args: Additional arguments for the method. Not used in this method.
    :param kwargs: Additional keyword arguments for the method. Not used in this method.
    :return: A list of launch processes.

    * to spawn the robot and get its state.
    """
    odom_frame = LaunchConfiguration("odom_frame").perform(context)
    config_dict = config.config()
    namespace = config_dict["namespace"]
    fleet = config_dict["fleet"]

    launch_processes = []
    launch_processes.append(micro_ros_agent())
    for robot in fleet:
        robot_name = f"{namespace}{robot}"
        robot_mode = fleet[robot]["mode"]
        launch_processes.extend(spawn(robot_name, robot_mode))
        launch_processes.append(state(robot_name))
        if odom_frame:
            launch_processes.append(odometry(robot_name))
    return launch_processes


def micro_ros_agent():
    """
    :return: LaunchDescription instance containing a Node with package 'micro_ros_agent'
    and executable 'micro_ros_agent'
    """
    return LaunchDescription(
        [
            Node(
                package="micro_ros_agent",
                executable="micro_ros_agent",
                arguments=["udp4", "--port", "8888"],
            ),
        ]
    )


def spawn(robot_name, robot_mode):
    """
    Spawn the robot in the current Gazebo world.

    :return: list of launch processes
    """
    model.spawn_args(robot_name, mode=robot_mode)
    spawn_launch_path = os.path.join(
        get_package_share_directory("minipock_description"), "launch", "spawn.launch.py"
    )
    spawn_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_launch_path),
        launch_arguments={
            "robot_name": robot_name,
        }.items(),
    )
    launch_processes = [spawn_description]

    return launch_processes


def state(robot_name):
    """
    :return: LaunchDescription instance containing a Node with package 'joint_state_publisher'
    and executable 'joint_state_publisher'
    """
    return LaunchDescription(
        [
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                namespace=robot_name,
            ),
        ]
    )


def odometry(robot_name):
    """
    :return: LaunchDescription instance containing a Node with package 'joint_state_publisher'
    and executable 'joint_state_publisher'
    """
    return LaunchDescription(
        [
            Node(
                package="minipock_bringup",
                executable="raw_data_transformer",
                namespace=robot_name,
                parameters=[
                    {"robot_name": robot_name},
                ],
            ),
        ]
    )


def generate_launch_description():
    """
    Generates a launch description for spawning a robot in a specific position and orientation.

    :return: A LaunchDescription object.
    """
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "odom_frame", default_value="true", description="Publish odom frame"
            ),
            OpaqueFunction(function=parse_config),
        ]
    )

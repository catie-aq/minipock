"""
This file contains the definitions of three main functions that are used to generate the launch
descriptions for different robots, based on provided configurations.

Functions:
- parse_config: This function is employed to parse the configuration file.
- spawn: This function is tasked with generating the LaunchDescription tasks for each robot.
- generate_launch_description: This function is in charge of generating the launch description.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from minipock_description import config


def parse_config(context, *args, **kwargs):
    """
    Parse the configuration for launching processes

    :param context: the context for launching processes
    :param args: variable number of additional arguments
    :param kwargs: variable number of additional keyword arguments
    :return: a list of launch processes
    """
    robot_name = LaunchConfiguration("robot_name").perform(context)
    config_dict = config.config()
    use_sim_time = str(config_dict["use_sim_time"])
    namespace = config_dict["namespace"]
    launch_processes = []
    launch_processes.extend(spawn(use_sim_time, robot_name))
    return launch_processes


def spawn(use_sim_time, robot_name):
    """
    This method is used to spawn a robot in a simulation environment.
    It reads the URDF file of the robot model and generates launch processes to start the
    robot_state_publisher node.

    :param robot_name: The name of the robot.
    :param use_sim_time: A boolean flag to enable simulation time.
    :return: A list of launch processes.
    """
    # robot_state_publisher
    model_dir = os.path.join(get_package_share_directory(f"minipock_description"), "models/tmp")
    urdf_file = os.path.join(model_dir, f"{robot_name}_model.urdf")
    with open(urdf_file) as input_file:
        robot_desc = input_file.read()
    params = {"use_sim_time": bool(use_sim_time), "robot_description": robot_desc}
    nodes = [
        Node(
            package="robot_state_publisher",
            namespace=robot_name,
            executable="robot_state_publisher",
            output="both",
            parameters=[params],
        )
    ]
    group_action = GroupAction(nodes)
    launch_processes = [group_action]
    return launch_processes


def generate_launch_description():
    """
    Generates a launch description for spawning a robot in a specific position and orientation.

    :return: A LaunchDescription object.
    """
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_name",
                default_value="",
                description="The namespace of the robot",
            ),
            OpaqueFunction(function=parse_config),
        ]
    )

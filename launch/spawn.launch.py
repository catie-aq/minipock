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
from launch_ros.actions import PushRosNamespace

import minipock_description.model


def parse_config(context, *args, **kwargs):
    """
    Parse the configuration for launching processes

    :param context: the context for launching processes
    :param args: variable number of additional arguments
    :param kwargs: variable number of additional keyword arguments
    :return: a list of launch processes
    """
    x_pos = LaunchConfiguration('x').perform(context)
    y_pos = LaunchConfiguration('y').perform(context)
    z_pos = LaunchConfiguration('z').perform(context)
    r_rot = LaunchConfiguration('R').perform(context)
    p_rot = LaunchConfiguration('P').perform(context)
    y_rot = LaunchConfiguration('Y').perform(context)
    robot_name = LaunchConfiguration('robot_name').perform(context)
    position = [x_pos, y_pos, z_pos, r_rot, p_rot, y_rot]
    launch_processes = []
    launch_processes.extend(spawn(position, robot_name))
    return launch_processes


def spawn(position, robot_name):
    """
    This method is used to spawn a robot in a simulation environment. It takes two parameters:
    `position` and `robot_name`.

    :param position: The position where the robot should be spawned.
    :param robot_name: The name of the robot.

    :return: A list of launch processes that need to be executed to spawn the robot.
    """
    launch_processes = [Node(package='ros_gz_sim', executable='create', output='screen',
                             arguments=minipock_description.model.spawn_args(position))]
    # robot_state_publisher
    model_dir = os.path.join(get_package_share_directory('minipock_description'), 'models/tmp')
    urdf_file = os.path.join(model_dir, 'model.urdf')
    with open(urdf_file) as input_file:
        robot_desc = input_file.read()
    params = {'use_sim_time': True, 'robot_description': robot_desc,
              'namespace': robot_name}
    nodes = [
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='both',
             parameters=[params], remappings=[('/joint_states', f'/{robot_name}/joint_states')])]
    group_action = GroupAction([PushRosNamespace(robot_name), *nodes])
    launch_processes.append(group_action)
    return launch_processes


def generate_launch_description():
    """
    Generates a launch description for spawning a robot in a specific position and orientation.

    :return: A LaunchDescription object.
    """
    return LaunchDescription([DeclareLaunchArgument('x',
                                                    default_value='0',
                                                    description='X position to spawn'),
                              DeclareLaunchArgument('y',
                                                    default_value='0',
                                                    description='y position to spawn'),
                              DeclareLaunchArgument('z',
                                                    default_value='1.0',
                                                    description='z position to spawn'),
                              DeclareLaunchArgument('R',
                                                    default_value='0',
                                                    description='R rotation to spawn'),
                              DeclareLaunchArgument('P',
                                                    default_value='0',
                                                    description='P rotation to spawn'),
                              DeclareLaunchArgument('Y',
                                                    default_value='0',
                                                    description='Y rotation to spawn'),
                              DeclareLaunchArgument('robot_name',
                                                    default_value='minipock',
                                                    description='Robot name'),
                              OpaqueFunction(function=parse_config)])

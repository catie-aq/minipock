"""
This script manages the launching process of the Gazebo simulator for the Minipock robot.

It sets the initial position parameters for the robot in the Gazebo world,
runs the simulation, and spawns the robot-related processes.

It also bridges ROS messages and Gazebo simulator information.
"""

import os

import minipock_description.model
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from minipock_gz import bridges

robot_name = 'minipock'


def parse_config(context, *args, **kwargs):
    """
    Parse the launch arguments and spawn the robot in the Gazebo world.

    :param context: LaunchContext with arguments
    :return: list of launch processes
    """
    x_pos = LaunchConfiguration('x').perform(context)
    y_pos = LaunchConfiguration('y').perform(context)
    z_pos = LaunchConfiguration('z').perform(context)
    r_rot = LaunchConfiguration('R').perform(context)
    p_rot = LaunchConfiguration('P').perform(context)
    y_rot = LaunchConfiguration('Y').perform(context)
    position = [x_pos, y_pos, z_pos, r_rot, p_rot, y_rot]
    world = LaunchConfiguration('world').perform(context)
    extra_gz_args = LaunchConfiguration('extra_gz_args').perform(context)
    launch_processes = []
    launch_processes.extend(simulation(world_name=world, extra_gz_args=extra_gz_args))
    launch_processes.append(lidar_process())
    launch_processes.extend(spawn(position))
    launch_processes.extend(bridge(world_name=world))
    return launch_processes


def lidar_process():
    """
    This function returns a lidar process wrapped within a LaunchDescription object.

    :return: LaunchDescription object containing the lidar process.
    """
    return LaunchDescription([
        Node(
            package='minipock_gz',
            executable='lidar_process',
        )
    ])


def simulation(world_name, extra_gz_args):
    """
    This method takes in the name of a world file, a boolean indicating whether the simulation
    should start in a paused state, and any additional command line arguments to pass to the
    Gazebo simulator.
    It returns a LaunchDescription object that can be used to start the simulation.

    :param world_name: the name of the world file to load in the simulation
    :param extra_gz_args: additional command line arguments to pass to the Gazebo simulator
    :return: a list containing a LaunchDescription for starting the simulation
    """
    gz_args = ['-r', extra_gz_args, f'{world_name}.sdf']

    gz_sim = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
        launch_arguments={'gz_args': ' '.join(gz_args)}.items())
    return [gz_sim]


def spawn(position):
    """
    Spawn the robot in the current Gazebo world.

    :param position: list of a position and rotation
    :return: list of launch processes
    """
    launch_processes = [Node(package='ros_gz_sim', executable='create', output='screen',
                             arguments=minipock_description.model.spawn_args(position))]
    spawn_launch_path = os.path.join(get_package_share_directory('minipock_description'),
                                     'launch',
                                     'spawn.launch.py')
    spawn_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_launch_path),
        launch_arguments={
            'position': position,
            'robot_name': robot_name,
        }.items()
    )
    launch_processes.append(spawn_description)

    return launch_processes


def bridge(world_name):
    """
    This function manages the bridge between ROS messages and Gazebo simulator.

    :param world_name: the name of the Gazebo world file
    :return: a list of nodes
    """
    bridges_list = [bridges.clock(),
                    bridges.pose(model_name=robot_name),
                    bridges.joint_states(model_name=robot_name, world_name=world_name),
                    bridges.odometry(model_name=robot_name), bridges.cmd_vel(),
                    bridges.scan_lidar(),
                    bridges.tf()]
    nodes = [Node(package='ros_gz_bridge',
                  executable='parameter_bridge',
                  output='screen',
                  arguments=[bridge_name.argument() for bridge_name in bridges_list],
                  remappings=[bridge_name.remapping() for bridge_name in bridges_list])]
    return nodes


def generate_launch_description():
    """
    Generate the launch description for spawning an object in a simulation.

    :return: LaunchDescription object.
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
                              DeclareLaunchArgument('world', default_value='minipock_world',
                                                    description='Name of world'),
                              DeclareLaunchArgument('paused', default_value='False',
                                                    description='True to start the simulation '
                                                                'paused. '),
                              DeclareLaunchArgument('extra_gz_args', default_value='',
                                                    description='Additional arguments to be '
                                                                'passed to gz sim. '),
                              OpaqueFunction(function=parse_config)])

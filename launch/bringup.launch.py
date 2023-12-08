import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from minipock_description import model


def parse_config(context, *args, **kwargs):
    """
    :param context: The context object containing information for parsing the config.
    :param args: Additional arguments for the method. Not used in this method.
    :param kwargs: Additional keyword arguments for the method. Not used in this method.
    :return: A list of launch processes.

    The `parse_config` method is used to parse a configuration in the specified context. It
    retrieves the robot name using the LaunchConfiguration class and then performs the necessary
    operations
    * to spawn the robot and get its state.
    """
    robot_name = LaunchConfiguration('robot_name').perform(context)
    launch_processes = []
    launch_processes.extend(spawn(robot_name))
    launch_processes.append(state())
    launch_processes.extend(lidar())
    return launch_processes


def spawn(robot_name):
    """
    Spawn the robot in the current Gazebo world.

    :return: list of launch processes
    """
    model.spawn_args()
    spawn_launch_path = os.path.join(get_package_share_directory('minipock_description'),
                                     'launch',
                                     'spawn.launch.py')
    spawn_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_launch_path),
        launch_arguments={
            'robot_name': robot_name,
        }.items()
    )
    launch_processes = [spawn_description]

    return launch_processes


def state():
    """
    :return: LaunchDescription instance containing a Node with package 'joint_state_publisher'
    and executable 'joint_state_publisher'
    """
    return LaunchDescription([
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
        ),
    ])


def lidar():
    """
    :return:
    """
    lidar_launch_path = os.path.join(get_package_share_directory('hls_lfcd_lds_driver'),
                                     'launch',
                                     'hlds_laser.launch.py')
    lidar_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch_path),
        launch_arguments={
            "frame_id": "lds_01_link",
            "port": "/dev/ttyUSB1",
        }.items()
    )
    launch_processes = [lidar_launch_description]
    return launch_processes


def generate_launch_description():
    """
    Generates a launch description for spawning a robot in a specific position and orientation.

    :return: A LaunchDescription object.
    """
    return LaunchDescription([DeclareLaunchArgument('robot_name',
                                                    default_value='minipock',
                                                    description='Robot name'),
                              OpaqueFunction(function=parse_config)])

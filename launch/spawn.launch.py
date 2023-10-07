# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os

import minipock_description.model
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

from launch import LaunchDescription
from minipock_gz import bridges

robot_name = 'minipock'


def parse_config(context, *args, **kwargs):
    """
    Parse the launch arguments and launch the spawn function
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
    launch_processes.append(map_server())
    launch_processes.extend(spawn(position))
    launch_processes.extend(bridge(world_name=world))
    return launch_processes


def map_server():
    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'yaml_filename': 'test_map.yaml'}])
    ])


def simulation(world_name, extra_gz_args):
    """
    This method takes in the name of a world file, a boolean indicating whether the simulation should start in a
    paused state, and any additional command line arguments to pass to the Gazebo simulator. It returns a
    LaunchDescription object that can be used to start the simulation.

    :param world_name: the name of the world file to load in the simulation
    :param extra_gz_args: additional command line arguments to pass to the Gazebo simulator
    :return: a list containing a LaunchDescription for starting the simulation
    """
    gz_args = []
    gz_args.append('-r')

    gz_args.append(extra_gz_args)

    gz_args.append(f'{world_name}.sdf')

    gz_sim = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
        launch_arguments={'gz_args': ' '.join(gz_args)}.items())
    return [gz_sim]


def spawn(position):
    """
    Spawn the robot in the current gazebo world

    :param position: list of a position and rotation
    :return: list of launch processes
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
    nodes = [Node(package='robot_state_publisher', executable='robot_state_publisher', output='both',
                  parameters=[params], remappings=[('/joint_states', f'/{robot_name}/joint_states')])]
    group_action = GroupAction([PushRosNamespace(robot_name), *nodes])
    launch_processes.append(group_action)
    return launch_processes


def bridge(world_name):
    bridges_list = [bridges.clock(),
                    bridges.pose(model_name=robot_name),
                    bridges.joint_states(model_name=robot_name, world_name=world_name),
                    bridges.odometry(model_name=robot_name), bridges.cmd_vel(model_name=robot_name)]
    nodes = [Node(package='ros_gz_bridge',
                  executable='parameter_bridge',
                  output='screen',
                  arguments=[bridge.argument() for bridge in bridges_list],
                  remappings=[bridge.remapping() for bridge in bridges_list])]
    return nodes


def generate_launch_description():
    """
    Necessary function launched by the ROS launch system
    Define coordinates parameters. use them with x:=12 after the launch command
    """
    return LaunchDescription([DeclareLaunchArgument('x', default_value='0', description='X position to spawn'),
                              DeclareLaunchArgument('y', default_value='0', description='y position to spawn'),
                              DeclareLaunchArgument('z', default_value='1.0', description='z position to spawn'),
                              DeclareLaunchArgument('R', default_value='0', description='R rotation to spawn'),
                              DeclareLaunchArgument('P', default_value='0', description='P rotation to spawn'),
                              DeclareLaunchArgument('Y', default_value='0', description='Y rotation to spawn'),
                              DeclareLaunchArgument('world', default_value='minipock_world',
                                                    description='Name of world'),
                              DeclareLaunchArgument('paused', default_value='False',
                                                    description='True to start the simulation paused. '),
                              DeclareLaunchArgument('extra_gz_args', default_value='',
                                                    description='Additional arguments to be passed to gz sim. '),
                              OpaqueFunction(function=parse_config)])

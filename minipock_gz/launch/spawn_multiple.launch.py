"""
This script manages the launching process of the Gazebo simulator for the Minipock robot.

It sets the initial position parameters for the robot in the Gazebo world,
runs the simulation, and spawns the robot-related processes.

It also bridges ROS messages and Gazebo simulator information.
"""

import math
import os

import minipock_description.model_v2
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from minipock_gz import bridges_multiple as bridges


def parse_config(context, *args, **kwargs):
    """
    Parse the launch arguments and spawn the robot in the Gazebo world.

    :param context: LaunchContext with arguments
    :return: list of launch processes
    """
    nb_robots = int(LaunchConfiguration("nb_robots").perform(context))
    robot_name = LaunchConfiguration("robot_name").perform(context)
    world = LaunchConfiguration("world").perform(context)
    paused = LaunchConfiguration("paused").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    use_sim_time_bool = IfCondition(use_sim_time).evaluate(context)
    extra_gz_args = LaunchConfiguration("extra_gz_args").perform(context)
    launch_processes = []
    launch_processes.extend(
        simulation(
            world_name=world,
            paused=paused,
            extra_gz_args=extra_gz_args,
            use_sim_time=use_sim_time,
        )
    )
    robots = make_robots(nb_robots, robot_name)
    launch_processes.append(lidar_process(use_sim_time=use_sim_time_bool, robots=robots))
    launch_processes.extend(spawn(use_sim_time=use_sim_time, robots=robots))
    launch_processes.extend(
        bridge(world_name=world, robots=robots, use_sim_time=use_sim_time_bool)
    )
    return launch_processes


def generate_spiral_positions(num_entities, spacing=1):
    """
    Generate a list of positions in a spiral pattern.

    :param num_entities: number of entities
    :param spacing: spacing between entities
    :return: list of positions
    """
    positions = []
    first_position = (0, 0)
    if num_entities >= 1:
        positions.append(first_position)
    if num_entities > 1:
        for i in range(num_entities - 1):
            radius = spacing * (i // 8 + 1)
            angle = (i % 8) * (2 * math.pi / 8)
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            positions.append((x, y))
    return positions


def make_robots(nb_robots, robot_name):
    """
    Create a list of robots with their names and positions.

    :param nb_robots: number of robots
    :param robot_name: name of the robot
    :return: list of robots
    """
    robots = []
    positions = generate_spiral_positions(nb_robots)
    for i in range(nb_robots):
        name = f"{robot_name}{i}"
        robot_position_str = f"{positions[i][0]} {positions[i][1]} 0"
        robots.append({"name": name, "position": robot_position_str})
    return robots


def lidar_process(use_sim_time, robots):
    """
    This function returns a lidar process wrapped within a LaunchDescription object.

    :return: LaunchDescription object containing the lidar process.
    """
    nodes = []
    for robot in robots:
        nodes.append(
            Node(
                package="minipock_gz",
                executable="lidar_process_multiple",
                parameters=[
                    {"use_sim_time": use_sim_time, "robot_name": robot["name"]},
                ],
            )
        )

    return LaunchDescription(nodes)


def simulation(world_name, paused, extra_gz_args, use_sim_time):
    """
    This method takes in the name of a world file, a boolean indicating whether the simulation
    should start in a paused state, and any additional command line arguments to pass to the
    Gazebo simulator.
    It returns a LaunchDescription object that can be used to start the simulation.

    :param world_name: the name of the world file to load in the simulation
    :param extra_gz_args: additional command line arguments to pass to the Gazebo simulator
    :return: a list containing a LaunchDescription for starting the simulation
    """
    gz_args = ["-r", extra_gz_args, f"{world_name}.sdf"]
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("ros_gz_sim"), "launch"),
                "/gz_sim.launch.py",
            ]
        ),
        launch_arguments={
            "gz_args": " ".join(gz_args),
            "paused": paused,
            "use_sim_time": use_sim_time,
        }.items(),
    )
    return [gz_sim]


def spawn(use_sim_time, robots):
    """
    Spawn the robot in the current Gazebo world.

    :param position: list of a position and rotation
    :return: list of launch processes
    """
    launch_processes = []
    spawn_launch_path = os.path.join(
        get_package_share_directory("minipock_description"), "launch", "spawn_multiple.launch.py"
    )
    for robot in robots:
        launch_processes.append(
            Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=minipock_description.model_v2.spawn_args(
                    robot_name=robot["name"],
                    robot_position_str=robot["position"],
                ),
            )
        )
        spawn_description = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(spawn_launch_path),
            launch_arguments={
                "robot_name": robot["name"],
                "robot_position_str": robot["position"],
                "use_sim_time": use_sim_time,
            }.items(),
        )
        launch_processes.append(spawn_description)

    return launch_processes


def bridge(world_name, robots, use_sim_time):
    """
    This function manages the bridge between ROS messages and Gazebo simulator.

    :param world_name: the name of the Gazebo world file
    :return: a list of nodes
    """
    bridges_list = [
        bridges.clock(),
    ]
    for robot in robots:
        bridges_list.extend(
            [
                bridges.pose(model_name=robot["name"]),
                bridges.joint_states(model_name=robot["name"], world_name=world_name),
                bridges.odometry(model_name=robot["name"]),
                bridges.cmd_vel(model_name=robot["name"]),
                bridges.scan_lidar(model_name=robot["name"]),
                bridges.tf(model_name=robot["name"]),
            ]
        )
    nodes = [
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            output="screen",
            arguments=[bridge_name.argument() for bridge_name in bridges_list],
            remappings=[bridge_name.remapping() for bridge_name in bridges_list],
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                }
            ],
        )
    ]
    return nodes


def generate_launch_description():
    """
    Generate the launch description for spawning an object in a simulation.

    :return: LaunchDescription object.
    """
    return LaunchDescription(
        [
            DeclareLaunchArgument("nb_robots", default_value="1", description="Number of robots"),
            DeclareLaunchArgument(
                "robot_name", default_value="minipock", description="Name of robot"
            ),
            DeclareLaunchArgument(
                "world", default_value="minipock_world", description="Name of world"
            ),
            DeclareLaunchArgument(
                "paused",
                default_value="False",
                description="True to start the simulation " "paused. ",
            ),
            DeclareLaunchArgument(
                "extra_gz_args",
                default_value="",
                description="Additional arguments to be " "passed to gz sim. ",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="True",
                description="Use simulation time",
            ),
            OpaqueFunction(function=parse_config),
        ]
    )

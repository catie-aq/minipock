#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString


def parse_config(context, *args, **kwargs):
    """
    Generate the launch description for the application.

    :param context: The context of the launch.
    :return: A LaunchDescription object.
    """
    nb_robots = int(LaunchConfiguration("nb_robots").perform(context))
    robot_name = LaunchConfiguration("robot_name").perform(context)
    start_rviz = LaunchConfiguration("start_rviz").perform(context)
    use_sim_time = IfCondition(LaunchConfiguration("use_sim_time")).evaluate(context)
    autostart = IfCondition(LaunchConfiguration("autostart")).evaluate(context)
    use_composition = IfCondition(LaunchConfiguration("use_composition")).evaluate(context)
    use_respawn = IfCondition(LaunchConfiguration("use_respawn")).evaluate(context)
    bringup = IfCondition(LaunchConfiguration("bringup")).evaluate(context)

    map_yaml_file = LaunchConfiguration(
        "map_yaml_file",
        default=PathJoinSubstitution(
            [FindPackageShare("minipock_navigation2"), "map", "map.yaml"]
        ),
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("minipock_navigation2"), "rviz", "navigation2.rviz"]
    )

    robots = make_robots(nb_robots, robot_name)

    namespaced_rviz_config_file = ReplaceString(
        source_file=rviz_config_file,
        replacements={
            "<relative_namespace>": robots[0]["name"],
            "<absolute_namespace>": "/" + robots[0]["name"],
        },
    )

    minipock_bringup = None
    if bringup:
        minipock_bringup_file_dir = PathJoinSubstitution(
            [
                FindPackageShare("minipock_bringup"),
                "launch",
            ]
        )
        minipock_bringup = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([minipock_bringup_file_dir, "/bringup.launch.py"]),
            launch_arguments={
                "use_sim_time": use_sim_time,
            }.items(),
        )

    localization_nodes = launch_localization(
        robots, use_sim_time, autostart, use_respawn, map_yaml_file
    )

    navigation_nodes = launch_navigation(
        robots, use_sim_time, autostart, use_respawn, use_composition
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", namespaced_rviz_config_file],
        output="screen",
        condition=IfCondition(start_rviz),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    launch_actions = [
        localization_nodes,
        navigation_nodes,
        rviz_node,
    ]
    if minipock_bringup is not None:
        launch_actions.append(minipock_bringup)

    return launch_actions


def make_robots(nb_robots, robot_name):
    """
    Create a list of robots with their names and positions.

    :param nb_robots: number of robots
    :param robot_name: name of the robot
    :return: list of robots
    """
    robots = []
    for i in range(nb_robots):
        name = f"{robot_name}{i}"
        if nb_robots == 1:
            name = robot_name
        robots.append({"name": name})
    return robots


def launch_localization(robots, use_sim_time, autostart, use_respawn, map_yaml_file):
    """
    Generate the launch description for localization.

    :param robots: list of robots containing their names
    :param use_sim_time: use simulation time
    :param autostart: autostart the nodes
    :param use_respawn: respawn the nodes
    :param map_yaml_file: path to the map yaml file
    :return: list of launch actions
    """
    launch_map_server = LaunchDescription(
        [
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"topic_name": "/map"},
                    {"frame_id": "map"},
                    {"yaml_filename": map_yaml_file},
                ],
            ),
        ]
    )

    localization_lifecycle_nodes = ["map_server"]

    launch_amcl = LaunchDescription()
    for robot in robots:
        namespace = robot["name"]
        absolute_namespace = namespace
        if namespace != "":
            absolute_namespace = "/" + namespace
        params_file = LaunchConfiguration(
            "params_file",
            default=PathJoinSubstitution(
                [FindPackageShare("minipock_navigation2"), "param", "minipock.yaml"]
            ),
        )
        params_file = ReplaceString(
            source_file=params_file,
            replacements={
                "<absolute_namespace>": absolute_namespace,
                "<relative_namespace>": namespace,
                "<use_sim_time>": str(use_sim_time),
            },
        )

        launch_amcl.add_action(
            Node(
                namespace=namespace,
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[
                    params_file,
                ],
            )
        )
        localization_lifecycle_nodes.append(f"{namespace}/amcl")

    launch_lifecycle_manager_localization = LaunchDescription(
        [
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[
                    {"autostart": autostart},
                    {"node_names": localization_lifecycle_nodes},
                    {"use_sim_time": use_sim_time},
                    {"bond_timeout": 0.0},
                ],
            ),
        ]
    )
    launch_localization = LaunchDescription(
        [launch_map_server, launch_amcl, launch_lifecycle_manager_localization]
    )
    return launch_localization


def standalone_navigation_nodes(config):
    """
    Generate the launch description for standalone navigation.

    :param config: dictionary containing the configuration parameters
    :return: list of launch actions
    """
    namespace = config["namespace"]
    use_sim_time = config["use_sim_time"]
    use_respawn = config["use_respawn"]
    autostart = config["autostart"]
    params_file = config["params_file"]
    nodes_info = config["nodes_info"]
    collision_monitor = config["collision_monitor"]
    lifecycle_manager = config["lifecycle_manager"]
    navigation_lifecycle_nodes = config["navigation_lifecycle_nodes"]

    standalone_navigation = LaunchDescription()
    for node_info in nodes_info:
        standalone_navigation.add_action(
            Node(
                namespace=namespace,
                package=node_info["package"],
                executable=node_info["executable"],
                name=node_info["name"],
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[params_file],
            )
        )
    standalone_navigation.add_action(
        Node(
            namespace=namespace,
            package=collision_monitor["package"],
            executable=collision_monitor["executable"],
            name=collision_monitor["name"],
            output="screen",
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=collision_monitor["params"],
        )
    )
    standalone_navigation.add_action(
        Node(
            package=lifecycle_manager["package"],
            executable=lifecycle_manager["executable"],
            name=lifecycle_manager["name"],
            output="screen",
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[
                {"autostart": autostart},
                {"node_names": navigation_lifecycle_nodes},
                {"use_sim_time": use_sim_time},
                {"bond_timeout": 0.0},
            ],
        ),
    )
    return standalone_navigation


def composed_navigation_nodes(config):
    """
    Generate the launch description for composed navigation in a container.

    :param config: dictionary containing the configuration parameters
    :return: list of launch actions
    """
    namespace = config["namespace"]
    use_sim_time = config["use_sim_time"]
    autostart = config["autostart"]
    params_file = config["params_file"]
    nodes_info = config["nodes_info"]
    collision_monitor = config["collision_monitor"]
    lifecycle_manager = config["lifecycle_manager"]
    navigation_lifecycle_nodes = config["navigation_lifecycle_nodes"]

    composed_navigation = LaunchDescription()
    composed_navigation.add_action(
        GroupAction(
            [
                PushRosNamespace(namespace),
                Node(
                    package="rclcpp_components",
                    executable="component_container_isolated",
                    name="nav2_container",
                    output="screen",
                    parameters=[
                        params_file,
                        {"autostart": autostart},
                        {"use_sim_time": use_sim_time},
                    ],
                ),
            ]
        )
    )
    composable_nodes = []
    for node_info in nodes_info:
        composable_nodes.append(
            ComposableNode(
                package=node_info["package"],
                plugin=node_info["plugin"],
                name=node_info["name"],
                namespace=namespace,
                parameters=[params_file],
            )
        )
    composable_nodes.append(
        ComposableNode(
            package=collision_monitor["package"],
            plugin=collision_monitor["plugin"],
            name=collision_monitor["name"],
            namespace=namespace,
            parameters=collision_monitor["params"],
        )
    )
    composable_nodes.extend(
        [
            ComposableNode(
                package=lifecycle_manager["package"],
                plugin=lifecycle_manager["plugin"],
                name=lifecycle_manager["name"],
                parameters=[
                    {"autostart": autostart},
                    {"node_names": navigation_lifecycle_nodes},
                    {"use_sim_time": use_sim_time},
                    {"bond_timeout": 0.0},
                ],
            )
        ]
    )
    load_composable_nodes = GroupAction(
        [
            LoadComposableNodes(
                target_container=f"{namespace}/nav2_container",
                composable_node_descriptions=composable_nodes,
            ),
        ]
    )
    composed_navigation.add_action(load_composable_nodes)
    return composed_navigation


def launch_navigation(robots, use_sim_time, autostart, use_respawn, use_composition):
    """
    Generate the launch description for navigation.

    :param robots: list of robots containing their names
    :param use_sim_time: use simulation time
    :param autostart: autostart the nodes
    :param use_respawn: respawn the nodes
    :return: list of launch actions
    """
    launch_navigation = LaunchDescription()
    for robot in robots:
        namespace = robot["name"]
        absolute_namespace = namespace
        if namespace != "":
            absolute_namespace = "/" + namespace

        params_file = LaunchConfiguration(
            "params_file",
            default=PathJoinSubstitution(
                [FindPackageShare("minipock_navigation2"), "param", "minipock.yaml"]
            ),
        )
        configured_params_file = ReplaceString(
            source_file=params_file,
            replacements={
                "<absolute_namespace>": absolute_namespace,
                "<relative_namespace>": namespace,
                "<use_sim_time>": str(use_sim_time),
            },
        )
        navigation_nodes_info = [
            {
                "package": "nav2_controller",
                "executable": "controller_server",
                "plugin": "nav2_controller::ControllerServer",
                "name": "controller_server",
            },
            {
                "package": "nav2_smoother",
                "executable": "smoother_server",
                "plugin": "nav2_smoother::SmootherServer",
                "name": "smoother_server",
            },
            {
                "package": "nav2_planner",
                "executable": "planner_server",
                "plugin": "nav2_planner::PlannerServer",
                "name": "planner_server",
            },
            {
                "package": "nav2_behaviors",
                "executable": "behavior_server",
                "plugin": "behavior_server::BehaviorServer",
                "name": "behavior_server",
            },
            {
                "package": "nav2_bt_navigator",
                "executable": "bt_navigator",
                "plugin": "nav2_bt_navigator::BtNavigator",
                "name": "bt_navigator",
            },
            {
                "package": "nav2_waypoint_follower",
                "executable": "waypoint_follower",
                "plugin": "nav2_waypoint_follower::WaypointFollower",
                "name": "waypoint_follower",
            },
            {
                "package": "nav2_velocity_smoother",
                "executable": "velocity_smoother",
                "plugin": "nav2_velocity_smoother::VelocitySmoother",
                "name": "velocity_smoother",
            },
        ]
        collision_monitor = {
            "package": "nav2_collision_monitor",
            "executable": "collision_monitor",
            "plugin": "nav2_collision_monitor::CollisionMonitor",
            "name": "collision_monitor",
            "params": [
                {"base_frame_id": f"{namespace}/base_footprint"},
                {"odom_frame_id": f"{namespace}/odom"},
                {"use_sim_time": use_sim_time},
                {"cmd_vel_in_topic": "cmd_vel_smoothed"},
                {"cmd_vel_out_topic": "cmd_vel"},
                {"state_topic": "collision_monitor_state"},
                {"transform_tolerance": 0.2},
                {"source_timeout": 1.0},
                {"base_shift_correction": True},
                {"stop_pub_timeout": 2.0},
                {"polygons": ["FootprintApproach"]},
                {
                    "FootprintApproach": {
                        "type": "polygon",
                        "action_type": "approach",
                        "footprint_topic": f"{namespace}/local_costmap/published_footprint",
                        "time_before_collision": 1.2,
                        "simulation_time_step": 0.1,
                        "min_points": 6,
                        "visualize": False,
                        "enabled": True,
                    }
                },
                {"observation_sources": ["laser_scan"]},
                {
                    "laser_scan": {
                        "type": "scan",
                        "topic": f"{namespace}/scan",
                        "min_height": 0.15,
                        "max_height": 2.0,
                        "enabled": True,
                    }
                },
            ],
        }
        lifecycle_manager = {
            "package": "nav2_lifecycle_manager",
            "executable": "lifecycle_manager",
            "plugin": "nav2_lifecycle_manager::LifecycleManager",
            "name": f"lifecycle_manager_navigation",
        }

        navigation_lifecycle_nodes = []
        for node_info in navigation_nodes_info:
            navigation_lifecycle_nodes.append(f"{namespace}/{node_info['name']}")
        navigation_lifecycle_nodes.append(f"{namespace}/{collision_monitor['name']}")

        config = {
            "namespace": namespace,
            "params_file": configured_params_file,
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "use_respawn": use_respawn,
            "nodes_info": navigation_nodes_info,
            "collision_monitor": collision_monitor,
            "lifecycle_manager": lifecycle_manager,
            "navigation_lifecycle_nodes": navigation_lifecycle_nodes,
        }
        if use_composition:
            composed_navigation = composed_navigation_nodes(config)
            launch_navigation.add_action(composed_navigation)
        else:
            standalone_navigation = standalone_navigation_nodes(config)
            launch_navigation.add_action(standalone_navigation)

    return launch_navigation


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
                "start_rviz", default_value="true", description="Whether execute rviz2"
            ),
            DeclareLaunchArgument(
                "use_sim_time", default_value="false", description="Set use_sim_time"
            ),
            DeclareLaunchArgument(
                "bringup",
                default_value="true",
                description="Whether to bringup minipock",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="false",
                description="Automatically startup the nav2 stack",
            ),
            DeclareLaunchArgument(
                "use_composition",
                default_value="True",
                description="Whether to use composed bringup",
            ),
            DeclareLaunchArgument(
                "use_respawn",
                default_value="false",
                description="Whether to respawn if a node crashes. \
                Applied when composition is disabled.",
            ),
            OpaqueFunction(function=parse_config),
        ]
    )

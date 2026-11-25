"""Fleet localization: one shared map server, one AMCL per robot."""

from launch import Action
from launch.actions import GroupAction
from launch_ros.actions import Node, SetRemap
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml

MAP_TOPIC = "/map"
MAP_FRAME = "map"


def localization_actions(
    robots: list[tuple[str, str]], map_yaml: str, *, use_sim_time: bool
) -> list[Action]:
    """Return the localization actions of the fleet.

    One `map_server` publishes the map on `/map` in the `map` frame, managed by
    `lifecycle_manager_map`; both are in the root namespace, even when the actions are included
    under a pushed namespace. Each robot runs `amcl` in its namespace on the global `/tf`,
    managed by its own `<namespace>/lifecycle_manager_localization`.

    A lifecycle manager only bonds with servers of its own namespace (the server publishes its
    bond on `<namespace>/bond` under its short name), hence one manager per namespace, as
    nav2_bringup does for navigation. Bonds stay active: a crashed server is detected.

    Args:
        robots: `(namespace, params_file)` of each robot, the parameter file being rendered by
            `minipock_bringup.nav2_params.render_params_file` (root keys without namespace).
        map_yaml: Path of the map YAML file.
        use_sim_time: Whether the nodes use the simulation clock.

    Returns:
        The launch actions.
    """
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        namespace="/",
        output="screen",
        parameters=[
            {
                "yaml_filename": map_yaml,
                "topic_name": MAP_TOPIC,
                "frame_id": MAP_FRAME,
                "use_sim_time": use_sim_time,
            }
        ],
    )
    amcls = []
    for namespace, params_file in robots:
        amcl = Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            namespace=namespace,
            output="screen",
            parameters=[
                ParameterFile(
                    RewrittenYaml(
                        source_file=params_file,
                        root_key=namespace,
                        param_rewrites={},
                        convert_types=True,
                    ),
                    allow_substs=True,
                ),
                {"use_sim_time": use_sim_time},
            ],
        )
        amcls += [
            amcl,
            _lifecycle_manager(
                "lifecycle_manager_localization", namespace, ["amcl"], use_sim_time=use_sim_time
            ),
        ]
    return [
        map_server,
        _lifecycle_manager(
            "lifecycle_manager_map", "/", ["map_server"], use_sim_time=use_sim_time
        ),
        GroupAction(
            [SetRemap("/tf", "/tf"), SetRemap("/tf_static", "/tf_static"), *amcls],
            scoped=True,
        ),
    ]


def _lifecycle_manager(
    name: str, namespace: str, node_names: list[str], *, use_sim_time: bool
) -> Node:
    """Return a Nav2 lifecycle manager that brings up `node_names` of its namespace."""
    return Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name=name,
        namespace=namespace,
        output="screen",
        parameters=[
            {"autostart": True, "node_names": node_names, "use_sim_time": use_sim_time},
        ],
    )

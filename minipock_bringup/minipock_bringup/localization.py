"""Fleet localization: one shared map server, one AMCL per robot."""

from launch import Action
from launch.actions import GroupAction
from launch_ros.actions import Node, SetRemap
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml

MAP_TOPIC = "/map"
MAP_FRAME = "map"
# Bond disabled: a server creates its bond on `<its namespace>/bond` with its bare name
# (`amcl`), the root-namespace manager waits on `/bond` for `minipock_0/amcl`. They never
# connect, so the manager would abort the bringup.
BOND_TIMEOUT = 0.0


def localization_actions(
    robots: list[tuple[str, str]], map_yaml: str, *, use_sim_time: bool
) -> list[Action]:
    """Return the localization actions of the fleet.

    One `map_server` publishes the map on `/map` in the `map` frame; each robot runs `amcl` in
    its namespace on the global `/tf`; one `lifecycle_manager_localization` manages them all.
    The map server and the lifecycle manager are in the root namespace, even when the actions
    are included under a pushed namespace.

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
    amcls = [
        Node(
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
        for namespace, params_file in robots
    ]
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        namespace="/",
        output="screen",
        parameters=[
            {
                "autostart": True,
                "bond_timeout": BOND_TIMEOUT,
                "node_names": ["map_server", *(f"{namespace}/amcl" for namespace, _ in robots)],
                "use_sim_time": use_sim_time,
            }
        ],
    )
    return [
        map_server,
        GroupAction(
            [SetRemap("/tf", "/tf"), SetRemap("/tf_static", "/tf_static"), *amcls],
            scoped=True,
        ),
        lifecycle_manager,
    ]

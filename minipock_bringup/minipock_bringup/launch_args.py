"""Launch arguments and includes of the per-robot launches used by the entry launches."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from minipock_bringup.config import FleetConfig, Robot
from minipock_bringup.nav2_params import render_params_file

# Each simulated robot hides its own LD19 mesh from its LiDAR with one Gazebo visibility bit
# (see `minipock_description/urdf/sensors.xacro`): 18 bits are free.
MAX_SIM_ROBOTS = 18


def robot_launch_arguments(
    robot: Robot, cfg: FleetConfig, *, use_sim_time: bool
) -> dict[str, str]:
    """Return the launch arguments of one robot, its Nav2 parameter file rendered.

    Args:
        robot: The robot, one of `cfg.robots`.
        cfg: The fleet configuration the robot comes from.
        use_sim_time: Whether the robot nodes use the simulation clock.

    Returns:
        `namespace`, `mode`, `x`, `y`, `yaw`, `sim_index` (position of the robot in
        `cfg.robots`), `params_file` and `use_sim_time` (`true` or `false`), as strings.

    Raises:
        ValueError: If `robot` is not part of `cfg`, or if a simulated fleet has more than
            `MAX_SIM_ROBOTS` robots.
    """
    if robot not in cfg.robots:
        msg = f"Robot {robot.name} is not part of the fleet configuration"
        raise ValueError(msg)
    if use_sim_time and len(cfg.robots) > MAX_SIM_ROBOTS:
        msg = (
            f"A simulated fleet is limited to {MAX_SIM_ROBOTS} robots "
            f"(one LiDAR visibility bit each), got {len(cfg.robots)}"
        )
        raise ValueError(msg)
    params_file = render_params_file(
        robot.mode, robot.name, robot.x, robot.y, robot.yaw, override=robot.nav2_params
    )
    return {
        "namespace": robot.name,
        "mode": robot.mode,
        "x": str(robot.x),
        "y": str(robot.y),
        "yaw": str(robot.yaw),
        "sim_index": str(cfg.robots.index(robot)),
        "params_file": params_file,
        "use_sim_time": "true" if use_sim_time else "false",
    }


def pick(arguments: dict[str, str], *keys: str) -> dict[str, str]:
    """Return `namespace`, `use_sim_time` and `keys` of robot launch arguments.

    Args:
        arguments: Arguments returned by `robot_launch_arguments`.
        keys: Extra keys the included launch declares.

    Returns:
        The selected arguments.
    """
    return {key: arguments[key] for key in ("namespace", "use_sim_time", *keys)}


def include(package: str, launch_file: str, arguments: dict[str, str]) -> IncludeLaunchDescription:
    """Include a launch file installed in `share/<package>/launch`.

    Args:
        package: Package that installs the launch file.
        launch_file: File name in the `launch` directory of the package.
        arguments: Launch arguments passed to the included file.

    Returns:
        The include action.
    """
    path = Path(get_package_share_directory(package)) / "launch" / launch_file
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(path)), launch_arguments=arguments.items()
    )

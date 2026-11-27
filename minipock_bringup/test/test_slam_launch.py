"""Construction of the entry launches with and without `slam:=true` (no process started)."""

import importlib.util
from pathlib import Path
from types import ModuleType

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import Action, LaunchContext
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node

from minipock_bringup.localization import MAP_TOPIC, slam_actions

DATA = Path(__file__).parent / "data"
LAUNCH_DIR = Path(get_package_share_directory("minipock_bringup")) / "launch"


def _launch_module(name: str) -> ModuleType:
    spec = importlib.util.spec_from_file_location(name, LAUNCH_DIR / f"{name}.launch.py")
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _one_robot_fleet(tmp_path: Path) -> str:
    content = yaml.safe_load((DATA / "two_robots.yaml").read_text())
    first = next(iter(content["fleet"]))
    content["fleet"] = {first: content["fleet"][first]}
    path = tmp_path / "one_robot.yaml"
    path.write_text(yaml.safe_dump(content))
    return str(path)


def _flatten(actions: list[Action]) -> list[Action]:
    flat = []
    for action in actions:
        flat.append(action)
        if isinstance(action, GroupAction):
            flat += _flatten(action.get_sub_entities())
    return flat


def _fleet_actions(launch: str, config: str, *, slam: str) -> list[Action]:
    context = LaunchContext()
    context.launch_configurations.update(
        {"config": config, "slam": slam, "rviz": "false", "headless": "true"}
    )
    return _flatten(_launch_module(launch)._fleet(context))


def _packages(actions: list[Action]) -> set[str]:
    return {action.node_package for action in actions if isinstance(action, Node)}


def _includes(actions: list[Action]) -> dict[str, IncludeLaunchDescription]:
    """Return the includes by launch file name (loading each file expands its location)."""
    includes = {}
    for action in actions:
        if isinstance(action, IncludeLaunchDescription):
            source = action.launch_description_source
            source.get_launch_description(LaunchContext())
            includes[Path(source.location).name] = action
    return includes


def _include_arguments(action: IncludeLaunchDescription) -> dict[str, str]:
    context = LaunchContext()

    def text(value: object) -> str:
        return perform_substitutions(context, normalize_to_list_of_substitutions(value))

    return {text(key): text(value) for key, value in action.launch_arguments}


@pytest.mark.parametrize("launch", ["sim", "real"])
def test_slam_replaces_map_server_and_amcl(launch: str, tmp_path: Path) -> None:
    actions = _fleet_actions(launch, _one_robot_fleet(tmp_path), slam="true")
    packages = _packages(actions)
    assert "nav2_map_server" not in packages
    assert "nav2_amcl" not in packages
    includes = _includes(actions)
    assert "cartographer.launch.py" in includes
    assert _include_arguments(includes["cartographer.launch.py"]) == {
        "namespace": "minipock_0",
        "use_sim_time": "true" if launch == "sim" else "false",
        "map_topic": MAP_TOPIC,
        "start_rviz": "false",
    }
    # Nav2 still runs for the robot, on the map Cartographer publishes.
    assert "robot.launch.py" in includes


@pytest.mark.parametrize("launch", ["sim", "real"])
def test_default_localizes_on_the_map(launch: str) -> None:
    actions = _fleet_actions(launch, str(DATA / "two_robots.yaml"), slam="false")
    packages = _packages(actions)
    assert {"nav2_map_server", "nav2_amcl"} <= packages
    assert "cartographer.launch.py" not in _includes(actions)


@pytest.mark.parametrize("launch", ["sim", "real"])
def test_slam_rejects_several_robots(launch: str) -> None:
    with pytest.raises(ValueError, match=r"single robot, the fleet has 2"):
        _fleet_actions(launch, str(DATA / "two_robots.yaml"), slam="true")


def test_slam_actions_need_one_robot() -> None:
    with pytest.raises(ValueError, match="minipock_0, minipock_1"):
        slam_actions(["minipock_0", "minipock_1"], use_sim_time=True)
    with pytest.raises(ValueError, match="has 0"):
        slam_actions([], use_sim_time=True)

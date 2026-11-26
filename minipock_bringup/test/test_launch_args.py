"""Tests for the per-robot launch arguments built by the entry launches."""

from pathlib import Path

import pytest
import yaml

from minipock_bringup.config import FleetConfig, Robot
from minipock_bringup.launch_args import MAX_SIM_ROBOTS, robot_launch_arguments

ROBOT = Robot(name="minipock_0", mode="holonomic", x=1.5, y=-2.0, yaw=0.5, nav2_params="")
CONFIG = FleetConfig(
    namespace="minipock_",
    world="office",
    map="office",
    rviz="fleet",
    robots=(ROBOT,),
)


def test_use_sim_time_sim() -> None:
    assert robot_launch_arguments(ROBOT, CONFIG, use_sim_time=True)["use_sim_time"] == "true"


def test_use_sim_time_real() -> None:
    assert robot_launch_arguments(ROBOT, CONFIG, use_sim_time=False)["use_sim_time"] == "false"


def test_pose_forwarded() -> None:
    arguments = robot_launch_arguments(ROBOT, CONFIG, use_sim_time=True)
    assert arguments["namespace"] == "minipock_0"
    assert arguments["mode"] == "holonomic"
    assert (arguments["x"], arguments["y"], arguments["yaw"]) == ("1.5", "-2.0", "0.5")
    assert all(isinstance(value, str) for value in arguments.values())


def test_params_file_rendered_for_robot() -> None:
    arguments = robot_launch_arguments(ROBOT, CONFIG, use_sim_time=True)
    params = yaml.safe_load(Path(arguments["params_file"]).read_text())
    amcl = params["amcl"]["ros__parameters"]
    assert amcl["base_frame_id"] == "minipock_0/base_footprint"
    assert amcl["initial_pose"]["x"] == pytest.approx(1.5)


def test_robot_outside_fleet_rejected() -> None:
    stranger = Robot(name="minipock_9", mode="differential", x=0.0, y=0.0, yaw=0.0, nav2_params="")
    with pytest.raises(ValueError, match="minipock_9"):
        robot_launch_arguments(stranger, CONFIG, use_sim_time=True)


def _fleet(count: int) -> FleetConfig:
    robots = tuple(
        Robot(
            name=f"minipock_{i}", mode="differential", x=float(i), y=0.0, yaw=0.0, nav2_params=""
        )
        for i in range(count)
    )
    return FleetConfig(
        namespace="minipock_", world="office", map="office", rviz="fleet", robots=robots
    )


def test_sim_index_is_fleet_position() -> None:
    cfg = _fleet(MAX_SIM_ROBOTS)
    indices = [
        robot_launch_arguments(robot, cfg, use_sim_time=True)["sim_index"] for robot in cfg.robots
    ]
    assert indices == [str(i) for i in range(MAX_SIM_ROBOTS)]


def test_sim_fleet_too_large_rejected() -> None:
    cfg = _fleet(MAX_SIM_ROBOTS + 1)
    with pytest.raises(ValueError, match=str(MAX_SIM_ROBOTS)):
        robot_launch_arguments(cfg.robots[0], cfg, use_sim_time=True)


def test_real_fleet_size_not_limited() -> None:
    cfg = _fleet(MAX_SIM_ROBOTS + 1)
    arguments = robot_launch_arguments(cfg.robots[-1], cfg, use_sim_time=False)
    assert arguments["namespace"] == f"minipock_{MAX_SIM_ROBOTS}"

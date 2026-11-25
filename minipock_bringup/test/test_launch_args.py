"""Tests for the per-robot launch arguments built by the entry launches."""

from pathlib import Path

import pytest
import yaml

from minipock_bringup.config import FleetConfig, Robot
from minipock_bringup.launch_args import robot_launch_arguments

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

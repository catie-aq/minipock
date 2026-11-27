"""Tests of the spawn height of a robot (`launch/robot.launch.py`)."""

import importlib.util
from pathlib import Path
from types import ModuleType

ROBOT_LAUNCH = Path(__file__).resolve().parents[1] / "launch" / "robot.launch.py"
# Wheel contact plane below base_footprint (and base_link), checked against the URDF by
# minipock_description/test/test_urdf.py (WHEEL_CONTACT_DEPTH).
WHEEL_CONTACT_DEPTH = 0.0975
MAX_DROP = 0.01


def _robot_launch() -> ModuleType:
    spec = importlib.util.spec_from_file_location("robot_launch", ROBOT_LAUNCH)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_spawn_above_floor() -> None:
    # Spawned lower, the wheels and the chassis start inside the floor mesh and the physics
    # leaves the robot sunk, its chassis rubbing the floor. Spawned much higher, it falls.
    height = float(_robot_launch().SPAWN_HEIGHT)
    assert WHEEL_CONTACT_DEPTH < height <= WHEEL_CONTACT_DEPTH + MAX_DROP

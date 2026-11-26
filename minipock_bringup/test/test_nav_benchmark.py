"""Tests of the metrics and the goals file of nav_benchmark (no simulation)."""

import math
from pathlib import Path

import pytest

from minipock_bringup.nav_benchmark import (
    Goal,
    Measure,
    Row,
    angle_diff,
    load_goals,
    markdown_table,
    path_length,
    sign_changes,
    summarize,
)

GOALS_FILE = Path(__file__).resolve().parent / "data" / "benchmark_goals.yaml"


def test_angle_diff_wraps() -> None:
    assert angle_diff(math.pi - 0.1, -math.pi + 0.1) == pytest.approx(-0.2)
    assert angle_diff(0.3, 0.1) == pytest.approx(0.2)


def test_path_length() -> None:
    assert path_length([(0.0, 0.0), (3.0, 0.0), (3.0, 4.0)]) == pytest.approx(7.0)
    assert path_length([(1.0, 1.0)]) == 0.0


def test_sign_changes_ignores_deadband() -> None:
    assert sign_changes([0.3, 0.01, -0.02, 0.4, -0.3, -0.2, 0.2]) == 2
    assert sign_changes([0.0, 0.01, -0.01]) == 0


def test_summarize_straight_goal() -> None:
    goal = Goal("straight", (3.0, 0.0, 0.0))
    measure = Measure(
        points=[(0.0, 0.0), (1.5, 0.1), (3.0, 0.0)],
        yaws=[0.0, 0.05, -0.1],
        angular_commands=[0.1, -0.1, 0.1],
        min_range=0.5,
    )
    metrics = summarize(measure, goal, (0.0, 0.0, 0.0), (2.95, 0.0, 0.02))
    assert metrics["path_ratio"] == pytest.approx(path_length(measure.points) / 3.0)
    assert metrics["clearance"] == pytest.approx(0.5 - 0.17)
    assert metrics["oscillations"] == pytest.approx(2 / path_length(measure.points))
    assert metrics["yaw_drift"] == pytest.approx(0.1)
    assert metrics["position_error"] == pytest.approx(0.05)
    assert metrics["yaw_error"] == pytest.approx(0.02)


def test_summarize_turn_in_place_has_no_path_ratio() -> None:
    goal = Goal("turn", (1.0, 1.0, math.pi))
    measure = Measure(points=[(1.0, 1.0), (1.0, 1.0)], yaws=[0.0, 3.1], angular_commands=[0.5])
    metrics = summarize(measure, goal, (1.0, 1.0, 0.0), (1.0, 1.0, -3.1))
    assert metrics["path_ratio"] is None
    assert metrics["oscillations"] == 0
    assert metrics["yaw_error"] == pytest.approx(math.pi - 3.1)


def test_benchmark_goals_file() -> None:
    plans = load_goals(GOALS_FILE)
    assert sorted(plans) == ["minipock_0", "minipock_1"]
    names = {robot: [goal.name for goal in plan.goals] for robot, plan in plans.items()}
    for robot in plans:
        assert {"straight_3m", "turn_in_place", "behind_1m", "wall_0.3m"} <= set(names[robot])
    lateral = [goal for goal in plans["minipock_0"].goals if goal.keep_yaw]
    assert [goal.name for goal in lateral] == ["lateral_1m"]
    assert not any(goal.keep_yaw for goal in plans["minipock_1"].goals)


def test_load_goals_rejects_short_pose(tmp_path: Path) -> None:
    path = tmp_path / "goals.yaml"
    path.write_text("r:\n  start: [0, 0, 0]\n  goals:\n    - {name: a, pose: [1, 2]}\n")
    with pytest.raises(ValueError, match=r"\[x, y, yaw\]"):
        load_goals(path)


def test_markdown_table() -> None:
    row = Row(
        robot="minipock_0",
        goal="turn",
        repetition=1,
        succeeded=True,
        duration=4.0,
        path_ratio=None,
        clearance=0.3,
        oscillations=0.0,
        yaw_drift=3.1,
        position_error=0.02,
        yaw_error=0.05,
    )
    table = markdown_table([row]).splitlines()
    assert len(table) == 3
    assert table[2] == "| minipock_0 | turn | 1 | ok | 4.0 | - | 0.30 | 0.0 | 3.10 | 0.02, 0.05 |"

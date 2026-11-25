"""Tests for the odometry helpers of raw_data_transformer."""

import math

import pytest

from minipock_bringup.odometry import Pose2D, body_twist, yaw_from_quaternion


def test_twist_straight_line() -> None:
    twist = body_twist(Pose2D(0.0, 0.0, 0.0), Pose2D(0.1, 0.0, 0.0), 0.1)
    assert twist == pytest.approx((1.0, 0.0, 0.0))


def test_twist_in_body_frame() -> None:
    heading = math.pi / 2
    vx, vy, wz = body_twist(Pose2D(0.0, 0.0, heading), Pose2D(0.0, 0.1, heading), 0.1)
    assert vx > 0
    assert vx == pytest.approx(1.0)
    assert vy == pytest.approx(0.0, abs=1e-9)
    assert wz == pytest.approx(0.0)


def test_twist_lateral_in_body_frame() -> None:
    vx, vy, _ = body_twist(Pose2D(0.0, 0.0, 0.0), Pose2D(0.0, 0.1, 0.0), 0.1)
    assert vx == pytest.approx(0.0)
    assert vy == pytest.approx(1.0)


def test_twist_zero_dt() -> None:
    assert body_twist(Pose2D(0.0, 0.0, 0.0), Pose2D(0.1, 0.1, 0.1), 0.0) == (0.0, 0.0, 0.0)


def test_twist_negative_dt() -> None:
    assert body_twist(Pose2D(0.0, 0.0, 0.0), Pose2D(0.1, 0.1, 0.1), -0.05) == (0.0, 0.0, 0.0)


def test_twist_wraps_angle() -> None:
    _, _, wz = body_twist(Pose2D(0.0, 0.0, 3.1), Pose2D(0.0, 0.0, -3.1), 0.1)
    assert wz == pytest.approx((2 * math.pi - 6.2) / 0.1)
    assert wz == pytest.approx(0.83, abs=0.01)


def test_yaw_from_quaternion() -> None:
    half = math.pi / 4
    assert yaw_from_quaternion(0.0, 0.0, math.sin(half), math.cos(half)) == pytest.approx(
        math.pi / 2
    )


def test_yaw_from_identity_quaternion() -> None:
    assert yaw_from_quaternion(0.0, 0.0, 0.0, 1.0) == pytest.approx(0.0)

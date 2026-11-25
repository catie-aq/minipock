"""Planar odometry helpers: heading from a quaternion and body-frame velocity from two poses."""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Pose2D:
    """Planar pose in the odometry frame.

    Attributes:
        x: Position along x, in meters.
        y: Position along y, in meters.
        yaw: Heading in radians.
    """

    x: float
    y: float
    yaw: float


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Return the heading (rotation around z) of a quaternion.

    Args:
        x: Quaternion x component.
        y: Quaternion y component.
        z: Quaternion z component.
        w: Quaternion w component.

    Returns:
        The yaw in radians, in [-pi, pi].
    """
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _shortest_angle(angle: float) -> float:
    """Wrap an angle difference to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def body_twist(prev: Pose2D, curr: Pose2D, dt: float) -> tuple[float, float, float]:
    """Return the velocity that moved the robot from `prev` to `curr` in `dt` seconds.

    Args:
        prev: Previous pose.
        curr: Current pose.
        dt: Time between the two poses, in seconds.

    Returns:
        `(vx, vy, wz)`: linear velocity in the robot frame at `prev.yaw`, in m/s, and angular
        velocity along the shortest rotation, in rad/s. `(0.0, 0.0, 0.0)` if `dt <= 0`
        (equal or out-of-order stamps).
    """
    if dt <= 0:
        return (0.0, 0.0, 0.0)
    dx = curr.x - prev.x
    dy = curr.y - prev.y
    cos_yaw = math.cos(prev.yaw)
    sin_yaw = math.sin(prev.yaw)
    vx = (cos_yaw * dx + sin_yaw * dy) / dt
    vy = (-sin_yaw * dx + cos_yaw * dy) / dt
    wz = _shortest_angle(curr.yaw - prev.yaw) / dt
    return (vx, vy, wz)

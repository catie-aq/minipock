"""Planar odometry helpers: heading from a quaternion and body-frame velocity from two poses."""

import math
from dataclasses import dataclass

# Diagonal of the pose and twist covariances of the real robot odometry, in the order
# x, y, z, roll, pitch, yaw (m², rad²; (m/s)², (rad/s)² for the twist). Not measured: modest
# values (about 3 cm and 0.1 rad standard deviation, per sample for the twist) so that a
# consumer that weighs odometry (robot_localization, Cartographer) neither ignores it (zero
# covariance reads as perfect) nor discards it. z, roll and pitch are fixed for a planar base:
# tiny variance. To refine on the real robot (wheel slip in rotation).
POSE_VARIANCES = (1e-3, 1e-3, 1e-6, 1e-6, 1e-6, 1e-2)
TWIST_VARIANCES = (1e-3, 1e-3, 1e-6, 1e-6, 1e-6, 1e-2)
_DOF = 6


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


def diagonal_covariance(variances: tuple[float, ...]) -> list[float]:
    """Return the row-major 6x6 covariance of a ROS message, `variances` on its diagonal.

    Args:
        variances: Six variances, in the order x, y, z, roll, pitch, yaw.

    Returns:
        The 36 values of the matrix.

    Raises:
        ValueError: If there are not six non-negative variances.
    """
    if len(variances) != _DOF or any(v < 0 for v in variances):
        msg = f"Expected {_DOF} non-negative variances, got {variances}"
        raise ValueError(msg)
    return [variances[i] if i == j else 0.0 for i in range(_DOF) for j in range(_DOF)]

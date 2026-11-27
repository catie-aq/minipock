"""Tests of the messages built by raw_data_transformer and of its stamp warning."""

from collections.abc import Iterator

import pytest
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import PoseStamped

from minipock_bringup.odometry import POSE_VARIANCES, TWIST_VARIANCES
from minipock_bringup.raw_data_transformer import (
    RawDataTransformer,
    odometry_message,
    transform_message,
)

ODOM = "minipock_0/odom"
BASE = "minipock_0/base_footprint"


def _pose(sec: int, x: float = 0.0, nanosec: int = 0) -> PoseStamped:
    msg = PoseStamped()
    msg.header.stamp = TimeMsg(sec=sec, nanosec=nanosec)
    msg.header.frame_id = "firmware_frame"
    msg.pose.position.x = x
    msg.pose.orientation.w = 1.0
    return msg


def test_odometry_message() -> None:
    odom = odometry_message(_pose(3, x=1.5), (0.2, 0.1, 0.3), ODOM, BASE)
    assert odom.header.stamp == TimeMsg(sec=3)
    assert odom.header.frame_id == ODOM
    assert odom.child_frame_id == BASE
    assert odom.pose.pose.position.x == 1.5
    assert (odom.twist.twist.linear.x, odom.twist.twist.linear.y) == (0.2, 0.1)
    assert odom.twist.twist.angular.z == 0.3
    assert [odom.pose.covariance[7 * i] for i in range(6)] == list(POSE_VARIANCES)
    assert [odom.twist.covariance[7 * i] for i in range(6)] == list(TWIST_VARIANCES)


def test_odometry_header_not_shared_with_input() -> None:
    msg = _pose(3)
    odom = odometry_message(msg, (0.0, 0.0, 0.0), ODOM, BASE)
    assert odom.header is not msg.header
    assert msg.header.frame_id == "firmware_frame"


def test_transform_has_its_own_header() -> None:
    odom = odometry_message(_pose(3, x=1.5), (0.0, 0.0, 0.0), ODOM, BASE)
    transform = transform_message(odom)
    assert transform.header is not odom.header
    assert transform.header == odom.header
    assert transform.child_frame_id == BASE
    assert transform.transform.translation.x == 1.5
    transform.header.frame_id = "changed"
    assert odom.header.frame_id == ODOM


class _Logger:
    def __init__(self) -> None:
        self.warnings: list[str] = []

    def warning(self, message: str, **_: object) -> None:
        self.warnings.append(message)


@pytest.fixture
def node() -> Iterator[RawDataTransformer]:
    rclpy.init()
    transformer = RawDataTransformer()
    yield transformer
    transformer.destroy_node()
    rclpy.try_shutdown()


def _capture(node: RawDataTransformer, monkeypatch: pytest.MonkeyPatch) -> tuple[list, _Logger]:
    published: list = []
    logger = _Logger()
    monkeypatch.setattr(node._odom_publisher, "publish", published.append)
    monkeypatch.setattr(node._tf_broadcaster, "sendTransform", lambda _: None)
    monkeypatch.setattr(node, "get_logger", lambda: logger)
    return published, logger


@pytest.mark.parametrize("second_stamp", [(5, 0), (4, 900_000_000)])
def test_stamp_not_moving_forward_warns(
    node: RawDataTransformer, monkeypatch: pytest.MonkeyPatch, second_stamp: tuple[int, int]
) -> None:
    published, logger = _capture(node, monkeypatch)
    node._on_odom(_pose(5))
    node._on_odom(_pose(second_stamp[0], x=1.0, nanosec=second_stamp[1]))
    assert len(published) == 2
    assert published[1].twist.twist.linear.x == 0.0
    assert len(logger.warnings) == 1
    assert "synchronization" in logger.warnings[0]


def test_stamp_moving_forward_does_not_warn(
    node: RawDataTransformer, monkeypatch: pytest.MonkeyPatch
) -> None:
    published, logger = _capture(node, monkeypatch)
    node._on_odom(_pose(5))
    node._on_odom(_pose(6, x=1.0))
    assert published[1].twist.twist.linear.x == pytest.approx(1.0)
    assert logger.warnings == []

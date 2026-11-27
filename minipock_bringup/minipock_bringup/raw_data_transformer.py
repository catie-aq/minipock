"""Shape the raw odometry and scan of a real MiniPock into standard ROS messages and TF."""

import contextlib

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster

from minipock_bringup.odometry import (
    POSE_VARIANCES,
    TWIST_VARIANCES,
    Pose2D,
    body_twist,
    diagonal_covariance,
    yaw_from_quaternion,
)

_WARN_PERIOD = 5.0


_POSE_COVARIANCE = diagonal_covariance(POSE_VARIANCES)
_TWIST_COVARIANCE = diagonal_covariance(TWIST_VARIANCES)


def _pose_2d(msg: PoseStamped) -> Pose2D:
    orientation = msg.pose.orientation
    yaw = yaw_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)
    return Pose2D(msg.pose.position.x, msg.pose.position.y, yaw)


def odometry_message(
    msg: PoseStamped, twist: tuple[float, float, float], odom_frame: str, base_frame: str
) -> Odometry:
    """Return the `Odometry` of a firmware pose and the body-frame twist computed from it.

    Args:
        msg: Firmware pose, stamped by the firmware.
        twist: `(vx, vy, wz)` in the robot frame.
        odom_frame: Frame of the pose.
        base_frame: Frame moved by the odometry.

    Returns:
        The message, with the stamp of `msg` and the diagonal covariances of
        `minipock_bringup.odometry`.
    """
    odom = Odometry()
    odom.header = Header(stamp=msg.header.stamp, frame_id=odom_frame)
    odom.child_frame_id = base_frame
    odom.pose.pose = msg.pose
    odom.pose.covariance = _POSE_COVARIANCE
    odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.angular.z = twist
    odom.twist.covariance = _TWIST_COVARIANCE
    return odom


def transform_message(odom: Odometry) -> TransformStamped:
    """Return the `odom -> base` transform of an `Odometry`, with its own header."""
    transform = TransformStamped()
    transform.header = Header(stamp=odom.header.stamp, frame_id=odom.header.frame_id)
    transform.child_frame_id = odom.child_frame_id
    position = odom.pose.pose.position
    transform.transform.translation.x = position.x
    transform.transform.translation.y = position.y
    transform.transform.translation.z = position.z
    transform.transform.rotation = odom.pose.pose.orientation
    return transform


class RawDataTransformer(Node):
    """Republish the firmware topics of one robot, in the robot namespace.

    Inputs: `odom_raw` (`PoseStamped`, pose of the robot in its odometry frame) and `scan_raw`
    (`LaserScan`). Outputs: `odom` (`Odometry`, pose and body-frame twist), `scan` (stamp kept,
    frame set to `scan_frame_id`) and the `<ns>/odom -> <ns>/base_footprint` transform.
    """

    def __init__(self) -> None:
        """Declare the parameters and create the subscriptions, publishers and TF broadcaster."""
        super().__init__("raw_data_transformer")
        prefix = self.get_namespace().strip("/")
        prefix = f"{prefix}/" if prefix else ""
        self._odom_frame = self._declare_frame(
            "odom_frame_id", f"{prefix}odom", "Frame of the odometry pose"
        )
        self._base_frame = self._declare_frame(
            "base_frame_id", f"{prefix}base_footprint", "Robot frame moved by the odometry"
        )
        # The firmware still stamps its scans `lds_01_link`: the frame is replaced.
        self._scan_frame = self._declare_frame(
            "scan_frame_id", f"{prefix}lidar_link", "Frame written in the republished scans"
        )

        self._odom_publisher = self.create_publisher(Odometry, "odom", qos_profile_sensor_data)
        self._scan_publisher = self.create_publisher(LaserScan, "scan", qos_profile_sensor_data)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._previous: tuple[Pose2D, Time] | None = None

        self.create_subscription(PoseStamped, "odom_raw", self._on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "scan_raw", self._on_scan, qos_profile_sensor_data)

    def _declare_frame(self, name: str, default: str, description: str) -> str:
        descriptor = ParameterDescriptor(
            description=description, type=ParameterType.PARAMETER_STRING, read_only=True
        )
        return self.declare_parameter(name, default, descriptor).value

    def _on_odom(self, msg: PoseStamped) -> None:
        pose = _pose_2d(msg)
        stamp = Time.from_msg(msg.header.stamp)
        twist = (0.0, 0.0, 0.0)
        if self._previous is not None:
            previous_pose, previous_stamp = self._previous
            dt = (stamp - previous_stamp).nanoseconds * 1e-9
            if dt <= 0:
                # Equal or backward stamps: the firmware clock is not synchronized (micro-ROS
                # time sync lost or not done). The twist is zero until stamps move forward.
                self.get_logger().warning(
                    f"Odometry stamp did not move forward (dt = {dt:.3f} s): twist set to "
                    "zero; check the firmware time synchronization",
                    throttle_duration_sec=_WARN_PERIOD,
                )
            twist = body_twist(previous_pose, pose, dt)
        self._previous = (pose, stamp)

        odom = odometry_message(msg, twist, self._odom_frame, self._base_frame)
        self._tf_broadcaster.sendTransform(transform_message(odom))
        self._odom_publisher.publish(odom)

    def _on_scan(self, msg: LaserScan) -> None:
        if msg.angle_max <= msg.angle_min:
            self.get_logger().warning(
                "Dropping a scan with angle_max <= angle_min", throttle_duration_sec=_WARN_PERIOD
            )
            return
        msg.header.frame_id = self._scan_frame
        self._scan_publisher.publish(msg)


def main(args: list[str] | None = None) -> None:
    """Run the node until shutdown.

    Args:
        args: Command line arguments for rclpy.
    """
    rclpy.init(args=args)
    node = RawDataTransformer()
    with contextlib.suppress(KeyboardInterrupt, ExternalShutdownException):
        rclpy.spin(node)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()

import unittest
from unittest.mock import Mock

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

from minipock_bringup import odometry_transform_publisher


class TestOdometryTransformPublisher(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = odometry_transform_publisher.OdometryTransformPublisher()

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_initialization(self):
        self.assertIsInstance(self.node, Node)
        self.assertIsNone(self.node._OdometryTransformPublisher__last_odom)

    def test_callback(self):
        msg = Odometry()
        self.node.callback(msg)
        self.assertEqual(self.node._OdometryTransformPublisher__last_odom, msg)

    def test_callback_tf(self):
        # Mocking the TransformBroadcaster to not actually send transform
        self.node._OdometryTransformPublisher__tf_broadcaster = Mock()
        # Testing with no odom message received
        tf_msg = TFMessage()
        self.node.callback_tf(tf_msg)
        self.node._OdometryTransformPublisher__tf_broadcaster.sendTransform.assert_not_called()

        # Testing with a received odom message
        self.node._OdometryTransformPublisher__last_odom = Odometry()
        self.node.callback_tf(tf_msg)
        self.node._OdometryTransformPublisher__tf_broadcaster.sendTransform.assert_called_once()


if __name__ == "__main__":
    unittest.main()

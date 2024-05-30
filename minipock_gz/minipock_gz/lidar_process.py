"""
The Python module introduces a ROS2 ScanFilterNode class.
The constructor initializes a node ('scan_filter_node'), sets up a subscription to the
/minipock/scan_raw topic,
 and creates a publisher for /minipock/scan, all with a queue size of 10.

The listener_callback method modifies the frame_id of incoming LaserScan messages
and uses the publisher to send them to /minipock/scan.

The main function initializes rclpy, creates a ScanFilterNode instance, spins the node,
then terminates the node and rclpy.
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan


class ScanFilterNode(Node):
    def __init__(self):
        super().__init__("scan_filter_node")
        self.subscription = self.create_subscription(
            LaserScan, "/scan_raw", self.listener_callback, 50
        )
        self.publisher = self.create_publisher(LaserScan, "/scan", 50)

    def listener_callback(self, msg):
        """
        This listener_callback method is used to process the received LaserScan message.
        It updates the frame_id of the message to 'lds_01_link' and publishes it using a publisher.

        :param msg: The LaserScan message received from the listener.
        :return: None
        """
        msg.header.frame_id = "lds_01_link"
        self.publisher.publish(msg)


def main(args=None):
    """
    :param args: (Optional) Command line arguments passed to the script.
    :return: None
    """
    rclpy.init(args=args)

    scan_filter_node = ScanFilterNode()

    rclpy.spin(scan_filter_node)

    scan_filter_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

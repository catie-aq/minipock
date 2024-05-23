import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


class OdometryTransformPublisher(Node):
    def __init__(self):
        super().__init__("odometry_transform_publisher")
        self.create_subscription(Odometry, "/odom", self.callback, qos_profile_sensor_data)
        self.__tf_broadcaster = TransformBroadcaster(self)

        self.__odom_transform = TransformStamped()
        self.__odom_transform.header.frame_id = "odom"
        self.__odom_transform.child_frame_id = "base_link"

    def callback(self, msg):
        """
        Callback to receive a message.

        :param msg: The message received.
        :return: None.
        """
        self.__odom_transform.header.stamp = self.get_clock().now().to_msg()
        self.__odom_transform.transform.translation.x = msg.pose.pose.position.x
        self.__odom_transform.transform.translation.y = msg.pose.pose.position.y
        self.__odom_transform.transform.translation.z = msg.pose.pose.position.z
        self.__odom_transform.transform.rotation = msg.pose.pose.orientation
        self.__tf_broadcaster.sendTransform(self.__odom_transform)


def main(args=None):
    """
    :param args: The command line arguments passed to the program.
    :return: None.

    The main method is the entry point for the program.
    """
    rclpy.init(args=args)

    odom_subscriber = OdometryTransformPublisher()

    rclpy.spin(odom_subscriber)

    # Destruction du nœud explicitement
    odom_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

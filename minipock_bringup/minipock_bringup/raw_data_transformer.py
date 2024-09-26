import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan


class RawDataTransformer(Node):
    def __init__(self):
        super().__init__("raw_data_transformer")

        self.declare_parameter("robot_name", "minipock")
        self.__robot_name = self.get_parameter("robot_name").value

        self.create_subscription(
            PoseStamped,
            f"odom_raw",
            self.callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            f"scan_raw",
            self.callback_scan,
            qos_profile_sensor_data,
        )
        self.___odom_publisher = self.create_publisher(Odometry, f"odom", qos_profile_sensor_data)
        self.__scan_publisher = self.create_publisher(LaserScan, f"scan", qos_profile_sensor_data)
        self.__tf_broadcaster = TransformBroadcaster(self)

        self.__odom_transform = TransformStamped()
        self.__odom_transform.header.frame_id = f"{self.__robot_name}/odom"
        self.__odom_transform.child_frame_id = f"{self.__robot_name}/base_footprint"

    def callback(self, msg):
        """
        Callback to receive a message.

        :param msg: The message received.
        :return: None.
        """

        self.__odom_transform.header.stamp = msg.header.stamp
        self.__odom_transform.transform.translation.x = msg.pose.position.x
        self.__odom_transform.transform.translation.y = msg.pose.position.y
        self.__odom_transform.transform.translation.z = msg.pose.position.z
        self.__odom_transform.transform.rotation = msg.pose.orientation

        odom = Odometry()
        odom.header = self.__odom_transform.header
        odom.child_frame_id = self.__odom_transform.child_frame_id
        odom.pose.pose = msg.pose

        self.__tf_broadcaster.sendTransform(self.__odom_transform)
        self.___odom_publisher.publish(odom)

    def callback_scan(self, msg):
        """
        Callback to receive a message.

        :param msg: The message received.
        :return: None.
        """
        msg.header.stamp = self.get_clock().now().to_msg()
        if msg.angle_max > msg.angle_min:
            self.__scan_publisher.publish(msg)


def main(args=None):
    """
    :param args: The command line arguments passed to the program.
    :return: None.

    The main method is the entry point for the program.
    """
    rclpy.init(args=args)
    odom_subscriber = RawDataTransformer()
    odom_subscriber.get_logger().info("Raw data transformer has been started.")

    rclpy.spin(odom_subscriber)

    # Destruction du nœud explicitement
    odom_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

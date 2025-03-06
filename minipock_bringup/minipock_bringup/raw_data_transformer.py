import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from filterpy.kalman import KalmanFilter
import numpy as np

class ExtendedKalmanFilter:
    def __init__(self):
        # Initialize the Kalman Filter
        self.kf = KalmanFilter(dim_x=3, dim_z=3)
        self.kf.x = np.array([0., 0., 0.])  # initial state
        self.kf.P *= 1.  # initial uncertainty
        self.kf.R = np.eye(3)  # measurement noise
        self.kf.Q = np.eye(3)  # process noise
        self.kf.F = np.eye(3)  # state transition matrix
        self.kf.H = np.eye(3)  # measurement function

    def update(self, measurement):
        # Update the state with the new measurement
        self.kf.update(np.array(measurement))

    def get_state(self):
        return self.kf.x

class RawDataTransformer(Node):
    def __init__(self):
        super().__init__("raw_data_transformer")

        self.declare_parameter("robot_name", "minipock_0")
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
        self.create_subscription(
            PoseStamped,
            f"odom_optc_raw",
            self.callback_optc,
            qos_profile_sensor_data,
        )

        self.___odom_publisher = self.create_publisher(Odometry, f"odom", qos_profile_sensor_data)
        self.___odom_optc_publisher = self.create_publisher(Odometry, f"odom_optc", qos_profile_sensor_data)
        self.___odom_fusion_publisher = self.create_publisher(Odometry, f"odom_fusion", qos_profile_sensor_data)
        self.__scan_publisher = self.create_publisher(LaserScan, f"scan", qos_profile_sensor_data)
        self.__tf_broadcaster = TransformBroadcaster(self)

        self.__odom_transform = TransformStamped()
        self.__odom_transform.header.frame_id = f"{self.__robot_name}/odom"
        self.__odom_transform.child_frame_id = f"{self.__robot_name}/base_footprint"

        self.__last_odom_msg = None
        self.__last_odom_optc_msg = None

    def callback(self, msg):
        """
        Callback to receive a message.

        :param msg: The message received.
        :return: None.
        """
        self.__last_odom_msg = msg
        self.__publish_odom(msg, self.___odom_publisher)

    def callback_optc(self, msg):
        """
        Callback to receive a message.

        :param msg: The message received.
        :return: None.
        """
        self.__last_odom_optc_msg = msg
        self.__publish_odom(msg, self.___odom_optc_publisher)
        self.callback_fusion()

    def callback_fusion(self):
        """
        Callback to publish the mean pose of the two messages.
        :return: None.
        """
        if self.__last_odom_msg is None or self.__last_odom_optc_msg is None:
            return

        # Assuming you have an EKF implementation available
        ekf = ExtendedKalmanFilter()

        # Prepare the measurements
        z1 = [self.__last_odom_msg.pose.position.x, self.__last_odom_msg.pose.position.y, self.__last_odom_msg.pose.position.z]
        z2 = [self.__last_odom_optc_msg.pose.position.x, self.__last_odom_optc_msg.pose.position.y, self.__last_odom_optc_msg.pose.position.z]

        # Update the EKF with the measurements
        ekf.update(z1)
        ekf.update(z2)

        # Get the estimated state
        estimated_state = ekf.get_state()

        mean_pose = PoseStamped()
        mean_pose.header.stamp = self.get_clock().now().to_msg()
        mean_pose.pose.position.x = estimated_state[0]
        mean_pose.pose.position.y = estimated_state[1]
        mean_pose.pose.position.z = estimated_state[2]
        mean_pose.pose.orientation = self.__last_odom_msg.pose.orientation  # Assuming orientation is the same

        self.__publish_odom(mean_pose, self.___odom_fusion_publisher)

    def __publish_odom(self, msg, publisher):
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
        publisher.publish(odom)

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
    odom_subscriber.get_logger().info("Raw data transformer has been started. /")

    rclpy.spin(odom_subscriber)

    # Destruction du nœud explicitement
    odom_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

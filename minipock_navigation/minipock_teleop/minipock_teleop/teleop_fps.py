import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard


class TeleopFps(Node):
    def __init__(self):
        super().__init__("teleop_keyboard")
        self.declare_parameter("namespace", "")
        self.namespace = self.get_parameter("namespace").value
        cmd_vel_topics = self.select_topic()
        cmd_vel_topic = ""
        if len(cmd_vel_topics) == 0:
            self.get_logger().error("No cmd_vel topic found")
            return
        elif "/" + self.namespace + "/cmd_vel" in cmd_vel_topics:
            cmd_vel_topic = "/" + self.namespace + "/cmd_vel"
        else:
            cmd_vel_topic = cmd_vel_topics[0]

        self.boost = False

        self.max_vel_lin = 10.0
        self.max_vel_ang = 20.0

        self.current_vel_lin = 0.0
        self.current_vel_ang = 0.0

        self.candidate_vel_lin = 0.0
        self.candidate_vel_ang = 0.0
        self.publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.timer = self.create_timer(0.01, self.send_cmd_vel)

    def select_topic(self):
        topics = self.get_topic_names_and_types()
        cmd_vel_topics = []
        for topic in topics:
            if "cmd_vel" in topic[0]:
                cmd_vel_topics.append(topic[0])
        return cmd_vel_topics

    def send_cmd_vel(self):
        if (
            self.current_vel_lin != self.candidate_vel_lin
            or self.current_vel_ang != self.candidate_vel_ang
        ):
            self.current_vel_lin = self.candidate_vel_lin
            self.current_vel_ang = self.candidate_vel_ang
            twist = Twist()
            if not self.boost:
                twist.linear.x = self.current_vel_lin / 10
                twist.angular.z = self.current_vel_ang / 10
            else:
                twist.linear.x = self.current_vel_lin
                twist.angular.z = self.current_vel_ang
            self.publisher.publish(twist)

    def on_press(self, key):
        try:
            if key.char == "z":
                self.candidate_vel_lin = self.max_vel_lin
            elif key.char == "s":
                self.candidate_vel_lin = -self.max_vel_lin
            elif key.char == "q":
                self.candidate_vel_ang = self.max_vel_ang
            elif key.char == "d":
                self.candidate_vel_ang = -self.max_vel_ang
            elif key.char == "b":
                self.boost = not self.boost
        except AttributeError:
            pass

    def on_release(self, key):
        try:
            if key.char == "z" or key.char == "s":
                self.candidate_vel_lin = 0.0
            elif key.char == "q" or key.char == "d":
                self.candidate_vel_ang = 0.0
        except AttributeError:
            pass


def main():
    """
    Entry point of the program.
    Initializes ROS, creates a TeleopController node, and executes the main loop.
    """
    rclpy.init()
    teleop = TeleopFps()
    with keyboard.Listener(on_press=teleop.on_press, on_release=teleop.on_release) as listener:
        rclpy.spin(teleop)
        listener.join()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

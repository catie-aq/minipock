"""Keyboard teleoperation for MiniPock (z/s linear, q/d angular, x stop)."""

import os
import select
import sys

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile

if os.name == "nt":
    import msvcrt
else:
    import termios
    import tty


def constrain(input_vel: float, low_bound: float, high_bound: float) -> float:
    """Constrain a velocity value within lower and upper bounds.

    Args:
        input_vel: The velocity value to constrain.
        low_bound: The lower bound for the velocity value.
        high_bound: The upper bound for the velocity value.

    Returns:
        The velocity value constrained within the bounds.
    """
    return max(min(input_vel, high_bound), low_bound)


def get_key(settings: list | None) -> str:
    """Read one key from the terminal without echo.

    Args:
        settings: The termios settings to restore after reading the key.

    Returns:
        The pressed key, or an empty string if no key was pressed.
    """
    if os.name == "nt":
        return msvcrt.getch().decode("utf-8")
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else ""

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def make_twist(linear: float, angular: float) -> Twist:
    """Create a Twist message with the given linear and angular values.

    Args:
        linear: The linear velocity along x.
        angular: The angular velocity around z.

    Returns:
        A Twist message with the specified velocities.
    """
    twist = Twist()
    twist.linear.x = linear
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = angular
    return twist


class TeleopController(Node):
    """Node publishing velocity commands from keyboard input."""

    def __init__(self) -> None:
        """Resolve the namespace, check the cmd_vel topic and create the publisher."""
        super().__init__("teleop_keyboard")
        self.declare_parameter("namespace", "")
        namespace = self.get_parameter("namespace").value
        self.get_logger().info(f"Namespace: {namespace}")

        namespace = self.check_ns_exists(namespace)
        if namespace[-1] != "/" and namespace != "":
            namespace += "/"
            self.get_logger().info("Non empty namespace set with no '/' at the end. Adding it.")
            self.get_logger().info(f"New namespace: {namespace}")

        self.check_topic_exists(f"/{namespace}cmd_vel")

        self.MINIPOCK_MAX_LIN_VEL = 2.0
        self.MINIPOCK_MAX_ANG_VEL = 12.0
        self.LIN_VEL_STEP_SIZE = 0.1
        self.ANG_VEL_STEP_SIZE = 0.2

        self.qos = QoSProfile(depth=10)
        self.publisher = self.create_publisher(Twist, f"{namespace}cmd_vel", self.qos)
        self.rate = self.create_rate(10)
        self.status = 0
        self.settings = None

        self.target_linear_velocity = 0.0
        self.target_angular_velocity = 0.0
        self.control_linear_velocity = 0.0
        self.control_angular_velocity = 0.0

        self.previous_x_linear = 0.0
        self.previous_z_angular = 0.0

        self.help_msg = """
        Control Your MiniPock!
        ---------------------------
        Moving around:
                z
           q    x    d
                s

        z/s : increase/decrease linear velocity
        q/d : increase/decrease angular velocity

        x : force stop

        CTRL-C to quit
        """

    def clean_namespace(self, namespace: str) -> str:
        """Clean the namespace by removing the leading and trailing slashes.

        Args:
            namespace: The namespace to clean.

        Returns:
            The cleaned namespace.
        """
        try:
            if namespace[0] == "/":
                namespace = namespace[1:]
            if len(namespace) > 1 and namespace[-1] == "/":
                namespace = namespace[:-1]
        except IndexError:
            return ""
        return namespace

    def retrieve_all_namespaces(self) -> list[str]:
        """Retrieve all namespaces in the ROS 2 environment.

        Returns:
            The non-empty namespaces of the running nodes.
        """
        node_names_and_namespaces = self.get_node_names_and_namespaces()
        namespaces = []
        for _, ns in node_names_and_namespaces:
            raw_ns = self.clean_namespace(ns)
            if raw_ns not in namespaces and raw_ns != "":
                namespaces.append(raw_ns)
        return namespaces

    def check_ns_exists(self, namespace: str) -> str:
        """Check if a given namespace exists in the ROS 2 environment.

        Falls back to the first existing namespace if it does not exist.

        Args:
            namespace: The namespace to check.

        Returns:
            The cleaned namespace, or the first existing one.
        """
        global_namespaces = self.retrieve_all_namespaces()
        namespace = self.clean_namespace(namespace)
        if namespace not in global_namespaces:
            self.get_logger().info(
                f"Namespace '{namespace}' not in global namespaces. "
                f"Existing namespaces are: {global_namespaces}. "
                "Selecting the first namespace in the list"
            )
            return global_namespaces[0]
        return namespace

    def check_topic_exists(self, topic: str) -> None:
        """Check if a given topic exists in the ROS 2 environment.

        Exits the program if the topic does not exist, and prints the existing topics.

        Args:
            topic: The topic to check.
        """
        found = False
        topic_list = []
        topic_exists = self.get_topic_names_and_types()
        for topic_name, _ in topic_exists:
            if topic_name == topic:
                self.get_logger().info(f"Topic '{topic}' found")
                found = True
            if "cmd_vel" in topic_name:
                topic_list.append(topic_name)
        if not found:
            self.get_logger().error(
                f"Topic '{topic}' not found. \n \t Existing topics are: {topic_list}"
            )
            sys.exit(1)

    def print_velocities(self) -> None:
        """Print the current linear and angular velocities."""
        self.get_logger().info(
            f"currently:\tlinear velocity {self.target_linear_velocity}\t "
            f"angular velocity {self.target_angular_velocity} "
        )

    def check_linear_limit_velocity(self, velocity: float) -> float:
        """Check if the given linear velocity is within the acceptable range.

        Args:
            velocity: The linear velocity to check.

        Returns:
            The constrained linear velocity.
        """
        return constrain(velocity, -self.MINIPOCK_MAX_LIN_VEL, self.MINIPOCK_MAX_LIN_VEL)

    def check_angular_limit_velocity(self, velocity: float) -> float:
        """Check the angular limit velocity.

        Args:
            velocity: The input velocity value.

        Returns:
            The constrained velocity value.
        """
        return constrain(velocity, -self.MINIPOCK_MAX_ANG_VEL, self.MINIPOCK_MAX_ANG_VEL)

    def set_terminal_settings(self) -> None:
        """Save the terminal settings needed to restore the terminal after raw input mode."""
        self.settings = None
        if os.name != "nt":
            self.settings = termios.tcgetattr(sys.stdin)

    def reset_terminal_settings(self) -> None:
        """Reset the terminal settings to their original state."""
        if os.name != "nt":
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def make_simple_profile_linear(self) -> None:
        """Ramp the control linear velocity towards the target."""
        if self.target_linear_velocity > self.control_linear_velocity:
            self.control_linear_velocity = min(
                self.target_linear_velocity,
                self.control_linear_velocity + self.LIN_VEL_STEP_SIZE / 2.0,
            )
        elif self.target_linear_velocity < self.control_linear_velocity:
            self.control_linear_velocity = max(
                self.target_linear_velocity,
                self.control_linear_velocity - self.LIN_VEL_STEP_SIZE / 2.0,
            )

    def make_simple_profile_angular(self) -> None:
        """Ramp the control angular velocity towards the target."""
        if self.target_angular_velocity > self.control_angular_velocity:
            self.control_angular_velocity = min(
                self.target_angular_velocity,
                self.control_angular_velocity + self.ANG_VEL_STEP_SIZE / 2.0,
            )
        elif self.target_angular_velocity < self.control_angular_velocity:
            self.control_angular_velocity = max(
                self.target_angular_velocity,
                self.control_angular_velocity - self.ANG_VEL_STEP_SIZE / 2.0,
            )

    def process_key(self, key: str) -> None:
        """Process a key press and update the target linear and angular velocity.

        Args:
            key: The pressed key.
        """
        if key in ["z", "q"]:
            step_size = self.LIN_VEL_STEP_SIZE if key == "z" else self.ANG_VEL_STEP_SIZE
            if key == "q":
                self.target_angular_velocity = self.check_angular_limit_velocity(
                    self.target_angular_velocity + step_size
                )
            else:
                self.target_linear_velocity = self.check_linear_limit_velocity(
                    self.target_linear_velocity + step_size
                )

        if key in ["s", "d"]:
            step_size = self.LIN_VEL_STEP_SIZE if key == "s" else self.ANG_VEL_STEP_SIZE
            if key == "d":
                self.target_angular_velocity = self.check_angular_limit_velocity(
                    self.target_angular_velocity - step_size
                )
            else:
                self.target_linear_velocity = self.check_linear_limit_velocity(
                    self.target_linear_velocity - step_size
                )
        if key == "x":
            self.target_linear_velocity = 0.0
            self.target_angular_velocity = 0.0

    def main(self) -> None:
        """Control the robot with the keyboard until CTRL-C is pressed."""
        self.set_terminal_settings()
        try:
            self.get_logger().info(self.help_msg)
            status = 0
            while True:
                key = get_key(self.settings)
                if key in ["z", "q", "s", "d", "x"]:
                    self.process_key(key)
                    self.print_velocities()
                    status = (status + 1) % 20
                    if status == 0:
                        self.get_logger().info(self.help_msg)
                elif key == "\x03":
                    self.publisher.publish(make_twist(0.0, 0.0))
                    break
                self.make_simple_profile_linear()
                self.make_simple_profile_angular()
                if (
                    self.control_linear_velocity != self.previous_x_linear
                    or self.control_angular_velocity != self.previous_z_angular
                ):
                    self.publisher.publish(
                        make_twist(self.control_linear_velocity, self.control_angular_velocity)
                    )
                    self.previous_x_linear = self.control_linear_velocity
                    self.previous_z_angular = self.control_angular_velocity
        except Exception as e:  # noqa: BLE001 - the terminal must always be restored
            self.get_logger().error("Error: " + str(e))
        finally:
            self.publisher.publish(make_twist(0.0, 0.0))
            self.reset_terminal_settings()


def main() -> None:
    """Initialize ROS, create the TeleopController node and run its main loop."""
    rclpy.init()
    node = TeleopController()
    node.main()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

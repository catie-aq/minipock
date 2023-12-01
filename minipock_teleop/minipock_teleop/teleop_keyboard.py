#!/usr/bin/env python3

import os
import select
import sys

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty


def constrain(input_vel, low_bound, high_bound):
    """
    Constrains a given input velocity value within specified lower and upper bounds.

    :param input_vel: The input velocity value has to be constrained.
    :param low_bound: The lower bound for the velocity value.
    :param high_bound: The upper bound for the velocity value.
    :return: The constrained velocity value within the specified bounds.
    """
    return max(min(input_vel, high_bound), low_bound)


def get_key(settings):
    """
    :param settings: the termios settings to restore after reading the key
    :return: the pressed key as a string or an empty string if no key was pressed
    """
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def make_twist(linear, angular):
    """
    Creates a Twist message object with the given linear and angular values.

    :param linear: The linear value of the Twist (float).
    :param angular: The angular value of the Twist (float).
    :return: A Twist message object with the specified linear and angular values.
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
    def __init__(self):
        super().__init__('teleop_keyboard')

        self.MINIPOCK_MAX_LIN_VEL = 2.0
        self.MINIPOCK_MAX_ANG_VEL = 12.0
        self.LIN_VEL_STEP_SIZE = 0.1
        self.ANG_VEL_STEP_SIZE = 0.2

        self.qos = QoSProfile(depth=10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', self.qos)
        self.status = 0
        self.settings = None

        self.target_linear_velocity = 0.0
        self.target_angular_velocity = 0.0
        self.control_linear_velocity = 0.0
        self.control_angular_velocity = 0.0

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

    def print_velocities(self):
        """
        Prints the current linear and angular velocities.

        :return: None
        """
        self.get_logger().info('currently:\tlinear velocity {0}\t angular velocity {1} '.format(
            self.target_linear_velocity,
            self.target_angular_velocity))

    def check_linear_limit_velocity(self, velocity):
        """
        Check if the given linear velocity is within the acceptable range.

        :param velocity: The linear velocity to check.
        :return: The constrained linear velocity.
        """
        return constrain(velocity, -self.MINIPOCK_MAX_LIN_VEL, self.MINIPOCK_MAX_LIN_VEL)

    def check_angular_limit_velocity(self, velocity):
        """
        Check the angular limit velocity.

        :param velocity: The input velocity value.
        :return: The constrained velocity value.

        """
        return constrain(velocity, -self.MINIPOCK_MAX_ANG_VEL, self.MINIPOCK_MAX_ANG_VEL)

    def set_terminal_settings(self):
        """
        Sets the terminal settings needed for raw input mode.

        :returns: None
        """
        self.settings = None
        if os.name != 'nt':
            self.settings = termios.tcgetattr(sys.stdin)

    def reset_terminal_settings(self):
        """
        Resets the terminal settings to their original state.

        :returns: None
        """
        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def make_simple_profile_linear(self):
        """
        :return: The updated output value.
        """
        if self.target_linear_velocity > self.control_linear_velocity:
            self.control_linear_velocity = min(self.target_linear_velocity,
                                               self.control_linear_velocity +
                                               self.LIN_VEL_STEP_SIZE / 2.0)
        elif self.target_linear_velocity < self.control_linear_velocity:
            self.control_linear_velocity = max(self.target_linear_velocity,
                                               self.control_linear_velocity -
                                               self.LIN_VEL_STEP_SIZE / 2.0)

    def make_simple_profile_angular(self):
        """
        Calculate the angular velocity profile based on the target and current angular velocities.

        :return: The calculated angular velocity profile.
        """
        if self.target_angular_velocity > self.control_angular_velocity:
            self.control_angular_velocity = min(self.target_angular_velocity,
                                                self.control_angular_velocity +
                                                self.ANG_VEL_STEP_SIZE / 2.0)
        elif self.target_angular_velocity < self.control_angular_velocity:
            self.control_angular_velocity = max(self.target_angular_velocity,
                                                self.control_angular_velocity -
                                                self.ANG_VEL_STEP_SIZE / 2.0)

    def process_key(self, key):
        """
        Process a key press event and update the target linear and angular velocity.

        :param key: str representing the key pressed.

        :return: Tuple of two floats representing the updated target linear and angular velocity.
        """
        if key in ['z', 'q']:
            step_size = self.LIN_VEL_STEP_SIZE if key == 'z' else self.ANG_VEL_STEP_SIZE
            if key == 'q':
                self.target_angular_velocity = self.check_angular_limit_velocity(
                    self.target_angular_velocity + step_size)
            else:
                self.target_linear_velocity = self.check_linear_limit_velocity(
                    self.target_linear_velocity + step_size)

        if key in ['s', 'd']:
            step_size = self.LIN_VEL_STEP_SIZE if key == 's' else self.ANG_VEL_STEP_SIZE
            if key == 'd':
                self.target_angular_velocity = self.check_angular_limit_velocity(
                    self.target_angular_velocity - step_size)
            else:
                self.target_linear_velocity = self.check_linear_limit_velocity(
                    self.target_linear_velocity - step_size)
        if key in ['x']:
            self.target_linear_velocity = 0.0
            self.target_angular_velocity = 0.0

    def main(self):
        """
        Main method for controlling a robot with teleoperation.

        :return: None
        """
        self.set_terminal_settings()
        try:
            self.get_logger().info(self.help_msg)
            status = 0
            while True:
                key = get_key(self.settings)
                if key in ['z', 'q', 's', 'd', 'x']:
                    self.process_key(key)
                    self.print_velocities()
                    status = (status + 1) % 20
                    if status == 0:
                        self.get_logger().info(self.help_msg)
                elif key == '\x03':
                    self.publisher.publish(make_twist(0.0, 0.0))
                    break
                self.make_simple_profile_linear()
                self.make_simple_profile_angular()
                self.publisher.publish(make_twist(self.control_linear_velocity,
                                                  self.control_angular_velocity))
        except Exception as e:
            self.get_logger().error('Error: ' + str(e))
        finally:
            self.publisher.publish(make_twist(0.0, 0.0))
            self.reset_terminal_settings()


def main():
    """
    Entry point of the program.
    Initializes ROS, creates a TeleopController node, and executes the main loop.
    """
    rclpy.init()
    node = TeleopController()
    node.main()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

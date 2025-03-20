# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


"""
The RobotAPI class is a wrapper for API calls to the robot. Here users
are expected to fill up the implementations of functions which will be used
by the RobotCommandHandle. For example, if your robot has a REST API, you
will need to make http request calls to the appropriate endpoints within
these functions.
"""
import math
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped


class AmclPoseListener(Node):
    def __init__(self, robot_name):
        super().__init__("amcl_pose_listener_" + robot_name)
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            f"/{robot_name}/amcl_pose",
            self.listener_callback,
            10,
        )
        self.current_pose = None

    def listener_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        tetha = self.quaternion_to_yaw(orientation)
        self.get_logger().info(f"Position -> x: {position.x}, y: {position.y}, z: {position.z}")
        self.get_logger().info(
            f"Orientation -> x: {orientation.x}, y: {orientation.y}, z: {orientation.z}, w: {orientation.w}"
        )
        self.current_pose = {
            "position": {"x": position.x, "y": position.y, "z": position.z},
            "orientation": {"tetha": tetha},
        }

    def quaternion_to_yaw(self, orientation):
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def get_pose(self):
        """Return the latest position."""
        return self.current_pose


class GoalPosePulisher(Node):
    def __init__(self, robot_name):
        super().__init__("goal_pose_publisher" + robot_name)
        self.publisher = self.create_publisher(PoseStamped, f"/{robot_name}/goal_pose", 10)

    def publish_goal_pose(self, goal_pose, map_name):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = goal_pose[0]
        msg.pose.position.y = goal_pose[1]
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(goal_pose[2] / 2)
        msg.pose.orientation.w = math.cos(goal_pose[2] / 2)
        self.publisher.publish(msg)
        print("Published goal pose: ", goal_pose)

    def check_goal_pose_sent(self):
        # check if the goal pose has been sent
        return True


class RobotAPI:
    # The constructor below accepts parameters typically required to submit
    # http requests. Users should modify the constructor as per the
    # requirements of their robot's API
    def __init__(self, config_yaml):
        self.fleet_mgr = config_yaml["fleet_manager"]
        self.prefix = "http://" + self.fleet_mgr["ip"] + ":" + str(self.fleet_mgr["port"])
        self.user = self.fleet_mgr["user"]
        self.password = self.fleet_mgr["password"]
        self.timeout = 5.0
        self.debug = False

        self.robots = config_yaml["rmf_fleet"]["robots"]
        self.goal_pose_publishers = {}
        self.amcl_pose_listeners = {}
        for robot_name in self.robots:
            self.amcl_pose_listeners[robot_name] = AmclPoseListener(robot_name)
            self.goal_pose_publishers[robot_name] = GoalPosePulisher(robot_name)

    def get_amcl_pose_listener(self, robot_name):
        return self.amcl_pose_listeners[robot_name]

    def get_goal_pose_publisher(self, robot_name):
        return self.goal_pose_publishers[robot_name]

    def check_connection(self):
        """Return True if connection to the robot API server is successful"""
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        return True

    def navigate(self, robot_name: str, pose, map_name: str, speed_limit=0.0):
        """Request the robot to navigate to pose:[x,y,theta] where x, y and
        and theta are in the robot's coordinate convention. This function
        should return True if the robot has accepted the request,
        else False"""
        goal_pose_publisher = self.goal_pose_publishers[robot_name]
        goal_pose_publisher.publish_goal_pose(pose, map_name)
        return goal_pose_publisher.check_goal_pose_sent()

    def start_activity(self, robot_name: str, activity: str, label: str):
        """Request the robot to begin a process. This is specific to the robot
        and the use case. For example, load/unload a cart for Deliverybot
        or begin cleaning a zone for a cleaning robot.
        Return True if process has started/is queued successfully, else
        return False"""
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        return False

    def stop(self, robot_name: str):
        """Command the robot to stop.
        Return True if robot has successfully stopped. Else False."""
        goal_pose_publisher = self.goal_pose_publishers[robot_name]
        goal_pose_publisher.publish_goal_pose(
            self.amcl_pose_listeners[robot_name].current_pose, "ground"
        )
        print("Stop command sent to ", robot_name)
        return True

    def position(self, robot_name: str):
        """Return [x, y, theta] expressed in the robot's coordinate frame or
        None if any errors are encountered"""
        amcl_pose_listener = self.amcl_pose_listeners[robot_name]
        amcl_pose = amcl_pose_listener.get_pose()
        pose_xyt = None
        if amcl_pose is None:
            # Hardcoded initial pose for minipock robots because the amcl_pose is not available
            # or not accessed at launch --> need to change this
            if robot_name == "minipock_0":
                pose_xyt = [2.053461700390844, -6.2162598987180635, 0.0]
            elif robot_name == "minipock_1":
                pose_xyt = [7.929064222877705, -3.815474419287308, 0.0]
            else:
                pose_xyt = [0.0, 0.0, 0.0]
        else:
            pose_xyt = [
                amcl_pose["position"]["x"],
                amcl_pose["position"]["y"],
                amcl_pose["orientation"]["tetha"],
            ]
        return pose_xyt

    def battery_soc(self, robot_name: str):
        """Return the state of charge of the robot as a value between 0.0
        and 1.0. Else return None if any errors are encountered."""
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        return 1.0

    def map(self, robot_name: str):
        """Return the name of the map that the robot is currently on or
        None if any errors are encountered."""
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        return "ground"

    def is_command_completed(self):
        """Return True if the robot has completed its last command, else
        return False."""
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        return True

    def get_data(self, robot_name: str):
        """Returns a RobotUpdateData for one robot if a name is given. Otherwise
        return a list of RobotUpdateData for all robots."""
        robot_map = self.map(robot_name)
        position = self.position(robot_name)
        battery_soc = self.battery_soc(robot_name)
        if not (robot_map is None or position is None or battery_soc is None):
            return RobotUpdateData(robot_name, robot_map, position, battery_soc)
        return None


class RobotUpdateData:
    """Update data for a single robot."""

    def __init__(
        self,
        robot_name: str,
        map: str,
        position: list[float],
        battery_soc: float,
        requires_replan: bool | None = None,
    ):
        self.robot_name = robot_name
        self.position = position
        self.map = map
        self.battery_soc = battery_soc
        self.requires_replan = requires_replan

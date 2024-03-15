"""
Class for a bridge between Gazebo and ROS.
"""

from dataclasses import dataclass
from enum import Enum


class BridgeDirection(Enum):
    """
    Enum for the direction of a bridge.
    """

    BIDIRECTIONAL = 0
    GZ_TO_ROS = 1
    ROS_TO_GZ = 2


DIRECTION_SYMS = {
    BridgeDirection.BIDIRECTIONAL: "@",
    BridgeDirection.GZ_TO_ROS: "[",
    BridgeDirection.ROS_TO_GZ: "]",
}


@dataclass
class Bridge:
    """
    Class for a bridge between Gazebo and ROS.

    :param gz_topic: Gazebo topic name
    :param ros_topic: ROS topic name
    :param gz_type: Gazebo message type
    :param ros_type: ROS message type
    :param direction: BridgeDirection enum
    """

    gz_topic: str
    ros_topic: str
    gz_type: str
    ros_type: str
    direction: BridgeDirection

    def argument(self):
        """
        Return the argument string for the bridge.

        :return: argument string
        """
        out = f"{self.gz_topic}@{self.ros_type}{DIRECTION_SYMS[self.direction]}{self.gz_type}"
        return out

    def remapping(self):
        """
        Return the remapping tuple for the bridge.

        :return: remapping tuple
        """
        return (self.gz_topic, self.ros_topic)

"""Class for a bridge between Gazebo and ROS."""

from dataclasses import dataclass
from enum import Enum

# ROS QoS profile of the sensor topics, as the real robot publishes them
# (`qos_profile_sensor_data`: best effort, volatile, keep last 5).
SENSOR_DATA = "SENSOR_DATA"


class BridgeDirection(Enum):
    """Enum for the direction of a bridge."""

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
    """Class for a bridge between Gazebo and ROS.

    :param gz_topic: Gazebo topic name
    :param ros_topic: ROS topic name
    :param gz_type: Gazebo message type
    :param ros_type: ROS message type
    :param direction: BridgeDirection enum
    :param qos_profile: predefined ROS QoS profile of the bridge (`SENSOR_DATA`...), or `None`
        for the bridge default (reliable)
    """

    gz_topic: str
    ros_topic: str
    gz_type: str
    ros_type: str
    direction: BridgeDirection
    qos_profile: str | None = None

    def argument(self) -> str:
        """Return the argument string for the bridge.

        :return: argument string
        """
        return f"{self.gz_topic}@{self.ros_type}{DIRECTION_SYMS[self.direction]}{self.gz_type}"

    def remapping(self) -> tuple[str, str]:
        """Return the remapping tuple for the bridge.

        :return: remapping tuple
        """
        return (self.gz_topic, self.ros_topic)

    def config(self) -> dict[str, str]:
        """Return the entry of the bridge in a `parameter_bridge` YAML configuration file.

        :return: configuration entry
        """
        entry = {
            "ros_topic_name": self.ros_topic,
            "gz_topic_name": self.gz_topic,
            "ros_type_name": self.ros_type,
            "gz_type_name": self.gz_type,
            "direction": self.direction.name,
        }
        if self.qos_profile is not None:
            entry["qos_profile"] = self.qos_profile
        return entry

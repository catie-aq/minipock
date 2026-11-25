"""Factories for the Gazebo to ROS bridges of a MiniPock simulation."""

from minipock_gz.bridge import Bridge, BridgeDirection


def clock() -> Bridge:
    """Return the bridge for the simulation clock."""
    return Bridge(
        gz_topic="/clock",
        ros_topic="/clock",
        gz_type="gz.msgs.Clock",
        ros_type="rosgraph_msgs/msg/Clock",
        direction=BridgeDirection.GZ_TO_ROS,
    )


def robot_bridges(namespace: str) -> list[Bridge]:
    """Return the bridges of one robot.

    Args:
        namespace: Robot namespace without leading or trailing slash, for example `minipock_0`.

    Returns:
        The odometry, velocity command, LiDAR scan, joint states and TF bridges. The Gazebo
        topics are those set by the plugins of the robot description.
    """
    return [
        Bridge(
            gz_topic=f"{namespace}/odom",
            ros_topic=f"/{namespace}/odom",
            gz_type="gz.msgs.Odometry",
            ros_type="nav_msgs/msg/Odometry",
            direction=BridgeDirection.GZ_TO_ROS,
        ),
        Bridge(
            gz_topic=f"{namespace}/cmd_vel",
            ros_topic=f"/{namespace}/cmd_vel",
            gz_type="gz.msgs.Twist",
            ros_type="geometry_msgs/msg/Twist",
            direction=BridgeDirection.ROS_TO_GZ,
        ),
        Bridge(
            gz_topic=f"{namespace}/scan",
            ros_topic=f"/{namespace}/scan",
            gz_type="gz.msgs.LaserScan",
            ros_type="sensor_msgs/msg/LaserScan",
            direction=BridgeDirection.GZ_TO_ROS,
        ),
        Bridge(
            gz_topic=f"{namespace}/joint_states",
            ros_topic=f"/{namespace}/joint_states",
            gz_type="gz.msgs.Model",
            ros_type="sensor_msgs/msg/JointState",
            direction=BridgeDirection.GZ_TO_ROS,
        ),
        Bridge(
            gz_topic=f"/model/{namespace}/tf",
            ros_topic="/tf",
            gz_type="gz.msgs.Pose_V",
            ros_type="tf2_msgs/msg/TFMessage",
            direction=BridgeDirection.GZ_TO_ROS,
        ),
    ]

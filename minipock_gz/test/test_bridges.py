"""Tests of the Gazebo to ROS bridges of one robot."""

from minipock_gz import bridges

NAMESPACE = "minipock_1"


def _arguments() -> list[str]:
    return [bridge.argument() for bridge in bridges.robot_bridges(NAMESPACE)]


def test_robot_bridges() -> None:
    arguments = _arguments()
    assert f"/model/{NAMESPACE}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V" in arguments
    assert f"{NAMESPACE}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan" in arguments
    assert f"{NAMESPACE}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist" in arguments
    assert f"{NAMESPACE}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry" in arguments
    assert f"{NAMESPACE}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model" in arguments


def test_robot_bridges_no_scan_raw() -> None:
    assert not any("scan_raw" in argument for argument in _arguments())


def test_tf_remapped_to_global_tf() -> None:
    remappings = dict(bridge.remapping() for bridge in bridges.robot_bridges(NAMESPACE))
    assert remappings[f"/model/{NAMESPACE}/tf"] == "/tf"
    assert remappings[f"{NAMESPACE}/scan"] == f"/{NAMESPACE}/scan"
    assert remappings[f"{NAMESPACE}/cmd_vel"] == f"/{NAMESPACE}/cmd_vel"

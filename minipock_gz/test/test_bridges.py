"""Tests of the Gazebo to ROS bridges of one robot."""

from pathlib import Path

import yaml

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


def _configs() -> dict[str, dict[str, str]]:
    return {bridge.ros_topic: bridge.config() for bridge in bridges.robot_bridges(NAMESPACE)}


def test_sensor_topics_best_effort_like_real_robot() -> None:
    # raw_data_transformer publishes odom and scan with qos_profile_sensor_data.
    configs = _configs()
    assert configs[f"/{NAMESPACE}/scan"]["qos_profile"] == "SENSOR_DATA"
    assert configs[f"/{NAMESPACE}/odom"]["qos_profile"] == "SENSOR_DATA"


def test_other_topics_keep_default_qos() -> None:
    configs = _configs()
    for topic in ("/tf", f"/{NAMESPACE}/cmd_vel", f"/{NAMESPACE}/joint_states"):
        assert "qos_profile" not in configs[topic]


def test_config_file(tmp_path: Path) -> None:
    robot_bridges = bridges.robot_bridges(NAMESPACE)
    path = Path(bridges.write_config(robot_bridges, directory=str(tmp_path)))
    assert path.parent == tmp_path
    entries = yaml.safe_load(path.read_text())
    assert [entry["ros_topic_name"] for entry in entries] == [
        bridge.ros_topic for bridge in robot_bridges
    ]
    by_topic = {entry["ros_topic_name"]: entry for entry in entries}
    assert by_topic[f"/{NAMESPACE}/scan"] == {
        "ros_topic_name": f"/{NAMESPACE}/scan",
        "gz_topic_name": f"{NAMESPACE}/scan",
        "ros_type_name": "sensor_msgs/msg/LaserScan",
        "gz_type_name": "gz.msgs.LaserScan",
        "direction": "GZ_TO_ROS",
        "qos_profile": "SENSOR_DATA",
    }
    assert by_topic[f"/{NAMESPACE}/cmd_vel"]["direction"] == "ROS_TO_GZ"
    assert by_topic["/tf"]["gz_topic_name"] == f"/model/{NAMESPACE}/tf"

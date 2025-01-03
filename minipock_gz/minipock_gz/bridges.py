from minipock_gz import bridge


def clock():
    return bridge.Bridge(
        gz_topic="/clock",
        ros_topic="/clock",
        gz_type="gz.msgs.Clock",
        ros_type="rosgraph_msgs/msg/Clock",
        direction=bridge.BridgeDirection.GZ_TO_ROS,
    )


def tf(model_name):
    return bridge.Bridge(
        gz_topic=f"/model{model_name}/tf",
        ros_topic=f"/tf",
        gz_type="gz.msgs.Pose_V",
        ros_type="tf2_msgs/msg/TFMessage",
        direction=bridge.BridgeDirection.GZ_TO_ROS,
    )


def pose(model_name):
    return bridge.Bridge(
        gz_topic=f"/model{model_name}/pose",
        ros_topic=f"{model_name}/pose",
        gz_type="gz.msgs.Pose_V",
        ros_type="tf2_msgs/msg/TFMessage",
        direction=bridge.BridgeDirection.GZ_TO_ROS,
    )


def joint_states(world_name, model_name):
    return bridge.Bridge(
        gz_topic=f"/world{world_name}/model{model_name}/joint_state",
        ros_topic=f"{model_name}/joint_states",
        gz_type="gz.msgs.Model",
        ros_type="sensor_msgs/msg/JointState",
        direction=bridge.BridgeDirection.GZ_TO_ROS,
    )


def odometry(model_name):
    return bridge.Bridge(
        gz_topic=f"{model_name}/odom",
        ros_topic=f"{model_name}/odom",
        gz_type="gz.msgs.Odometry",
        ros_type="nav_msgs/msg/Odometry",
        direction=bridge.BridgeDirection.GZ_TO_ROS,
    )


def cmd_vel(model_name):
    return bridge.Bridge(
        gz_topic=f"{model_name}/cmd_vel",
        ros_topic=f"{model_name}/cmd_vel",
        gz_type="gz.msgs.Twist",
        ros_type="geometry_msgs/msg/Twist",
        direction=bridge.BridgeDirection.ROS_TO_GZ,
    )


def scan_lidar(model_name):
    return bridge.Bridge(
        gz_topic=f"{model_name}/scan_raw",
        ros_topic=f"{model_name}/scan_raw",
        gz_type="gz.msgs.LaserScan",
        ros_type="sensor_msgs/msg/LaserScan",
        direction=bridge.BridgeDirection.GZ_TO_ROS,
    )

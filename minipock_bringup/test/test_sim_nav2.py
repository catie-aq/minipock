"""Integration test: two simulated robots localize on the shared map and each reach a Nav2 goal.

Starts `sim.launch.py` headless with `test/data/two_robots.yaml` (office world, one holonomic and
one differential robot) and checks topics, TF tree, the single map server and one navigation goal
per robot. Waits are counted in simulated time (`/clock`): the test holds in software rendering,
slower than real time, as well as with a GPU.

Run it alone, it takes minutes: `colcon test --packages-select minipock_bringup
--pytest-args -m integration`; unit test runs exclude it with `-m "not integration"`.
"""

import math
import os
import threading
import time
import unittest
from collections.abc import Callable
from pathlib import Path

import launch_testing
import pytest
import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

FLEET_FILE = Path(__file__).resolve().parent / "data" / "two_robots.yaml"
ROBOTS = ("minipock_0", "minipock_1")

# Goals in the `map` frame, (x, y, yaw), read on office.pgm (resolution 0.05 m, origin
# (-1.3, -12.65)): free cells 1 to 1.5 m from the start of two_robots.yaml, in line of sight,
# with at least 0.45 m to the nearest occupied or unknown cell along the straight segment, and
# heading in the direction of travel. The robots move apart: minipock_0 south-west in the open
# space, minipock_1 north between the desks.
GOALS = {
    # Start (6.2, -5.5): 1.03 m away, clearance 0.46 m at the goal.
    "minipock_0": (5.3, -6.0, -2.63),
    # Start (8.05, -6.3): 1.28 m away, clearance 0.65 m at the goal.
    "minipock_1": (8.5, -5.1, 1.21),
}
# Distance between the localized pose and the goal once Nav2 reports success: goal checker
# tolerance (0.15 m) plus margin for the AMCL estimate moving after the last control cycle.
GOAL_REACHED_TOLERANCE = 0.25

# Simulated-time limits.
TOPICS_TIMEOUT = 120.0
TF_TIMEOUT = 180.0
NAV2_ACTIVE_TIMEOUT = 180.0
GOAL_TIMEOUT = 300.0
# Wall-clock guard, as a multiple of a simulated limit, in case `/clock` stops: software
# rendering runs well below real time, but not twenty times below.
WALL_FACTOR = 20.0
POLL_PERIOD = 0.2
# `/clock` must reach the test node within this wall time once the robot topics arrived;
# otherwise every simulated-time wait would fall back on the wall-clock guard (up to 100 min).
CLOCK_WALL_TIMEOUT = 30.0
NO_CLOCK = (
    "/clock not received by the test node {:.0f} s after the robot topics, while the simulation "
    "runs. Seen with stale Fast DDS shared-memory segments in /dev/shm: run `fastdds shm clean` "
    "(the test already uses the UDPv4 transport unless FASTDDS_BUILTIN_TRANSPORTS is set)."
)


@pytest.mark.launch_test
@pytest.mark.integration
def generate_test_description() -> LaunchDescription:
    # The plugin of launch_testing copies the marks of this function onto the test module, so
    # `-m integration` selects it and `-m "not integration"` leaves it out. A module-level
    # `pytestmark` would be ignored.
    # UDPv4 only, for the simulation and this test node: the shared-memory transport depends on
    # /dev/shm, which the container shares with the host, and stale segments left there by killed
    # runs have kept /clock from reaching the test node. Two robots use little bandwidth.
    os.environ.setdefault("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4")
    sim_launch = Path(get_package_share_directory("minipock_bringup")) / "launch" / "sim.launch.py"
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(sim_launch)),
                launch_arguments={
                    "config": str(FLEET_FILE),
                    "headless": "true",
                    "rviz": "false",
                }.items(),
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestSimNav2(unittest.TestCase):
    """Checks run in order (numbered names) against the running simulation."""

    node: Node
    executor: MultiThreadedExecutor
    spin_thread: threading.Thread
    tf_buffer: Buffer
    tf_listener: TransformListener
    tf_pairs: set[tuple[str, str]]
    topics_seen: float | None = None

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()
        cls.node = Node(
            "test_sim_nav2", parameter_overrides=[Parameter("use_sim_time", value=True)]
        )
        cls.tf_buffer = Buffer()
        cls.tf_listener = TransformListener(cls.tf_buffer, cls.node)
        cls.tf_pairs = set()
        cls.node.create_subscription(TFMessage, "/tf", cls._record_tf, 100)
        cls.executor = MultiThreadedExecutor()
        cls.executor.add_node(cls.node)
        cls.spin_thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.spin_thread.start()

    @classmethod
    def tearDownClass(cls) -> None:
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown()
        cls.spin_thread.join(timeout=5.0)

    @classmethod
    def _record_tf(cls, msg: TFMessage) -> None:
        for transform in msg.transforms:
            cls.tf_pairs.add((transform.header.frame_id, transform.child_frame_id))

    def _now(self) -> float:
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _wait_for(self, condition: Callable[[], bool], timeout: float) -> bool:
        """Wait until `condition` holds, for at most `timeout` simulated seconds.

        The simulated countdown starts with the first `/clock` message; a wall-clock guard of
        `WALL_FACTOR * timeout` covers a simulation that never starts or stalls.
        """
        wall_deadline = time.monotonic() + WALL_FACTOR * timeout
        start = None
        while time.monotonic() < wall_deadline:
            if condition():
                return True
            now = self._now()
            if now <= 0.0 and self._clock_missing():
                return condition()
            if now > 0.0:
                start = now if start is None else start
                if now - start > timeout:
                    return condition()
            time.sleep(POLL_PERIOD)
        return condition()

    def _clock_missing(self) -> bool:
        """Tell whether `/clock` is still missing long after the robot topics arrived."""
        seen = type(self).topics_seen
        return seen is not None and time.monotonic() - seen > CLOCK_WALL_TIMEOUT

    def _wait_for_message(
        self, msg_type: type, topic: str, qos: QoSProfile, timeout: float
    ) -> bool:
        received = threading.Event()
        subscription = self.node.create_subscription(
            msg_type, topic, lambda _msg: received.set(), qos
        )
        try:
            return self._wait_for(received.is_set, timeout)
        finally:
            self.node.destroy_subscription(subscription)

    def _is_active(self, manager: str) -> bool:
        """Return whether the lifecycle manager `manager` reports its nodes active."""
        client = self.node.create_client(Trigger, f"{manager}/is_active")
        try:
            if not client.service_is_ready():
                return False
            future = client.call_async(Trigger.Request())
            wall_deadline = time.monotonic() + 5.0
            while not future.done() and time.monotonic() < wall_deadline:
                time.sleep(0.05)
            return future.done() and future.result() is not None and future.result().success
        finally:
            self.node.destroy_client(client)

    def _pose_in_map(self, robot: str) -> tuple[float, float] | None:
        try:
            transform = self.tf_buffer.lookup_transform("map", f"{robot}/base_footprint", Time())
        except Exception:  # noqa: BLE001 - any TF failure means no pose yet
            return None
        return transform.transform.translation.x, transform.transform.translation.y

    def test_1_topics(self) -> None:
        for robot in ROBOTS:
            for topic, msg_type in (("scan", LaserScan), ("odom", Odometry)):
                name = f"/{robot}/{topic}"
                assert self._wait_for_message(
                    msg_type, name, qos_profile_sensor_data, TOPICS_TIMEOUT
                ), f"nothing on {name} within {TOPICS_TIMEOUT} s"
        type(self).topics_seen = time.monotonic()
        while self._now() <= 0.0 and not self._clock_missing():
            time.sleep(POLL_PERIOD)
        assert self._now() > 0.0, NO_CLOCK.format(CLOCK_WALL_TIMEOUT)

    def test_2_tf_tree(self) -> None:
        for robot in ROBOTS:
            pair = ("map", f"{robot}/odom")
            assert self._wait_for(lambda pair=pair: pair in self.tf_pairs, TF_TIMEOUT), (
                f"{pair[0]} -> {pair[1]} not published within {TF_TIMEOUT} s"
            )
            assert self._wait_for(
                lambda robot=robot: self._pose_in_map(robot) is not None, TF_TIMEOUT
            ), f"map -> {robot}/base_footprint unavailable within {TF_TIMEOUT} s"
        # Each `map -> <ns>/odom` comes from the AMCL of the robot: both are /tf publishers.
        publishers = {
            (info.node_namespace, info.node_name)
            for info in self.node.get_publishers_info_by_topic("/tf")
        }
        for robot in ROBOTS:
            assert (f"/{robot}", "amcl") in publishers, f"/{robot}/amcl does not publish /tf"

    def test_3_single_map_server(self) -> None:
        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        assert self._wait_for_message(OccupancyGrid, "/map", latched, TOPICS_TIMEOUT), (
            f"nothing on /map within {TOPICS_TIMEOUT} s"
        )
        map_servers = [
            (namespace, name)
            for name, namespace in self.node.get_node_names_and_namespaces()
            if name == "map_server"
        ]
        assert map_servers == [("/", "map_server")], f"map servers: {map_servers}"

    def test_4_reach_goal(self) -> None:
        for robot in ROBOTS:
            for manager in ("lifecycle_manager_localization", "lifecycle_manager_navigation"):
                name = f"/{robot}/{manager}"
                assert self._wait_for(
                    lambda name=name: self._is_active(name), NAV2_ACTIVE_TIMEOUT
                ), f"{name} not active within {NAV2_ACTIVE_TIMEOUT} s"

        clients = {}
        goal_futures = {}
        for robot, (x, y, yaw) in GOALS.items():
            client = ActionClient(self.node, NavigateToPose, f"/{robot}/navigate_to_pose")
            assert client.wait_for_server(timeout_sec=30.0), f"/{robot}/navigate_to_pose missing"
            goal = NavigateToPose.Goal()
            goal.pose = PoseStamped()
            goal.pose.header.frame_id = "map"
            goal.pose.header.stamp = self.node.get_clock().now().to_msg()
            goal.pose.pose.position.x = x
            goal.pose.pose.position.y = y
            goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
            clients[robot] = client
            goal_futures[robot] = client.send_goal_async(goal)

        assert self._wait_for(lambda: all(f.done() for f in goal_futures.values()), 30.0), (
            "goals not answered"
        )
        sent = self._now()
        finished: dict[str, float] = {}
        result_futures = {}
        for robot, future in goal_futures.items():
            handle = future.result()
            assert handle is not None, f"{robot}: no goal handle"
            assert handle.accepted, f"{robot}: goal rejected"
            result_futures[robot] = handle.get_result_async()
            result_futures[robot].add_done_callback(
                lambda _future, robot=robot: finished.setdefault(robot, self._now())
            )

        self._wait_for(lambda: all(f.done() for f in result_futures.values()), GOAL_TIMEOUT)
        for robot, future in result_futures.items():
            pose = self._pose_in_map(robot)
            assert future.done(), f"{robot}: goal not finished within {GOAL_TIMEOUT} s at {pose}"
            status = future.result().status
            assert status == GoalStatus.STATUS_SUCCEEDED, (
                f"{robot}: goal ended with status {status} at {pose}"
            )
            x, y, _yaw = GOALS[robot]
            assert pose is not None, f"{robot}: no pose in map"
            self.node.get_logger().info(
                f"{robot}: goal reached in {finished[robot] - sent:.1f} s (sim) at "
                f"({pose[0]:.2f}, {pose[1]:.2f}), goal ({x}, {y})"
            )
            assert math.dist(pose, (x, y)) < GOAL_REACHED_TOLERANCE, (
                f"{robot}: at {pose}, goal {(x, y)}"
            )
        for client in clients.values():
            client.destroy()

"""Navigation benchmark: send a list of goals to each robot and print a Markdown table.

Usage (simulation running, Nav2 active)::

    ros2 run minipock_bringup nav_benchmark --ros-args -p use_sim_time:=true
        -p robots:="[minipock_0, minipock_1]" -p goals_file:=<yaml> -p repeat:=3

The goals file maps each robot namespace to a `start` pose and a list of `goals`, each with a
`name` and a `pose` `[x, y, yaw]` in the `map` frame; `keep_yaw: true` marks a goal where the
robot must keep its heading (holonomic lateral move). The robots run their goals in parallel,
one after the other, and go back to `start` between repetitions (logged, not measured).

Per goal: result, duration (simulated time), travelled path over the distance actually covered
(start to final pose), smallest distance between the robot body and an obstacle (scan range
minus the robot radius), sign changes of the commanded angular velocity while the robot is
within 0.5 m of the goal, largest heading drift from the start pose (`keep_yaw` goals only), and
the final position and heading errors of the localized pose.

The process exits with status 1 if `/clock` does not arrive, an action server is missing or a
goal has no row.
"""

import itertools
import math
import sys
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

ROBOT_RADIUS = 0.17
# Angular commands smaller than this are not counted as a direction (deadband).
OSCILLATION_DEADBAND = 0.05
# Below this straight-line distance the path ratio is meaningless (turn in place, where the
# robot starts up to the goal tolerance away from the goal).
MIN_STRAIGHT_LINE = 0.3
# Angular commands sent closer than this to the goal count as end-of-goal oscillations.
NEAR_GOAL_DISTANCE = 0.5
GOAL_TIMEOUT = 120.0
SAMPLE_PERIOD = 0.1
# Wall-clock guards: `/clock` must arrive within CLOCK_WALL_TIMEOUT, and a simulated wait may
# last at most WALL_FACTOR times its simulated limit (software rendering is slower than real
# time, not twenty times slower).
CLOCK_WALL_TIMEOUT = 30.0
WALL_FACTOR = 20.0
SERVER_TIMEOUT = 60.0


def angle_diff(a: float, b: float) -> float:
    """Return a - b wrapped to [-pi, pi)."""
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


def path_length(points: list[tuple[float, float]]) -> float:
    """Return the length of a polyline."""
    return sum(math.dist(p, q) for p, q in itertools.pairwise(points))


def sign_changes(values: list[float], deadband: float = OSCILLATION_DEADBAND) -> int:
    """Count the sign changes of a signal, ignoring values within the deadband."""
    signs = [math.copysign(1.0, v) for v in values if abs(v) > deadband]
    return sum(1 for s, t in itertools.pairwise(signs) if s != t)


@dataclass(frozen=True)
class Goal:
    """One goal of the benchmark."""

    name: str
    pose: tuple[float, float, float]
    keep_yaw: bool = False


@dataclass(frozen=True)
class RobotPlan:
    """Start pose and goals of one robot."""

    start: tuple[float, float, float]
    goals: tuple[Goal, ...]


def load_goals(path: Path) -> dict[str, RobotPlan]:
    """Read a goals file.

    Args:
        path: YAML file, `{<robot>: {start: [x, y, yaw], goals: [{name, pose, keep_yaw}]}}`.

    Returns:
        The plan of each robot.

    Raises:
        ValueError: If a pose does not have three values.
    """
    plans = {}
    for robot, entry in yaml.safe_load(path.read_text()).items():
        poses = [entry["start"], *(goal["pose"] for goal in entry["goals"])]
        if any(len(pose) != 3 for pose in poses):  # noqa: PLR2004 - x, y, yaw
            msg = f"{path}: poses of {robot} must be [x, y, yaw]"
            raise ValueError(msg)
        goals = tuple(
            Goal(str(g["name"]), tuple(float(v) for v in g["pose"]), bool(g.get("keep_yaw")))
            for g in entry["goals"]
        )
        plans[robot] = RobotPlan(tuple(float(v) for v in entry["start"]), goals)
    return plans


@dataclass
class Measure:
    """What is recorded while a robot drives to one goal."""

    goal_xy: tuple[float, float]
    points: list[tuple[float, float]] = field(default_factory=list)
    yaws: list[float] = field(default_factory=list)
    # Angular velocity commands sent within NEAR_GOAL_DISTANCE of the goal.
    near_goal_commands: list[float] = field(default_factory=list)
    min_range: float = math.inf


@dataclass(frozen=True)
class Row:
    """One line of the result table."""

    robot: str
    goal: str
    repetition: int
    succeeded: bool
    duration: float
    path_ratio: float | None
    clearance: float
    oscillations: int
    yaw_drift: float | None
    position_error: float
    yaw_error: float


def summarize(
    measure: Measure, goal: Goal, start: tuple[float, float, float], final: tuple[float, ...]
) -> dict[str, float | None]:
    """Turn a measure into the metrics of a row.

    Args:
        measure: Samples of the goal.
        goal: The goal.
        start: Localized pose when the goal was sent.
        final: Localized pose at the end of the goal.

    Returns:
        `path_ratio` (travelled path over the start to final distance, None for a turn in
        place), `clearance`, `oscillations` (sign changes of the angular command near the
        goal), `yaw_drift` (`keep_yaw` goals only, else None), `position_error` and
        `yaw_error`.
    """
    travelled = path_length(measure.points)
    covered = math.dist(start[:2], final[:2])
    drift = max((abs(angle_diff(y, start[2])) for y in measure.yaws), default=0.0)
    return {
        "path_ratio": travelled / covered if covered > MIN_STRAIGHT_LINE else None,
        "clearance": measure.min_range - ROBOT_RADIUS,
        "oscillations": sign_changes(measure.near_goal_commands),
        "yaw_drift": drift if goal.keep_yaw else None,
        "position_error": math.dist(final[:2], goal.pose[:2]),
        "yaw_error": abs(angle_diff(final[2], goal.pose[2])),
    }


def markdown_table(rows: list[Row]) -> str:
    """Format the rows as a Markdown table."""
    lines = [
        (
            "| Robot | Goal | Rep | Result | Time (s) | Path / covered | Min clearance (m) "
            "| wz sign changes < 0.5 m | Yaw drift (rad) | Final error (m, rad) |"
        ),
        "|---|---|---|---|---|---|---|---|---|---|",
    ]
    for r in rows:
        ratio = "-" if r.path_ratio is None else f"{r.path_ratio:.2f}"
        drift = "-" if r.yaw_drift is None else f"{r.yaw_drift:.2f}"
        lines.append(
            f"| {r.robot} | {r.goal} | {r.repetition} | {'ok' if r.succeeded else 'FAIL'} "
            f"| {r.duration:.1f} | {ratio} | {r.clearance:.2f} | {r.oscillations} "
            f"| {drift} | {r.position_error:.2f}, {r.yaw_error:.2f} |"
        )
    return "\n".join(lines)


class NavBenchmark(Node):
    """Drive each robot through its goals and record the metrics."""

    def __init__(self) -> None:
        """Declare the parameters, read the goals and create the clients and subscriptions."""
        super().__init__("nav_benchmark")
        self.declare_parameter(
            "robots",
            ["minipock_0", "minipock_1"],
            ParameterDescriptor(
                description="Namespaces of the robots to run the goals for.",
                type=ParameterType.PARAMETER_STRING_ARRAY,
                read_only=True,
            ),
        )
        self.declare_parameter(
            "goals_file",
            "",
            ParameterDescriptor(
                description="Path of the YAML file listing the goals of each robot.",
                type=ParameterType.PARAMETER_STRING,
                read_only=True,
            ),
        )
        self.declare_parameter(
            "repeat",
            1,
            ParameterDescriptor(
                description="Number of times the whole goal suite is run.",
                type=ParameterType.PARAMETER_INTEGER,
                read_only=True,
            ),
        )
        robots = list(self.get_parameter("robots").value)
        self.repeat = int(self.get_parameter("repeat").value)
        plans = load_goals(Path(str(self.get_parameter("goals_file").value)))
        self.plans = {robot: plans[robot] for robot in robots}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.action_clients = {
            robot: ActionClient(self, NavigateToPose, f"/{robot}/navigate_to_pose")
            for robot in robots
        }
        self.measures: dict[str, Measure | None] = dict.fromkeys(robots)
        self.rows: list[Row] = []
        self.errors: list[str] = []
        self.lock = threading.Lock()
        for robot in robots:
            self.create_subscription(
                LaserScan,
                f"/{robot}/scan",
                lambda msg, robot=robot: self._on_scan(robot, msg),
                qos_profile_sensor_data,
            )
            self.create_subscription(
                Twist, f"/{robot}/cmd_vel", lambda msg, robot=robot: self._on_cmd(robot, msg), 10
            )
        self.create_timer(SAMPLE_PERIOD, self._sample)

    def now(self) -> float:
        """Return the node time in seconds (simulated time with use_sim_time)."""
        return self.get_clock().now().nanoseconds * 1e-9

    def pose(self, robot: str) -> tuple[float, float, float] | None:
        """Return the localized pose of a robot in `map`, or None."""
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", f"{robot}/base_footprint", Time()
            ).transform
        except Exception:  # noqa: BLE001 - no pose yet
            return None
        q = transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return transform.translation.x, transform.translation.y, yaw

    def _on_scan(self, robot: str, msg: LaserScan) -> None:
        measure = self.measures[robot]
        valid = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if measure is not None and valid:
            measure.min_range = min(measure.min_range, *valid)

    def _on_cmd(self, robot: str, msg: Twist) -> None:
        measure = self.measures[robot]
        near = (
            measure is not None
            and measure.points
            and math.dist(measure.points[-1], measure.goal_xy) < NEAR_GOAL_DISTANCE
        )
        if near:
            measure.near_goal_commands.append(msg.angular.z)

    def _sample(self) -> None:
        for robot, measure in self.measures.items():
            pose = self.pose(robot)
            if measure is not None and pose is not None:
                measure.points.append(pose[:2])
                measure.yaws.append(pose[2])

    def navigate(self, robot: str, pose: tuple[float, float, float]) -> tuple[bool, float]:
        """Send one goal and wait for its result.

        Returns:
            Whether the goal succeeded, and its duration in simulated seconds.
        """
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x, goal.pose.pose.position.y = pose[0], pose[1]
        goal.pose.pose.orientation.z = math.sin(pose[2] / 2.0)
        goal.pose.pose.orientation.w = math.cos(pose[2] / 2.0)
        start = self.now()
        handle_future = self.action_clients[robot].send_goal_async(goal)
        if not self._wait(handle_future.done, 10.0) or not handle_future.result().accepted:
            return False, self.now() - start
        result_future = handle_future.result().get_result_async()
        if not self._wait(result_future.done, GOAL_TIMEOUT):
            handle_future.result().cancel_goal_async()
            return False, self.now() - start
        return result_future.result().status == GoalStatus.STATUS_SUCCEEDED, self.now() - start

    def _wait(self, condition: Callable[[], bool], timeout: float) -> bool:
        """Wait `timeout` simulated seconds at most, and WALL_FACTOR times that in wall time."""
        start = self.now()
        wall_deadline = time.monotonic() + WALL_FACTOR * timeout
        while not condition():
            if self.now() - start > timeout or time.monotonic() > wall_deadline:
                return False
            time.sleep(0.05)
        return True

    def wait_for_clock(self) -> bool:
        """Wait for the first `/clock` message, CLOCK_WALL_TIMEOUT seconds at most."""
        deadline = time.monotonic() + CLOCK_WALL_TIMEOUT
        while self.now() <= 0.0:
            if time.monotonic() > deadline:
                return False
            time.sleep(0.1)
        return True

    def run_robot(self, robot: str) -> None:
        """Run the goals of one robot `repeat` times; record any error instead of raising."""
        try:
            self._run_robot(robot)
        except Exception as error:  # noqa: BLE001 - reported as missing rows
            with self.lock:
                self.errors.append(f"{robot}: {error!r}")
            self.get_logger().error(f"{robot}: worker stopped: {error!r}")

    def _run_robot(self, robot: str) -> None:
        plan = self.plans[robot]
        if not self.action_clients[robot].wait_for_server(timeout_sec=SERVER_TIMEOUT):
            msg = f"/{robot}/navigate_to_pose not available within {SERVER_TIMEOUT} s"
            raise RuntimeError(msg)
        for repetition in range(1, self.repeat + 1):
            if repetition > 1:
                succeeded, duration = self.navigate(robot, plan.start)
                log = self.get_logger().info if succeeded else self.get_logger().warning
                log(
                    f"{robot}: back to start {'ok' if succeeded else 'FAILED'} in {duration:.1f} s"
                )
            for goal in plan.goals:
                start = self.pose(robot) or plan.start
                self.measures[robot] = Measure(goal_xy=goal.pose[:2])
                succeeded, duration = self.navigate(robot, goal.pose)
                time.sleep(1.0)
                measure, self.measures[robot] = self.measures[robot], None
                metrics = summarize(measure, goal, start, self.pose(robot) or start)
                row = Row(robot, goal.name, repetition, succeeded, duration, **metrics)
                with self.lock:
                    self.rows.append(row)
                self.get_logger().info(f"{row}")


def main() -> None:
    """Run the benchmark and print the table."""
    rclpy.init()
    node = NavBenchmark()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    status = 0
    try:
        if not node.wait_for_clock():
            print(
                f"no /clock within {CLOCK_WALL_TIMEOUT} s: is the simulation running, with "
                "use_sim_time:=true? Stale Fast DDS shared-memory segments also block it: "
                "`fastdds shm clean`, or FASTDDS_BUILTIN_TRANSPORTS=UDPv4.",
                file=sys.stderr,
            )
            status = 1
        else:
            workers = [threading.Thread(target=node.run_robot, args=(r,)) for r in node.plans]
            for worker in workers:
                worker.start()
            for worker in workers:
                worker.join()
            rows = sorted(node.rows, key=lambda r: (r.robot, r.repetition))
            print(markdown_table(rows), flush=True)
            for robot, plan in node.plans.items():
                expected = node.repeat * len(plan.goals)
                count = sum(1 for r in rows if r.robot == robot)
                if count != expected:
                    print(f"{robot}: {count} rows out of {expected}", file=sys.stderr)
                    status = 1
            for error in node.errors:
                print(error, file=sys.stderr)
    except (KeyboardInterrupt, ExternalShutdownException):
        status = 1
    finally:
        # Shut the context down first: it wakes the spinning executor, which then returns.
        rclpy.try_shutdown()
        spinner.join(timeout=5.0)
        executor.shutdown()
        node.destroy_node()
    sys.exit(status)


if __name__ == "__main__":
    main()

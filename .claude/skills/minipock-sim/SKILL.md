---
name: minipock-sim
description: Start the MiniPock fleet simulation (Gazebo, Nav2, AMCL, RViz) with sim.launch.py, headless or with a GUI, with or without GPU, then probe topics, TF and lifecycle states, send a NavigateToPose goal and stop everything cleanly. Use when asked to run, launch, test, probe or debug the simulation, check that Nav2 is active, send a navigation goal, or when the simulation is slow or leaves processes behind.
---

# Run and probe the MiniPock simulation

Commands go through `scripts/docker-run.sh` (see `minipock-test` for the image, workspace and domain rules). Build first: `scripts/docker-run.sh "colcon build --symlink-install --merge-install"`.

## Launch arguments

```bash
scripts/docker-run.sh "ros2 launch minipock_bringup sim.launch.py --show-args"
```

`config:=<file>` (fleet YAML: a path, or a name from `minipock_bringup/config`, default `minipocks.yaml`: 2 robots in the `office` world), `headless:=true|false` (Gazebo server only), `rviz:=true|false`, `slam:=true|false` (Cartographer maps instead of `map_server` and AMCL; the fleet must have exactly one robot, and goals are in the Cartographer `map` frame, which starts at the robot's start pose). Fleet size, kinematics (`holonomic`/`differential`), start positions and world come from the YAML (`fleet:` keys are the robot ids; robot `n` gets namespace `minipock_<n>`); maximum 18 robots.

## Choose the rendering

The simulated LiDAR (`gpu_lidar`) is the cost driver.

| Mode | Command prefix | Speed measured |
|------|----------------|----------------|
| NVIDIA GPU, headless | `scripts/docker-run.sh --gpu` | scan 10 Hz, real-time factor about 1.2 (office, 2 robots) |
| NVIDIA GPU, with GUI | `scripts/docker-run.sh --gpu --gui` (RViz and Gazebo on the host display; `xhost +local:` once if the window does not open) | same |
| Software, headless | `DOCKER_EXTRA="-e LIBGL_ALWAYS_SOFTWARE=1" scripts/docker-run.sh` | far below real time (office, 2 robots: scan about 1 Hz, simulation clock almost still) |

Without GPU use `headless:=true rviz:=false`, give Nav2 several minutes to come up, and prefer a smaller world (`empty_room_world`) for quick checks. Several simulations at once slow each other down.

## Start headless in the background, probe, stop

Run the whole session in ONE container call, so the probes share the launch's domain and the stop kills the whole process group. In a script file inside the workspace (`$MINIPOCK_WS/sim.sh`), run with `scripts/docker-run.sh --gpu "bash sim.sh"`:

```bash
set -m                                   # job control: the launch gets its own process group
ros2 launch minipock_bringup sim.launch.py headless:=true rviz:=false > /tmp/home/sim.log 2>&1 &
pid=$!
sleep 60                                 # software rendering: wait much longer, and poll instead
# ... probes below ...
kill -INT -- -$pid; sleep 8              # Ctrl-C equivalent on the whole group
kill -KILL -- -$pid 2>/dev/null; true    # leftovers (gz sim often needs it)
```

Never launch it from a plain `&` outside the container call: when the call ends the container is killed with processes half stopped. From the host, `docker ps --filter ancestor=<image>` lists running containers and `docker stop <name>` ends one. Check nothing is left with `ps -eo args | grep "[g]z sim"` inside the call.

## Probes

```bash
ros2 node list | sort
ros2 topic list | grep -E "minipock_0/(scan|odom|cmd_vel)$|^/map$|^/clock"
timeout 15 ros2 topic hz /minipock_0/scan                         # 10 Hz with GPU
ros2 topic echo --once --field clock.sec /clock 2>/dev/null | grep -E '^[0-9]+$'   # twice, 10 s apart: sim seconds per wall second (12 with GPU)
ros2 lifecycle get /map_server                                    # also /minipock_0/amcl, bt_navigator, controller_server
timeout 15 ros2 service call /minipock_0/lifecycle_manager_navigation/is_active std_srvs/srv/Trigger
timeout 20 ros2 run tf2_ros tf2_echo map minipock_0/base_footprint
grep -iE "error|fatal|exception|failed" /tmp/home/sim.log | sort | uniq -c | sort -rn | head
```

Expected: every Nav2 server `active`, `is_active` `success=True`, TF `map -> minipock_<n>/base_footprint` equal to the start position of the YAML (set by the AMCL initial pose), a single `/tf` tree, topics relative to the namespace (`/minipock_0/scan`, never `/minipock_0/minipock_0/...`). Managers: `lifecycle_manager_map` (root), `<ns>/lifecycle_manager_localization`, `<ns>/lifecycle_manager_navigation`.

## Send a goal

Only once the robot's `bt_navigator` is `active` and `map -> <ns>/base_footprint` exists. The goal is in the `map` frame; pick a free cell of the office map: `(6.95, -5.5)` is free and 0.75 m from the start of `minipock_0`; `(6.2, -4.5)` is inside a wall and never completes. Without `--feedback` the command prints only the final result; always wrap it in `timeout`.

```bash
PYTHONUNBUFFERED=1 timeout 100 ros2 action send_goal /minipock_0/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 6.95, y: -5.5}, orientation: {w: 1.0}}}}" --feedback
```

`Goal accepted with ID: ...` appears at once, then feedback (`distance_remaining`, `number_of_recoveries`, `navigation_time`), then `Result` with `status: 4` (SUCCEEDED; 5 canceled, 6 aborted). Piping the output through `grep`/`tail` hides everything until the command ends unless `PYTHONUNBUFFERED=1` is set. An accepted goal is not a succeeded goal: check `status` and `number_of_recoveries` before concluding. With software rendering the clock barely advances: the goal takes minutes or times out; that does not mean Nav2 is broken.

## Pitfalls

- Parallel simulations on one `ROS_DOMAIN_ID`/`GZ_PARTITION` see each other (odometry, scans from another robot), and so do two launches from the same worktree: start one at a time and `docker stop` the previous container first. The script derives them from the source path; do not override unless you mean it.
- `ros2` calls from the host do not reach the container's graph unless the host uses the same domain and `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` (printed by `scripts/docker-run.sh --print x`); prefer running probes in the same container call.
- `/clock` not advancing: Gazebo is paused or the world is still loading (software rendering loads slowly).
- A `ros2 launch` killed with `kill -9` leaves `gz sim` and Nav2 servers alive: always use the INT-then-KILL sequence on the process group.
- After editing `minipock_gz/worlds`, regenerate the Nav2 map (see `CLAUDE.md`).

---
name: minipock-test
description: Build the MiniPock ROS 2 packages and run their tests (unit, integration, full gate) in the Docker simulation image, and read the colcon results. Use when asked to build, run the tests, check that tests pass, run the integration test, run the CI gate locally, or when a colcon build or test fails in this repository.
---

# Build and test MiniPock

Every ROS command runs in the simulation image through `scripts/docker-run.sh`, which mounts the repo on `/workspaces/src/minipock` and the workspace (`build/`, `install/`, `log/`) on `/workspaces`. It sources ROS and `install/setup.bash` itself. Run it from the repository root.

- Workspace: `$MINIPOCK_WS`, default `../ws` next to the repo. In a git worktree, give it its own (`MINIPOCK_WS=../ws-<name>`) so builds do not mix.
- Image: `IMAGE`, default `ghcr.io/catie-aq/minipock:<version>-simulation` (version of `minipock/package.xml`). After a local rebuild of the Dockerfile use `IMAGE=minipock:dev-simulation`.
- Each worktree gets its own `ROS_DOMAIN_ID` and `GZ_PARTITION`, so parallel runs do not see each other.

## Build

```bash
scripts/docker-run.sh "colcon build --symlink-install --merge-install"
scripts/docker-run.sh "colcon build --symlink-install --merge-install --packages-select minipock_bringup"
```

`--merge-install` is required: `world.launch.py` adds one `install/share` to `GZ_SIM_RESOURCE_PATH`, and the world URIs (`model://minipock_gz/...`, `package://...`) resolve against it.

## Unit tests (no simulation)

```bash
scripts/docker-run.sh "colcon test --merge-install --packages-select minipock_bringup --pytest-args -m 'not integration' --event-handlers console_direct+ && colcon test-result --verbose"
```

Drop `--packages-select` to test every package. `-m 'not integration'` keeps the Gazebo test out of the unit run.

## Integration test (starts Gazebo and Nav2)

Tests marked `integration` (`@pytest.mark.integration`, `minipock_bringup/test/test_sim_nav2.py`) start the simulation, check that Nav2 comes up and send one goal per robot. The marker is registered in `minipock_bringup/setup.cfg`. They are opt-in (`minipock_bringup/test/conftest.py`): a plain `colcon test` or `pytest` skips them; they run with `-m integration` or with `MINIPOCK_INTEGRATION=1` in the environment (for an integration CI job). Unit runs may still pass `-m 'not integration'`. Without a GPU force software rendering; the LiDAR then runs far below real time (see `minipock-sim`), so allow minutes:

```bash
scripts/docker-run.sh --gpu "colcon test --merge-install --packages-select minipock_bringup --pytest-args -m integration --event-handlers console_direct+ && colcon test-result --verbose"
DOCKER_EXTRA="-e LIBGL_ALWAYS_SOFTWARE=1" scripts/docker-run.sh "<same command>"
```

The test sets `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` for the simulation and its own node (stale Fast DDS shared-memory segments in `/dev/shm` once kept `/clock` from the test node), and fails within 30 s of wall time if `/clock` does not arrive after the robot topics. For your own probes in the same graph, `fastdds shm clean` in the container removes stale segments.

`colcon test-result` must report at least one test. `0 tests` means the check did not run (marker missing, wrong package, test file absent): that is a failure of the check, not a pass.

## Read the results

- `colcon test-result` lists only failing files; `--verbose` lists every case; `--all` includes passing packages.
- Details per package: `build/<pkg>/pytest.xml`, console output with `--event-handlers console_direct+`, logs in `log/latest_test/<pkg>/`.
- A package without tests (for example `minipock_gz` stubs) can show as failed with pytest exit code 5: no test collected, not a defect.
- `colcon test-result` reads every result already in `build/`: delete `build/<pkg>/pytest.xml` to forget a stale one.

## Full gate (what CI runs)

```bash
scripts/gate.sh [<git-ref>]
```

Exports the commit to `../ws-gate` (with the `minipock_msgs` revision the commit pins), runs `pre-commit` on a clean checkout without the submodule (as CI does), then builds and runs the unit tests in the image (`IMAGE` defaults to `minipock:dev-simulation`; set `IMAGE=ghcr.io/catie-aq/minipock:<version>-simulation` to use the published image). Exit status is non-zero on a pre-commit failure, a build failure, a failed test or zero tests run. It tests the committed state, not the working tree: commit first. The pinned submodule revision must exist locally (`git submodule update --init`).

Lint without Docker: `uvx ruff check . && uvx ruff format --check .` and `uvx pre-commit run --all-files`.

## Known pitfalls

- **Stale symlinks after deleting a file** (launch, config, rviz) with `--symlink-install`: the next build fails with `error: can't copy '/workspaces/build/<pkg>/launch/<file>': doesn't exist or not a regular file`. Fix, then rebuild the package:
  ```bash
  scripts/docker-run.sh "rm -rf build/<pkg> install/share/<pkg>/launch && colcon build --symlink-install --merge-install --packages-select <pkg>"
  ```
  Replace `launch` with the folder of the deleted file. Do the same after pulling a change that removes files.
- **Mixed install layouts**: a workspace built once without `--merge-install` breaks the Gazebo paths; delete `build install log` and rebuild.
- **Pre-commit passes locally and fails in CI** (typically ruff `I001` on `minipock_msgs` imports): CI has no submodule. Use `scripts/gate.sh`, or `known-third-party` in `pyproject.toml` already covers it.
- **Files created by the container** belong to your uid (the script runs with `-u`), so they can be removed from the host.
- **Parallel simulations** on one domain disturb each other (odometry and scans from another run); the script avoids it unless you set `ROS_DOMAIN_ID` by hand.

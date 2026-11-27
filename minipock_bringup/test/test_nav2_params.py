"""Tests for the Nav2 parameter rendering and the shared localization actions."""

import itertools
import math
from collections.abc import Iterable
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext, LaunchDescriptionEntity
from launch.utilities import perform_substitutions
from launch_ros.actions import Node

from minipock_bringup.localization import localization_actions
from minipock_bringup.nav2_params import merge_params, render_params_file

MODES = ("differential", "holonomic")

# Real robot limits, from the firmware (minipock_mbed-rbdc).
MAX_LINEAR_VELOCITY = 2.0
MAX_LINEAR_ACCEL = 0.7
MAX_LINEAR_DECEL = 3.4
MAX_ANGULAR_VELOCITY = 3 * math.pi
MAX_ANGULAR_ACCEL = math.pi


def _render(mode: str, override: str = "", **pose: float) -> dict:
    """Render the parameters of `minipock_0` and read them back as a dictionary."""
    position = {"x": 1.5, "y": -2.0, "yaw": 0.5} | pose
    path = render_params_file(mode, "minipock_0", override=override, **position)
    return yaml.safe_load(Path(path).read_text())


def _params(params: dict, *keys: str) -> dict:
    """Return the `ros__parameters` of the node at `keys`."""
    node = params
    for key in keys:
        node = node[key]
    return node["ros__parameters"]


def test_merge_overlay_wins() -> None:
    assert merge_params({"a": {"b": 1, "c": 2}}, {"a": {"b": 3}}) == {"a": {"b": 3, "c": 2}}


def test_merge_keeps_common_keys() -> None:
    merged = merge_params({"a": 1, "b": {"c": 2}}, {"d": 4})
    assert merged == {"a": 1, "b": {"c": 2}, "d": 4}


def test_merge_replaces_lists() -> None:
    merged = merge_params({"a": {"l": [1, 2, 3]}}, {"a": {"l": [4]}})
    assert merged == {"a": {"l": [4]}}


def test_merge_does_not_modify_inputs() -> None:
    base = {"a": {"b": 1}}
    overlay = {"a": {"c": 2}}
    merge_params(base, overlay)
    assert base == {"a": {"b": 1}}
    assert overlay == {"a": {"c": 2}}


def test_merge_rejects_none() -> None:
    with pytest.raises(ValueError, match=r"a\.b"):
        merge_params({"a": {"b": 1}}, {"a": {"b": None}})


@pytest.mark.parametrize(
    ("base", "where"),
    [({}, r"new\.sub\.c"), ({"new": 1}, r"new\.sub\.c"), ({"new": {"x": 1}}, r"new\.sub\.c")],
)
def test_merge_rejects_none_in_new_subdictionary(base: dict, where: str) -> None:
    with pytest.raises(ValueError, match=where):
        merge_params(base, {"new": {"sub": {"c": None}}})


def test_merge_copies_new_subdictionary() -> None:
    overlay = {"new": {"sub": {"c": 1}}}
    merged = merge_params({}, overlay)
    assert merged == overlay
    merged["new"]["sub"]["c"] = 2
    assert overlay["new"]["sub"]["c"] == 1


@pytest.mark.parametrize("mode", MODES)
def test_frames_are_prefixed(mode: str) -> None:
    params = _render(mode)
    amcl = _params(params, "amcl")
    assert amcl["base_frame_id"] == "minipock_0/base_footprint"
    assert amcl["odom_frame_id"] == "minipock_0/odom"
    assert amcl["map_topic"] == "/map"
    assert _params(params, "local_costmap", "local_costmap")["global_frame"] == "minipock_0/odom"
    assert _params(params, "global_costmap", "global_costmap")["global_frame"] == "map"
    assert _params(params, "smoother_server")["robot_base_frame"] == "minipock_0/base_footprint"
    dump = yaml.safe_dump(params)
    for marker in ("<robot_namespace>", "<x>", "<y>", "<yaw>"):
        assert marker not in dump


@pytest.mark.parametrize(
    ("mode", "amcl_model", "mppi_model"),
    [
        ("differential", "nav2_amcl::DifferentialMotionModel", "DiffDrive"),
        ("holonomic", "nav2_amcl::OmniMotionModel", "Omni"),
    ],
)
def test_motion_models(mode: str, amcl_model: str, mppi_model: str) -> None:
    params = _render(mode)
    assert _params(params, "amcl")["robot_model_type"] == amcl_model
    assert _params(params, "controller_server")["FollowPath"]["motion_model"] == mppi_model


def test_holonomic_allows_lateral_velocity() -> None:
    holonomic = _render("holonomic")
    smoother = _params(holonomic, "velocity_smoother")
    controller = _params(holonomic, "controller_server")
    assert smoother["max_velocity"][1] > 0
    assert smoother["max_accel"][1] > 0
    assert controller["FollowPath"]["vy_max"] > 0
    assert controller["min_y_velocity_threshold"] < controller["FollowPath"]["vy_max"]

    differential = _render("differential")
    smoother = _params(differential, "velocity_smoother")
    assert smoother["max_velocity"][1] == 0
    assert _params(differential, "controller_server")["FollowPath"]["vy_max"] == 0


def test_holonomic_critics_without_heading_preference() -> None:
    # A holonomic robot keeps its heading on lateral and backward moves.
    critics = _params(_render("holonomic"), "controller_server")["FollowPath"]["critics"]
    assert "PreferForwardCritic" not in critics
    assert "PathAngleCritic" not in critics
    assert "TwirlingCritic" in critics
    differential = _params(_render("differential"), "controller_server")["FollowPath"]
    assert "PreferForwardCritic" in differential["critics"]


def test_initial_pose_rendered() -> None:
    amcl = _params(_render("differential", x=3.25, y=-1.5, yaw=1.2), "amcl")
    assert amcl["set_initial_pose"] is True
    assert amcl["initial_pose"]["x"] == pytest.approx(3.25)
    assert amcl["initial_pose"]["y"] == pytest.approx(-1.5)
    assert amcl["initial_pose"]["yaw"] == pytest.approx(1.2)


@pytest.mark.parametrize("mode", MODES)
def test_limits_within_robot_capabilities(mode: str) -> None:
    params = _render(mode)
    smoother = _params(params, "velocity_smoother")
    linear_max = [*smoother["max_velocity"][:2], *(-v for v in smoother["min_velocity"][:2])]
    assert all(v <= MAX_LINEAR_VELOCITY for v in linear_max)
    assert abs(smoother["max_velocity"][2]) <= MAX_ANGULAR_VELOCITY
    assert abs(smoother["min_velocity"][2]) <= MAX_ANGULAR_VELOCITY
    assert all(a <= MAX_LINEAR_ACCEL for a in smoother["max_accel"][:2])
    assert smoother["max_accel"][2] <= MAX_ANGULAR_ACCEL
    assert all(-d <= MAX_LINEAR_DECEL for d in smoother["max_decel"][:2])
    assert -smoother["max_decel"][2] <= MAX_ANGULAR_ACCEL

    mppi = _params(params, "controller_server")["FollowPath"]
    assert mppi["vx_max"] <= MAX_LINEAR_VELOCITY
    assert -mppi["vx_min"] <= MAX_LINEAR_VELOCITY
    assert mppi["vy_max"] <= MAX_LINEAR_VELOCITY
    assert mppi["wz_max"] <= MAX_ANGULAR_VELOCITY
    assert mppi["ax_max"] <= MAX_LINEAR_ACCEL
    assert mppi["ay_max"] <= MAX_LINEAR_ACCEL
    assert -mppi["ax_min"] <= MAX_LINEAR_DECEL
    assert -mppi["ay_min"] <= MAX_LINEAR_DECEL
    assert mppi["az_max"] <= MAX_ANGULAR_ACCEL

    behavior = _params(params, "behavior_server")
    assert behavior["max_rotational_vel"] <= MAX_ANGULAR_VELOCITY
    assert behavior["rotational_acc_lim"] <= MAX_ANGULAR_ACCEL


@pytest.mark.parametrize("mode", MODES)
def test_mppi_acceleration_symmetric(mode: str) -> None:
    # MPPI clips each sampled control to [v + ax_min * dt, v + ax_max * dt] around the measured
    # speed v; with |ax_min| > ax_max the averaged command falls below v at every cycle and the
    # robot stays near zero speed.
    mppi = _params(_render(mode), "controller_server")["FollowPath"]
    assert mppi["ax_min"] == -mppi["ax_max"]
    assert mppi["ay_min"] == -mppi["ay_max"]


# Simulated LD19 points per scan, and AMCL position error measured in simulation after fast turns
# in place with the tuned parameters (docs/superpowers/specs/2026-11-25-nav2-tuning.md).
LIDAR_SAMPLES = 132
AMCL_POSITION_ERROR = 0.09


@pytest.mark.parametrize("mode", MODES)
def test_amcl_localization_settings(mode: str) -> None:
    params = _render(mode)
    amcl = _params(params, "amcl")
    assert amcl["max_beams"] == LIDAR_SAMPLES
    # Translation noise per radian of rotation: 0.2 spread the particles beyond what the laser
    # pulls back during turns in place.
    assert amcl["alpha4"] <= 0.05
    checker = _params(params, "controller_server")["general_goal_checker"]
    assert checker["xy_goal_tolerance"] > AMCL_POSITION_ERROR


@pytest.mark.parametrize("mode", MODES)
def test_velocity_smoother_open_loop(mode: str) -> None:
    # In closed loop the smoother output is the measured velocity plus a bounded change, never
    # clamped to max_velocity: a glitch of the odometry twist (gz-sim OdometryPublisher reports
    # about 62 rad/s when the yaw crosses -2*pi/3) reaches cmd_vel as is.
    assert _params(_render(mode), "velocity_smoother")["feedback"] == "OPEN_LOOP"


@pytest.mark.parametrize("mode", MODES)
def test_controller_and_smoother_agree(mode: str) -> None:
    params = _render(mode)
    mppi = _params(params, "controller_server")["FollowPath"]
    smoother = _params(params, "velocity_smoother")
    assert [mppi["vx_max"], mppi["vy_max"], mppi["wz_max"]] == smoother["max_velocity"]
    assert mppi["vx_min"] == smoother["min_velocity"][0]
    assert mppi["wz_max"] == _params(params, "behavior_server")["max_rotational_vel"]


@pytest.mark.parametrize("mode", MODES)
def test_override_applied(mode: str, tmp_path: Path) -> None:
    override = tmp_path / "slow.yaml"
    override.write_text(
        yaml.safe_dump({"controller_server": {"ros__parameters": {"FollowPath": {"vx_max": 0.3}}}})
    )
    params = _render(mode, override=str(override))
    mppi = _params(params, "controller_server")["FollowPath"]
    assert mppi["vx_max"] == pytest.approx(0.3)
    assert "motion_model" in mppi


def test_override_by_installed_name() -> None:
    params = _render("differential", override="holonomic.yaml")
    assert _params(params, "amcl")["robot_model_type"] == "nav2_amcl::OmniMotionModel"


def test_override_unknown_rejected() -> None:
    with pytest.raises(ValueError, match=r"does_not_exist\.yaml"):
        _render("differential", override="does_not_exist.yaml")


def test_unknown_mode_rejected() -> None:
    with pytest.raises(ValueError, match="ackermann"):
        _render("ackermann")


def _expand(entities: Iterable[LaunchDescriptionEntity], context: LaunchContext) -> list[Node]:
    """Visit launch entities like `ros2 launch` would, without starting any process."""
    nodes = []
    for entity in entities:
        if isinstance(entity, Node):
            entity._perform_substitutions(context)
            nodes.append(entity)
        else:
            nodes.extend(_expand(entity.visit(context) or [], context))
    return nodes


class _LaunchParamsLoader(yaml.SafeLoader):
    """Safe loader that also reads the tuples launch_ros writes in parameter files."""


_LaunchParamsLoader.add_constructor(
    "tag:yaml.org,2002:python/tuple",
    lambda loader, node: loader.construct_sequence(node),
)


def _param_files(node: Node, context: LaunchContext) -> list[dict]:
    """Return the content of every parameter file passed to a node."""
    context.extend_locals({"ros_specific_arguments": {"name": "", "ns": ""}})
    args = [perform_substitutions(context, part) for part in node.cmd]
    contents = []
    for flag, value in itertools.pairwise(args):
        if flag == "--params-file":
            loader = _LaunchParamsLoader(Path(value).read_text())
            try:
                contents.append(loader.get_single_data())
            finally:
                loader.dispose()
    return contents


def _dict_parameters(node: Node, context: LaunchContext) -> dict:
    """Return the parameters a node receives through generated parameter files."""
    params: dict = {}
    for content in _param_files(node, context):
        for rules in content.values():
            params.update(rules.get("ros__parameters", {}))
    return params


def test_localization_actions(tmp_path: Path) -> None:
    robots = [
        (namespace, render_params_file("differential", namespace, 0.0, 0.0, 0.0))
        for namespace in ("minipock_0", "minipock_1")
    ]
    map_yaml = str(tmp_path / "map.yaml")
    context = LaunchContext()
    nodes = _expand(localization_actions(robots, map_yaml, use_sim_time=True), context)

    map_servers = [node for node in nodes if node.node_executable == "map_server"]
    assert [node.node_name for node in map_servers] == ["/map_server"]
    map_params = _dict_parameters(map_servers[0], context)
    assert map_params["yaml_filename"] == map_yaml
    assert map_params["topic_name"] == "/map"
    assert map_params["frame_id"] == "map"

    amcls = [node for node in nodes if node.node_executable == "amcl"]
    assert [node.node_name for node in amcls] == ["/minipock_0/amcl", "/minipock_1/amcl"]
    assert [node.expanded_node_namespace for node in amcls] == ["/minipock_0", "/minipock_1"]
    for amcl, namespace in zip(amcls, ("minipock_0", "minipock_1"), strict=True):
        assert ("/tf", "/tf") in amcl.expanded_remapping_rules
        assert ("/tf_static", "/tf_static") in amcl.expanded_remapping_rules
        nested = [
            content[namespace]["amcl"]["ros__parameters"]
            for content in _param_files(amcl, context)
            if namespace in content
        ]
        assert len(nested) == 1
        assert nested[0]["map_topic"] == "/map"
        assert nested[0]["base_frame_id"] == f"{namespace}/base_footprint"
        assert nested[0]["odom_frame_id"] == f"{namespace}/odom"

    managers = {
        node.node_name: _dict_parameters(node, context)
        for node in nodes
        if node.node_executable == "lifecycle_manager"
    }
    assert sorted(managers) == [
        "/lifecycle_manager_map",
        "/minipock_0/lifecycle_manager_localization",
        "/minipock_1/lifecycle_manager_localization",
    ]
    assert managers["/lifecycle_manager_map"]["node_names"] == ["map_server"]
    assert managers["/minipock_0/lifecycle_manager_localization"]["node_names"] == ["amcl"]
    assert managers["/minipock_1/lifecycle_manager_localization"]["node_names"] == ["amcl"]
    for params in managers.values():
        assert params["autostart"] is True
        assert "bond_timeout" not in params
    assert all(_dict_parameters(node, context)["use_sim_time"] is True for node in nodes)

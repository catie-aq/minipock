"""Tests for the fleet configuration loader."""

import math
from pathlib import Path

import pytest
import yaml

from minipock_bringup.config import (
    ConfigError,
    default_config_path,
    load_config,
    resolve_config_path,
)

DATA = Path(__file__).parent / "data"


def _write(tmp_path: Path, **overrides: object) -> str:
    """Write a valid fleet file, with top-level keys replaced by `overrides`."""
    content: dict = yaml.safe_load((DATA / "valid.yaml").read_text())
    content.update(overrides)
    path = tmp_path / "fleet.yaml"
    path.write_text(yaml.safe_dump(content))
    return str(path)


def _fleet(**robot: object) -> dict:
    return {"fleet": {"7": {"mode": "differential", "position": [0.0, 0.0], **robot}}}


def test_load_default_file() -> None:
    assert Path(default_config_path()).parent.parts[-2:] == ("minipock_bringup", "config")
    config = load_config(default_config_path())
    assert len(config.robots) == 2
    assert config.robots[0].name == "minipock_0"
    assert config.world == "empty_room_world"
    assert config.map == "empty_room_map"


def test_position_with_yaw(tmp_path: Path) -> None:
    config = load_config(_write(tmp_path, **_fleet(position=[1.0, 2.0, 1.57])))
    assert config.robots[0].yaw == 1.57
    config = load_config(_write(tmp_path, **_fleet(position=[1.0, 2.0])))
    assert config.robots[0].yaw == 0.0
    assert (config.robots[0].x, config.robots[0].y) == (1.0, 2.0)


def test_missing_mode_rejected(tmp_path: Path) -> None:
    path = _write(tmp_path, fleet={"7": {"position": [0.0, 0.0]}})
    with pytest.raises(ConfigError, match="7"):
        load_config(path)


def test_unknown_mode_rejected(tmp_path: Path) -> None:
    with pytest.raises(ConfigError, match="7"):
        load_config(_write(tmp_path, **_fleet(mode="flying")))


def test_position_wrong_length_rejected(tmp_path: Path) -> None:
    with pytest.raises(ConfigError, match="7"):
        load_config(_write(tmp_path, **_fleet(position=[1.0])))


@pytest.mark.parametrize("namespace", ["minipock/", "Minipock_"])
def test_namespace_pattern_rejected(tmp_path: Path, namespace: str) -> None:
    with pytest.raises(ConfigError, match="namespace"):
        load_config(_write(tmp_path, namespace=namespace))


@pytest.mark.parametrize("field", ["bringup", "use_sim_time"])
def test_removed_fields_rejected(tmp_path: Path, field: str) -> None:
    with pytest.raises(ConfigError, match="entry launch"):
        load_config(_write(tmp_path, **{field: False}))


def test_resolve_installed_name() -> None:
    path = resolve_config_path("minipocks_real.yaml")
    assert Path(path).exists()
    assert "minipock_bringup" in Path(path).resolve().parts
    assert load_config(path).map == "catie"


def test_resolve_existing_path_unchanged() -> None:
    path = str(DATA / "valid.yaml")
    assert resolve_config_path(path) == path


def test_resolve_unknown_name() -> None:
    with pytest.raises(ConfigError, match=r"minipocks\.yaml") as error:
        resolve_config_path("nope.yaml")
    assert "minipock_bringup" in str(error.value)


def test_minipock_is_pure_metapackage() -> None:
    cmake = (Path(__file__).parents[2] / "minipock" / "CMakeLists.txt").read_text()
    assert ".yaml" not in cmake


def test_spiral_for_null_positions(tmp_path: Path) -> None:
    fleet = {str(i): {"mode": "holonomic", "position": None} for i in range(3)}
    config = load_config(_write(tmp_path, fleet=fleet))
    positions = {(r.x, r.y) for r in config.robots}
    assert len(positions) == 3
    assert (config.robots[0].x, config.robots[0].y) == (0.0, 0.0)


def test_nav2_params_optional(tmp_path: Path) -> None:
    assert load_config(_write(tmp_path, **_fleet())).robots[0].nav2_params == ""
    config = load_config(_write(tmp_path, **_fleet(nav2_params="holonomic_slow.yaml")))
    assert config.robots[0].nav2_params == "holonomic_slow.yaml"


def test_spiral_avoids_explicit_positions(tmp_path: Path) -> None:
    fleet = {
        "0": {"mode": "holonomic", "position": [0.0, 0.0]},
        "1": {"mode": "holonomic", "position": None},
    }
    config = load_config(_write(tmp_path, fleet=fleet))
    first, second = config.robots
    assert math.hypot(second.x - first.x, second.y - first.y) >= 0.5


def test_unquoted_fleet_key_rejected(tmp_path: Path) -> None:
    path = tmp_path / "fleet.yaml"
    path.write_text(
        "namespace: minipock_\nmap: m\nworld: w\nrviz: r\n"
        "fleet:\n  0:\n    mode: differential\n    position: null\n"
    )
    with pytest.raises(ConfigError, match=r"quote"):
        load_config(str(path))

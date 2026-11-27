"""Fleet configuration loading and validation."""

import math
import os
from collections.abc import Iterator
from dataclasses import dataclass
from pathlib import Path

import jsonschema
import yaml
from ament_index_python.packages import get_package_share_directory

NAMESPACE_PATTERN = r"^[a-z][a-z0-9_]*$"
MODES = ("differential", "holonomic")
MIN_SPACING = 0.5
_REMOVED_FIELDS = ("bringup", "use_sim_time")
_POSITION_WITH_YAW = 3

_SCHEMA = {
    "type": "object",
    "additionalProperties": False,
    "required": ["namespace", "world", "map", "rviz", "fleet"],
    "properties": {
        "namespace": {"type": "string", "pattern": NAMESPACE_PATTERN},
        "world": {"type": "string"},
        "map": {"type": "string"},
        "rviz": {"type": "string"},
        "fleet": {
            "type": "object",
            "minProperties": 1,
            "additionalProperties": False,
            "patternProperties": {
                "^[a-z0-9]+$": {
                    "type": "object",
                    "additionalProperties": False,
                    "required": ["mode", "position"],
                    "properties": {
                        "mode": {"type": "string", "enum": list(MODES)},
                        "position": {
                            "oneOf": [
                                {"type": "null"},
                                {
                                    "type": "array",
                                    "items": {"type": "number"},
                                    "minItems": 2,
                                    "maxItems": 3,
                                },
                            ],
                        },
                        "nav2_params": {"type": "string"},
                    },
                },
            },
        },
    },
}


class ConfigError(ValueError):
    """Raised when the fleet configuration is missing or invalid."""


@dataclass(frozen=True)
class Robot:
    """One robot of the fleet.

    Attributes:
        name: Robot name and namespace, `<namespace><id>`.
        mode: Kinematics, `differential` or `holonomic`.
        x: Start x position in the map frame, in meters.
        y: Start y position in the map frame, in meters.
        yaw: Start heading in radians.
        nav2_params: Nav2 parameter file name or path, empty for the mode default.
    """

    name: str
    mode: str
    x: float
    y: float
    yaw: float
    nav2_params: str


@dataclass(frozen=True)
class FleetConfig:
    """Validated fleet configuration."""

    namespace: str
    world: str
    map: str
    rviz: str
    robots: tuple[Robot, ...]


def _config_dir() -> Path:
    return Path(get_package_share_directory("minipock_bringup")) / "config"


def _installed_file(package: str, folder: str, name: str, suffix: str, field: str) -> None:
    """Check that `<name><suffix>` exists in `folder` of `package`.

    Raises:
        ConfigError: If the file is missing; the message lists the installed names.
    """
    directory = Path(get_package_share_directory(package)) / folder
    if (directory / f"{name}{suffix}").is_file():
        return
    installed = ", ".join(sorted(path.stem for path in directory.glob(f"*{suffix}")))
    msg = (
        f"Fleet file field '{field}': '{name}' not found in {directory}. "
        f"Installed names: {installed}"
    )
    raise ConfigError(msg)


def _check_installed_files(data: dict) -> None:
    """Check that the world, the map and the RViz configuration of a fleet file exist."""
    _installed_file("minipock_gz", "worlds", data["world"], ".sdf", "world")
    _installed_file("minipock_navigation2", "map", data["map"], ".yaml", "map")
    _installed_file("minipock_navigation2", "rviz", data["rviz"], ".rviz", "rviz")


def _resolve_nav2_params(value: str, fleet_dir: Path) -> str:
    """Anchor a relative Nav2 override on the folder of the fleet file.

    A bare file name stays as it is when no such file sits next to the fleet file: it then names
    a file of `minipock_navigation2/param`. A relative path with a folder is always taken from
    the fleet file folder; absolute paths are unchanged.
    """
    if not value or Path(value).is_absolute():
        return value
    if "/" in value or (fleet_dir / value).is_file():
        return str(fleet_dir / value)
    return value


def default_config_path() -> str:
    """Return the path of the default fleet file installed by `minipock_bringup`."""
    return str(_config_dir() / "minipocks.yaml")


def resolve_config_path(value: str) -> str:
    """Resolve a fleet file given as a path or as an installed file name.

    Args:
        value: An existing file path, or a file name installed in `minipock_bringup`.

    Returns:
        The path to load.

    Raises:
        ConfigError: If `value` is neither an existing path nor an installed file name.
    """
    if Path(value).is_file():
        return value
    share = _config_dir()
    candidate = share / value
    if "/" not in value and candidate.is_file():
        return str(candidate)
    installed = ", ".join(sorted(path.name for path in share.glob("*.yaml")))
    msg = f"Fleet file '{value}' not found. Installed files in {share}: {installed}"
    raise ConfigError(msg)


def _spiral_candidates(spacing: float = 1.0) -> Iterator[tuple[float, float]]:
    """Yield spiral positions: the origin, then rings of 8 positions."""
    yield (0.0, 0.0)
    ring = 1
    while True:
        for k in range(8):
            angle = k * (2 * math.pi / 8)
            yield (spacing * ring * math.cos(angle), spacing * ring * math.sin(angle))
        ring += 1


def _spiral_positions(count: int, taken: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Return `count` spiral positions at least MIN_SPACING from every taken position."""
    positions: list[tuple[float, float]] = []
    for x, y in _spiral_candidates():
        if len(positions) == count:
            break
        if all(math.hypot(x - tx, y - ty) >= MIN_SPACING for tx, ty in taken):
            positions.append((x, y))
    return positions


def _validate(data: object) -> None:
    if not isinstance(data, dict):
        msg = "The fleet file must contain a mapping"
        raise ConfigError(msg)
    removed = [field for field in _REMOVED_FIELDS if field in data]
    if removed:
        msg = f"Field(s) {', '.join(removed)} are removed: they are set by the entry launch"
        raise ConfigError(msg)
    fleet = data.get("fleet")
    if isinstance(fleet, dict):
        for key in fleet:
            if not isinstance(key, str):
                msg = f'Fleet id {key!r} is not a string: quote it in the YAML (e.g. "{key}":)'
                raise ConfigError(msg)
    validator = jsonschema.Draft202012Validator(_SCHEMA)
    errors = sorted(validator.iter_errors(data), key=lambda error: list(error.path))
    if errors:
        error = errors[0]
        where = ".".join(str(part) for part in error.path) or "<root>"
        message = error.message
        if len(error.path) > 1 and error.path[0] == "fleet":
            field = ".".join(str(part) for part in list(error.path)[2:])
            where = f"robot {error.path[1]} {field}".strip()
            if list(error.path)[2:3] == ["position"]:
                message = (
                    f"expected null (placed automatically), [x, y] or [x, y, yaw] in meters "
                    f"and radians, got {error.instance!r}"
                )
        msg = f"Invalid fleet file at {where}: {message}"
        raise ConfigError(msg)


def load_config(path: str) -> FleetConfig:
    """Load and validate a fleet file.

    Robots with `position: null` are placed on a spiral, away from explicit positions.

    Args:
        path: Path of the fleet YAML file.

    Returns:
        The validated configuration.

    Raises:
        ConfigError: If the file cannot be read, is not YAML or is not valid, or if its world,
            map or RViz configuration is not installed.

    A relative `nav2_params` path is resolved from the folder of the fleet file.
    """
    try:
        with Path(path).open() as file:
            data = yaml.safe_load(file)
    except OSError as error:
        msg = f"Cannot read the fleet file {path}: {error.strerror or error}"
        raise ConfigError(msg) from error
    except yaml.YAMLError as error:
        msg = f"The fleet file {path} is not valid YAML: {error}"
        raise ConfigError(msg) from error
    _validate(data)
    _check_installed_files(data)
    fleet_dir = Path(os.path.abspath(path)).parent  # noqa: PTH100 - keeps symlinks unresolved
    fleet = data["fleet"]
    explicit = [
        (float(spec["position"][0]), float(spec["position"][1]))
        for spec in fleet.values()
        if spec["position"] is not None
    ]
    free_ids = [robot_id for robot_id, spec in fleet.items() if spec["position"] is None]
    spiral = dict(zip(free_ids, _spiral_positions(len(free_ids), explicit), strict=True))
    robots = []
    for robot_id, spec in fleet.items():
        position = spec["position"]
        if position is None:
            x, y = spiral[robot_id]
            yaw = 0.0
        else:
            x, y = float(position[0]), float(position[1])
            yaw = float(position[2]) if len(position) == _POSITION_WITH_YAW else 0.0
        robots.append(
            Robot(
                name=f"{data['namespace']}{robot_id}",
                mode=spec["mode"],
                x=x,
                y=y,
                yaw=yaw,
                nav2_params=_resolve_nav2_params(spec.get("nav2_params", ""), fleet_dir),
            ),
        )
    return FleetConfig(
        namespace=data["namespace"],
        world=data["world"],
        map=data["map"],
        rviz=data["rviz"],
        robots=tuple(robots),
    )

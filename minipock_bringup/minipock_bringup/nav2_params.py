"""Nav2 parameters of one robot: common file, kinematics overlay and optional override."""

import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory

from minipock_bringup.config import MODES

NAMESPACE_MARKER = "<robot_namespace>"
_POSE_MARKERS = ("<x>", "<y>", "<yaw>")


def _param_dir() -> Path:
    return Path(get_package_share_directory("minipock_navigation2")) / "param"


def merge_params(base: dict, overlay: dict) -> dict:
    """Merge two parameter dictionaries recursively.

    Args:
        base: Parameters to start from.
        overlay: Parameters that win over `base`. Lists replace the base list as a whole.

    Returns:
        A new dictionary; the inputs are not modified.

    Raises:
        ValueError: If a value of `overlay` is empty (`None`), which ROS cannot load.
    """
    return _merge(base, overlay, ())


def _merge(base: dict, overlay: dict, path: tuple[str, ...]) -> dict:
    merged = dict(base)
    for key, value in overlay.items():
        where = (*path, str(key))
        if value is None:
            msg = f"Parameter {'.'.join(where)} has no value: give one or remove the key"
            raise ValueError(msg)
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _merge(merged[key], value, where)
        else:
            merged[key] = value
    return merged


def _resolve_override(value: str) -> Path:
    """Resolve an override given as a path or as a file name of `minipock_navigation2/param`."""
    if Path(value).is_file():
        return Path(value)
    share = _param_dir()
    candidate = share / value
    if "/" not in value and candidate.is_file():
        return candidate
    installed = ", ".join(sorted(path.name for path in share.glob("*.yaml")))
    msg = f"Nav2 parameter file '{value}' not found. Installed files in {share}: {installed}"
    raise ValueError(msg)


def _load(path: Path) -> dict:
    with path.open() as file:
        return yaml.safe_load(file) or {}


def _substitute(value: object, replacements: dict[str, object]) -> object:
    """Replace the markers in every string of a parameter tree.

    A string equal to a marker takes the replacement value with its type (a float for the
    pose); a marker inside a longer string is replaced by its text.
    """
    if isinstance(value, dict):
        return {key: _substitute(item, replacements) for key, item in value.items()}
    if isinstance(value, list):
        return [_substitute(item, replacements) for item in value]
    if isinstance(value, str):
        if value in replacements:
            return replacements[value]
        for marker, replacement in replacements.items():
            value = value.replace(marker, str(replacement))
    return value


def render_params_file(  # noqa: PLR0913 - kinematics, namespace, x, y, yaw and override are independent inputs
    mode: str,
    namespace: str,
    x: float,
    y: float,
    yaw: float,
    *,
    override: str = "",
) -> str:
    """Write the Nav2 parameters of one robot to a temporary file.

    Loads `param/common.yaml` of `minipock_navigation2`, merges `param/<mode>.yaml`, then
    `override` if given, and replaces the `<robot_namespace>`, `<x>`, `<y>` and `<yaw>` markers.

    Args:
        mode: Kinematics, `differential` or `holonomic`.
        namespace: Robot namespace, without leading or trailing slash.
        x: Start x position in the map frame, in meters.
        y: Start y position in the map frame, in meters.
        yaw: Start heading in radians.
        override: Path or file name in `minipock_navigation2/param` of extra parameters,
            merged last. Empty for none.

    Returns:
        The path of the rendered parameter file.

    Raises:
        ValueError: If `mode` is unknown or `override` cannot be found.
    """
    if mode not in MODES:
        msg = f"Unknown kinematics '{mode}', expected one of {', '.join(MODES)}"
        raise ValueError(msg)
    share = _param_dir()
    params = merge_params(_load(share / "common.yaml"), _load(share / f"{mode}.yaml"))
    if override:
        params = merge_params(params, _load(_resolve_override(override)))
    replacements: dict[str, object] = {NAMESPACE_MARKER: namespace}
    replacements.update(zip(_POSE_MARKERS, (float(x), float(y), float(yaw)), strict=True))
    params = _substitute(params, replacements)
    with tempfile.NamedTemporaryFile(
        mode="w", prefix=f"nav2_{namespace}_", suffix=".yaml", delete=False
    ) as file:
        yaml.safe_dump(params, file, sort_keys=False)
    return file.name

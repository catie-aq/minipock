"""Tests of the office world: offline assets, licenses and the generated Nav2 map."""

import math
import re
from pathlib import Path

import pytest
import yaml
from lxml import etree

PACKAGE = Path(__file__).resolve().parents[1]
REPO_ROOT = PACKAGE.parent
WORLD = PACKAGE / "worlds" / "office.sdf"
MODELS = PACKAGE / "models"
ATTRIBUTION = PACKAGE / "worlds" / "ATTRIBUTION.md"
MAP_DIR = REPO_ROOT / "minipock_navigation" / "minipock_navigation2" / "map"
MAP_YAML = MAP_DIR / "office.yaml"
FLEET = REPO_ROOT / "minipock_bringup" / "config" / "minipocks.yaml"

FREE = 254
SPAWN_RADIUS = 0.25
MODEL_URI = re.compile(r"model://([^/<\s]+)")


def _sdf_files() -> list[Path]:
    return [WORLD, *sorted(MODELS.glob("**/*.sdf"))]


def _read_pgm(path: Path) -> tuple[int, int, bytes]:
    """Return the width, height and pixels of a binary (P5) 8-bit PGM file."""
    data = path.read_bytes()
    tokens: list[bytes] = []
    index = 0
    while len(tokens) < 4:
        while data[index : index + 1].isspace():
            index += 1
        if data[index : index + 1] == b"#":
            index = data.index(b"\n", index) + 1
            continue
        end = index
        while not data[end : end + 1].isspace():
            end += 1
        tokens.append(data[index:end])
        index = end
    assert tokens[0] == b"P5"
    width, height, max_value = (int(token) for token in tokens[1:])
    assert max_value == 255
    pixels = data[index + 1 : index + 1 + width * height]
    assert len(pixels) == width * height
    return width, height, pixels


def _map() -> tuple[dict, int, int, bytes]:
    meta = yaml.safe_load(MAP_YAML.read_text())
    width, height, pixels = _read_pgm(MAP_DIR / meta["image"])
    return meta, width, height, pixels


def _model_poses() -> list[tuple[float, float]]:
    world = etree.parse(WORLD).getroot().find("world")
    poses = []
    for model in world.findall("model"):
        values = [float(value) for value in (model.findtext("pose") or "0 0 0").split()]
        poses.append((values[0], values[1]))
    return poses


def test_world_has_no_network_uri() -> None:
    for path in _sdf_files():
        assert "http" not in path.read_text(), path


def test_every_model_uri_resolves() -> None:
    names = set()
    for path in _sdf_files():
        names.update(MODEL_URI.findall(path.read_text()))
    assert names
    for name in names:
        assert (MODELS / name / "model.config").is_file(), name


def test_every_model_has_license() -> None:
    attribution = ATTRIBUTION.read_text()
    rows = {
        cells[0].strip("` "): cells[1:]
        for cells in (
            [cell.strip() for cell in line.strip().strip("|").split("|")]
            for line in attribution.splitlines()
            if line.startswith("|")
        )
    }
    models = sorted(path.name for path in MODELS.iterdir() if path.is_dir())
    assert models
    for name in models:
        assert name in rows, name
        license_cell, source = rows[name][0], rows[name][1]
        # License column checked against the source: husarion_gz_worlds is Apache-2.0, each
        # Fuel model carries its own license (checked on its Fuel page).
        assert license_cell in {"Apache-2.0", "CC-BY-4.0", "CC0-1.0"}, name
        if "husarion_gz_worlds" in source:
            assert license_cell == "Apache-2.0", name
        else:
            assert f"app.gazebosim.org/OpenRobotics/fuel/models/{name})" in source, name


def test_world_name() -> None:
    assert etree.parse(WORLD).getroot().find("world").get("name") == "office"


def test_map_matches_world() -> None:
    meta, width, height, _ = _map()
    assert meta["resolution"] == pytest.approx(0.05)
    assert meta["mode"] == "trinary"
    origin_x, origin_y, _ = meta["origin"]
    max_x = origin_x + width * meta["resolution"]
    max_y = origin_y + height * meta["resolution"]
    poses = _model_poses()
    for x, y in poses:
        assert origin_x <= x <= max_x
        assert origin_y <= y <= max_y
    # The map hugs the building: no more than a few meters beyond the furthest model.
    margin = 5.0
    assert origin_x >= min(x for x, _ in poses) - margin
    assert origin_y >= min(y for _, y in poses) - margin
    assert max_x <= max(x for x, _ in poses) + margin
    assert max_y <= max(y for _, y in poses) + margin


def test_default_spawns_are_free() -> None:
    fleet = yaml.safe_load(FLEET.read_text())
    assert fleet["world"] == "office"
    assert fleet["map"] == "office"
    meta, width, height, pixels = _map()
    resolution = meta["resolution"]
    origin_x, origin_y, _ = meta["origin"]
    cells = math.ceil(SPAWN_RADIUS / resolution)
    for robot in fleet["fleet"].values():
        x, y = robot["position"][:2]
        column = math.floor((x - origin_x) / resolution)
        row = height - 1 - math.floor((y - origin_y) / resolution)
        for d_row in range(-cells, cells + 1):
            for d_column in range(-cells, cells + 1):
                if math.hypot(d_row, d_column) * resolution > SPAWN_RADIUS:
                    continue
                r, c = row + d_row, column + d_column
                assert 0 <= r < height
                assert 0 <= c < width
                assert pixels[r * width + c] == FREE, (robot, r, c)


# LiDAR band of the map (CLAUDE.md, world_to_map.py --z-min and --z-max).
BAND = (0.02, 0.1975)
OCCUPIED = 0


def _pose(element: etree._Element) -> tuple[float, ...]:
    text = element.findtext("pose") or "0 0 0 0 0 0"
    return tuple(float(value) for value in text.split())


def _compose(parent: tuple[float, ...], child: tuple[float, ...]) -> tuple[float, ...] | None:
    """Compose two planar poses (x, y, z, roll, pitch, yaw); None if one is tilted."""
    if any(abs(angle) > 1e-6 for angle in (*parent[3:5], *child[3:5])):
        return None
    cos, sin = math.cos(parent[5]), math.sin(parent[5])
    return (
        parent[0] + cos * child[0] - sin * child[1],
        parent[1] + sin * child[0] + cos * child[1],
        parent[2] + child[2],
        0.0,
        0.0,
        parent[5] + child[5],
    )


def _primitives_in_band() -> list[tuple[str, float, float]]:
    """Return the name and world (x, y) center of every box or cylinder cut by the band."""
    world = etree.parse(WORLD).getroot().find("world")
    centers = []
    for model in world.findall("model"):
        for link in model.findall("link"):
            for collision in link.findall("collision"):
                geometry = collision.find("geometry")
                box, cylinder = geometry.find("box"), geometry.find("cylinder")
                if box is not None:
                    height = float(box.findtext("size").split()[2])
                elif cylinder is not None:
                    height = float(cylinder.findtext("length"))
                else:
                    continue
                pose = _compose(_pose(model), _pose(link))
                pose = pose and _compose(pose, _pose(collision))
                if pose is None:
                    continue
                if pose[2] + height / 2 < BAND[0] or pose[2] - height / 2 > BAND[1]:
                    continue
                name = f"{model.get('name')}::{collision.get('name')}"
                centers.append((name, pose[0], pose[1]))
    return centers


def test_map_cells_match_world_obstacles() -> None:
    # A sample of the map recomputed from the world: the cell under the center of each box and
    # cylinder cut by the LiDAR band is occupied. Moving one of them without regenerating the
    # map (CLAUDE.md, world_to_map.py) fails here.
    meta, width, height, pixels = _map()
    resolution = meta["resolution"]
    origin_x, origin_y, _ = meta["origin"]
    centers = _primitives_in_band()
    assert len(centers) >= 20
    for name, x, y in centers:
        column = math.floor((x - origin_x) / resolution)
        row = height - 1 - math.floor((y - origin_y) / resolution)
        assert pixels[row * width + column] == OCCUPIED, name


def test_no_classic_gazebo_material_script() -> None:
    # Gazebo Sim ignores `<script>` materials and the `file://media` files do not exist.
    for path in _sdf_files():
        assert "file://media" not in path.read_text(), path

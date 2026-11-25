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
        assert any(re.search(r"Apache-2\.0|CC-BY-4\.0|CC0-1\.0", cell) for cell in rows[name])


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

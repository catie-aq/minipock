"""Tests of the rasterization of `scripts/world_to_map.py` (map cells aligned with the world)."""

import importlib.util
import sys
import types
from pathlib import Path

import numpy as np
import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "world_to_map.py"
RESOLUTION = 0.05
OCCUPIED = 0


@pytest.fixture(scope="module")
def world_to_map() -> types.ModuleType:
    """Load the script with stubs for the geometry libraries the Grid does not use.

    trimesh and shapely are not in the ROS image; the script runs with `uv run` on the host.
    """
    stubs = {
        "trimesh": types.ModuleType("trimesh"),
        "shapely": types.ModuleType("shapely"),
        "shapely.geometry": types.ModuleType("shapely.geometry"),
    }
    stubs["trimesh"].Trimesh = type("Trimesh", (), {})
    stubs["shapely.geometry"].Polygon = type("Polygon", (), {})
    saved = {name: sys.modules.get(name) for name in stubs}
    sys.modules.update(stubs)
    try:
        spec = importlib.util.spec_from_file_location("world_to_map", SCRIPT)
        assert spec is not None
        assert spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    finally:
        for name, previous in saved.items():
            if previous is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = previous
    return module


def _occupied_cells(world_to_map: types.ModuleType, x: tuple, y: tuple) -> tuple[list, list]:
    """Rasterize the rectangle [x0, x1] x [y0, y1] and return its occupied columns and rows.

    Rows are counted from the lowest y, like the map origin.
    """
    grid = world_to_map.Grid(np.array([[0.0, 0.0], [2.0, 2.0]]), RESOLUTION, margin=0.0)
    corners = np.array([[x[i], y[j], 0.1] for i, j in ((0, 0), (1, 0), (1, 1), (0, 1))])
    grid.draw(grid.obstacles, np.array([corners[[0, 1, 2]], corners[[0, 2, 3]]]))
    occupied = np.flipud(grid.pixels()) == OCCUPIED
    columns = np.flatnonzero(occupied.any(axis=0)).tolist()
    rows = np.flatnonzero(occupied.any(axis=1)).tolist()
    return columns, rows


def test_obstacle_cells_match_world(world_to_map: types.ModuleType) -> None:
    # x in [0.51, 0.74] lies in cells 10 to 14 (cell i covers [0.05 i, 0.05 (i + 1))); a half-cell
    # shift would also fill cell 9 and move the obstacle towards -x.
    columns, rows = _occupied_cells(world_to_map, (0.51, 0.74), (1.02, 1.24))
    assert columns == list(range(10, 15))
    assert rows == list(range(20, 25))


def test_thin_obstacle_kept(world_to_map: types.ModuleType) -> None:
    # A 1 cm glass wall inside one cell still occupies it.
    columns, _ = _occupied_cells(world_to_map, (0.611, 0.621), (0.5, 1.0))
    assert columns == [12]

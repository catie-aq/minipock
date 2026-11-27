"""Tests of the URIs of every world: they resolve through GZ_SIM_RESOURCE_PATH."""

import re
from pathlib import Path

import pytest

PACKAGE = Path(__file__).resolve().parents[1]
WORLDS = sorted((PACKAGE / "worlds").glob("*.sdf"))
URI = re.compile(r"<uri>\s*([^<\s]+)\s*</uri>")
PACKAGE_PREFIX = "model://minipock_gz/"


@pytest.mark.parametrize("world", WORLDS, ids=[path.name for path in WORLDS])
def test_no_absolute_path(world: Path) -> None:
    # An absolute path (for example the container install) does not load elsewhere.
    for uri in URI.findall(world.read_text()):
        assert not uri.startswith(("/", "file:///")), uri


def test_empty_room_ground_resolves() -> None:
    # world.launch.py puts the share directory holding minipock_gz in GZ_SIM_RESOURCE_PATH.
    uris = URI.findall((PACKAGE / "worlds" / "empty_room_world.sdf").read_text())
    ground = [uri for uri in uris if uri.endswith("empty_room_ground")]
    assert ground == [f"{PACKAGE_PREFIX}worlds/models/empty_room_ground"]
    assert (PACKAGE / ground[0].removeprefix(PACKAGE_PREFIX) / "model.config").is_file()

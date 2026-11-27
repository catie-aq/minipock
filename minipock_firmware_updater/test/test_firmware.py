"""Tests of the firmware update logic (no ROS, no network)."""

import pytest
from jsonschema.exceptions import ValidationError

from minipock_firmware_updater import firmware


@pytest.mark.parametrize(
    ("tag", "version"),
    [("v0.0.0", (0, 0, 0)), ("v1.2.3", (1, 2, 3)), ("v10.20.300", (10, 20, 300))],
)
def test_parse_version(tag: str, version: tuple[int, int, int]) -> None:
    assert firmware.parse_version(tag) == version
    assert firmware.format_version(version) == tag


@pytest.mark.parametrize("tag", ["1.2.3", "v1.2", "v1.2.3-rc1", "v1.2.x", "latest", ""])
def test_parse_version_rejects_other_tags(tag: str) -> None:
    with pytest.raises(ValueError, match="v<major>"):
        firmware.parse_version(tag)


def test_format_version_drops_leading_zeros() -> None:
    assert firmware.format_version(firmware.parse_version("v01.002.3")) == "v1.2.3"


def test_select_latest_is_first_tag() -> None:
    assert firmware.select_version("latest", ["v2.0.0", "v1.0.0"]) == "v2.0.0"
    assert firmware.select_version("latest", []) is None


def test_select_explicit_tag() -> None:
    assert firmware.select_version("v1.0.0", ["v2.0.0", "v1.0.0"]) == "v1.0.0"


@pytest.mark.parametrize(
    ("current", "target", "expected"),
    [
        ((1, 2, 3), (1, 2, 4), True),
        ((1, 2, 3), (1, 3, 0), True),
        ((1, 9, 9), (2, 0, 0), True),
        ((1, 2, 3), (1, 2, 3), False),
        ((1, 2, 4), (1, 2, 3), False),
        ((1, 10, 0), (1, 9, 0), False),
    ],
)
def test_needs_update(
    current: tuple[int, int, int], target: tuple[int, int, int], *, expected: bool
) -> None:
    assert firmware.needs_update(current, target) is expected


def test_find_bin_url() -> None:
    release = {
        "assets": [
            {"name": "notes.txt", "browser_download_url": "https://example.com/notes.txt"},
            {"name": "zephyr.bin", "browser_download_url": "https://example.com/zephyr.bin"},
            {"name": "other.bin", "browser_download_url": "https://example.com/other.bin"},
        ]
    }
    assert firmware.find_bin_url(release) == "https://example.com/zephyr.bin"


def test_find_bin_url_without_bin() -> None:
    assert firmware.find_bin_url({"assets": [{"name": "a.hex"}]}) is None
    assert firmware.find_bin_url({}) is None


def test_stored_entry_validates() -> None:
    stored = {"v1.0.0": firmware.stored_entry(b"\x00\x01\xff")}
    assert stored["v1.0.0"] == {"size": 3, "content": "0001ff"}
    firmware.validate_stored(stored)


@pytest.mark.parametrize(
    "stored",
    [
        {"1.0.0": {"size": 1, "content": "00"}},
        {"v1.0.0": {"size": "1", "content": "00"}},
        {"v1.0.0": {"content": "00"}},
    ],
)
def test_validate_stored_rejects(stored: dict) -> None:
    with pytest.raises(ValidationError):
        firmware.validate_stored(stored)


IMAGE = bytes(range(10))


@pytest.mark.parametrize(
    ("chunk_id", "chunk_size", "expected"),
    [
        (0, 4, IMAGE[0:4]),
        (1, 4, IMAGE[4:8]),
        (2, 4, IMAGE[8:10]),
        (3, 4, b""),
        (0, 10, IMAGE),
        (0, 64, IMAGE),
        (1, 10, b""),
        (-1, 4, b""),
        (0, 0, b""),
    ],
)
def test_chunk(chunk_id: int, chunk_size: int, expected: bytes) -> None:
    assert firmware.chunk(firmware.stored_entry(IMAGE), chunk_id, chunk_size) == expected


def test_chunks_cover_the_image() -> None:
    entry = firmware.stored_entry(bytes(range(256)) * 3)
    chunks = []
    chunk_id = 0
    while data := firmware.chunk(entry, chunk_id, 100):
        chunks.append(data)
        chunk_id += 1
    assert b"".join(chunks) == bytes(range(256)) * 3
    assert len(chunks) == 8


# Reference values computed with `crc.Calculator(crc.Crc8.CCITT)` (crc 7.1.0), the library the
# node used before; 0xF4 is also the published check value of this CRC-8 for "123456789".
@pytest.mark.parametrize(
    ("data", "expected"),
    [
        (b"", 0x00),
        (b"123456789", 0xF4),
        (bytes(range(256)), 0x14),
        (b"\xff" * 7, 0x0C),
    ],
)
def test_crc8(data: bytes, expected: int) -> None:
    assert firmware.crc8(data) == expected


def test_crc8_detects_a_flipped_bit() -> None:
    data = bytearray(b"minipock firmware chunk")
    reference = firmware.crc8(bytes(data))
    data[5] ^= 0x10
    assert firmware.crc8(bytes(data)) != reference

"""Firmware update logic without ROS or network: versions, release assets, chunks, checksums."""

import re

import jsonschema

VERSION_PATTERN = r"^v[0-9]+\.[0-9]+\.[0-9]+$"
STORED_VERSION_SCHEMA = {
    "type": "object",
    "patternProperties": {
        VERSION_PATTERN: {
            "type": "object",
            "properties": {
                "size": {"type": "integer"},
                "content": {"type": "string"},
            },
            "required": ["size", "content"],
        }
    },
    "additionalProperties": False,
}
LATEST = "latest"
# CRC-8 checked by the firmware on every chunk: polynomial 0x07, initial value 0, no reflection,
# no final XOR (the `Crc8.CCITT` configuration of the `crc` Python package used before).
CRC8_POLYNOMIAL = 0x07

Version = tuple[int, int, int]


def parse_version(tag: str) -> Version:
    """Return the `(major, minor, patch)` of a release tag.

    Args:
        tag: Release tag, `v<major>.<minor>.<patch>`.

    Returns:
        The three version numbers.

    Raises:
        ValueError: If `tag` does not follow `v<major>.<minor>.<patch>`.
    """
    if not re.fullmatch(VERSION_PATTERN, tag):
        msg = f"Release tag '{tag}' does not follow v<major>.<minor>.<patch>"
        raise ValueError(msg)
    major, minor, patch = (int(part) for part in tag[1:].split("."))
    return (major, minor, patch)


def format_version(version: Version) -> str:
    """Return the release tag of a `(major, minor, patch)` version."""
    return "v{}.{}.{}".format(*version)


def select_version(method: str, tags: list[str]) -> str | None:
    """Return the release tag an update method asks for.

    Args:
        method: `latest`, or a release tag.
        tags: Release tags, newest first (order of the GitHub releases API).

    Returns:
        The newest tag for `latest` (`None` without any release), `method` otherwise.
    """
    if method == LATEST:
        return tags[0] if tags else None
    return method


def needs_update(current: Version, target: Version) -> bool:
    """Return whether the board running `current` must be updated to `target`."""
    return current < target


def find_bin_url(release: dict) -> str | None:
    """Return the download URL of the first `.bin` asset of a GitHub release, if any."""
    for asset in release.get("assets", []):
        if asset.get("name", "").endswith(".bin"):
            return asset["browser_download_url"]
    return None


def stored_entry(content: bytes) -> dict:
    """Return the stored form of a firmware image: its size in bytes and its hex content."""
    return {"size": len(content), "content": content.hex()}


def validate_stored(stored: dict) -> None:
    """Check the stored firmware images against `STORED_VERSION_SCHEMA`.

    Raises:
        jsonschema.exceptions.ValidationError: If a key is not a release tag or an entry is
            malformed.
    """
    jsonschema.validate(instance=stored, schema=STORED_VERSION_SCHEMA)


def chunk(entry: dict, chunk_id: int, chunk_size: int) -> bytes:
    """Return one chunk of a stored firmware image.

    Args:
        entry: Stored image, as returned by `stored_entry`.
        chunk_id: Index of the chunk, from 0.
        chunk_size: Chunk size in bytes; the last chunk may be shorter.

    Returns:
        The bytes of the chunk, empty past the end of the image or for a negative index or
        a non-positive size.
    """
    if chunk_id < 0 or chunk_size <= 0:
        return b""
    start = chunk_id * chunk_size
    end = min(start + chunk_size, entry["size"])
    if start >= end:
        return b""
    return bytes.fromhex(entry["content"][2 * start : 2 * end])


def crc8(data: bytes) -> int:
    """Return the CRC-8 of `data` (polynomial 0x07, initial value 0, no reflection)."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ CRC8_POLYNOMIAL) if crc & 0x80 else (crc << 1)
            crc &= 0xFF
    return crc

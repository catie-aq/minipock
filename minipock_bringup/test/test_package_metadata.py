"""Check that package metadata is aligned across the repository."""

import re
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
# Packages of this repository, relative to its root. Only these directories are read: a
# workspace next to the sources (build/, install/, log/) or .git is never walked.
PACKAGE_DIRS = (
    "minipock",
    "minipock_bringup",
    "minipock_description",
    "minipock_firmware_updater",
    "minipock_gz",
    "minipock_navigation/minipock_cartographer",
    "minipock_navigation/minipock_navigation",
    "minipock_navigation/minipock_navigation2",
    "minipock_navigation/minipock_teleop",
)
# Open-RMF adapter (separate work) and the message submodule (own repository and version).
NOT_CHECKED = ("minipock_fleet_adapter", "minipock_msgs")
PACKAGE_LOCATIONS = ("*/package.xml", "minipock_navigation/*/package.xml")
EXPECTED_LICENSE = "Apache-2.0"
EXPECTED_EMAIL = "s.delpeuch@catie.fr"


def _package_xmls() -> list[Path]:
    return [REPO_ROOT / directory / "package.xml" for directory in PACKAGE_DIRS]


def _setup_pys() -> list[Path]:
    candidates = [path.with_name("setup.py") for path in _package_xmls()]
    return [path for path in candidates if path.exists()]


def _tag_values(path: Path, tag: str) -> list[str]:
    """Return the text of every ``<tag>`` element of a package.xml (flat, no nesting)."""
    return re.findall(rf"<{tag}(?:\s[^>]*)?>([^<]*)</{tag}>", path.read_text())


def _maintainer_emails(path: Path) -> list[str]:
    return re.findall(r'<maintainer\s+email="([^"]*)"', path.read_text())


def _ids(paths: list[Path]) -> list[str]:
    return [str(path.relative_to(REPO_ROOT)) for path in paths]


REFERENCE_VERSION = _tag_values(REPO_ROOT / "minipock" / "package.xml", "version")[0]


def test_packages_found() -> None:
    assert all(path.is_file() for path in _package_xmls())


def test_every_package_listed() -> None:
    found = {
        str(path.parent.relative_to(REPO_ROOT))
        for pattern in PACKAGE_LOCATIONS
        for path in REPO_ROOT.glob(pattern)
    }
    assert found - set(NOT_CHECKED) == set(PACKAGE_DIRS)


@pytest.mark.parametrize("path", _package_xmls(), ids=_ids(_package_xmls()))
def test_package_xml_metadata(path: Path) -> None:
    assert _tag_values(path, "version") == [REFERENCE_VERSION]
    assert _tag_values(path, "license") == [EXPECTED_LICENSE]
    emails = _maintainer_emails(path)
    assert emails
    assert set(emails) == {EXPECTED_EMAIL}


@pytest.mark.parametrize("path", _setup_pys(), ids=_ids(_setup_pys()))
def test_setup_py_metadata(path: Path) -> None:
    text = path.read_text()
    assert re.findall(r'\bversion="([^"]*)"', text) == [REFERENCE_VERSION]
    assert f'license="{EXPECTED_LICENSE}"' in text
    assert "python>=" not in text
    emails = re.findall(r'(?:maintainer|author)_email="([^"]+)"', text)
    assert emails
    assert set(emails) == {EXPECTED_EMAIL}

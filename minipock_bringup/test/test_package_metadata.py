"""Check that package metadata is aligned across the repository."""

import re
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
EXCLUDED = {"minipock_fleet_adapter", "minipock_msgs", "minipock_firmware"}
EXPECTED_LICENSE = "Apache-2.0"
EXPECTED_EMAIL = "s.delpeuch@catie.fr"


def _package_xmls() -> list[Path]:
    return sorted(
        path
        for path in REPO_ROOT.rglob("package.xml")
        if not EXCLUDED.intersection(path.relative_to(REPO_ROOT).parts)
        and "build" not in path.relative_to(REPO_ROOT).parts
    )


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
    names = {path.parent.name for path in _package_xmls()}
    assert {"minipock", "minipock_bringup", "minipock_gz", "minipock_teleop"} <= names


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
    assert f'version="{REFERENCE_VERSION}"' in text
    assert f'license="{EXPECTED_LICENSE}"' in text
    assert "python>=" not in text
    emails = re.findall(r'(?:maintainer|author)_email="([^"]+)"', text)
    assert emails
    assert set(emails) == {EXPECTED_EMAIL}

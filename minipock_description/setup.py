"""Packaging for minipock_description."""

from pathlib import Path

from setuptools import setup

package_name = "minipock_description"


def _data_dir(directory: str) -> tuple[str, list[str]]:
    """Return a data_files entry installing the files of `directory`.

    Args:
        directory: Directory relative to the package root.

    Returns:
        The install destination and the list of source files.
    """
    files = sorted(str(path) for path in Path(directory).glob("*.*"))
    return (f"share/{package_name}/{directory}", files)


setup(
    name=package_name,
    version="1.0.0",
    description="Description package for MiniPock robot",
    author="Sébastien Delpeuch",
    author_email="s.delpeuch@catie.fr",
    license="Apache 2.0",
    install_requires=["python>=3.10"],
    tests_require=["pytest"],
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        _data_dir("models"),
        _data_dir("urdf"),
        _data_dir("launch"),
    ],
)

"""Packaging for minipock_bringup."""

from pathlib import Path

from setuptools import find_packages, setup

package_name = "minipock_bringup"


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
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        _data_dir("launch"),
        _data_dir("config"),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sedelpeuch",
    maintainer_email="s.delpeuch@catie.fr",
    description="Bringup for MiniPock",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "raw_data_transformer = minipock_bringup.raw_data_transformer:main",
        ],
    },
)

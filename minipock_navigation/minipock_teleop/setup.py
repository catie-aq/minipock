"""Packaging for minipock_teleop."""

from setuptools import find_packages, setup

package_name = "minipock_teleop"

setup(
    name=package_name,
    version="2.2.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Sébastien Delpeuch",
    maintainer_email="s.delpeuch@catie.fr",
    description="Teleoperation for MiniPock bot",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "teleop_keyboard = minipock_teleop.teleop_keyboard:main",
            "teleop_fps = minipock_teleop.teleop_fps:main",
        ],
    },
)

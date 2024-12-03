import os
from glob import glob

from setuptools import setup

package_name = "minipock_fleet_adapter"

setup(
    name=package_name,
    version="0.0.0",
    description="Open RMF Fleet Adapter",
    author="Sébastien Delpeuch",
    author_email="s.delpeuch@catie.fr",
    license="Apache License 2.0",
    install_requires=[
        "python>=3.10",
    ],
    packages=[package_name],
    entry_points={
        "console_scripts": [
            "fleet_adapter=minipock_fleet_adapter.fleet_adapter:main",
            "dispatch_patrol=task.dispatch_patrol:main",
        ],
    },
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob(os.path.join("config", "*.*"))),
        (
            os.path.join("share", package_name, "config", "empty_room"),
            glob(os.path.join("config", "empty_room", "*.*")),
        ),
        (os.path.join("share", package_name, "launch"), glob(os.path.join("launch", "*.*"))),
        (os.path.join("share", package_name, "map"), glob(os.path.join("map", "*.*"))),
        (
            os.path.join("share", package_name, "map", "empty_room"),
            glob(os.path.join("map", "empty_room", "*.*")),
        ),
        (
            os.path.join("share", package_name, "map", "empty_room", "nav_graphs"),
            glob(os.path.join("map", "empty_room", "nav_graphs", "*.*")),
        ),
        (os.path.join("share", package_name, "rviz"), glob(os.path.join("rviz", "*.*"))),
        (os.path.join("share", package_name, "task"), glob(os.path.join("task", "*.*"))),
    ],
)

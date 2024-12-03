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
        ],
    },
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob(os.path.join("launch", "*.*"))),
    ],
)

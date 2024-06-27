import os
from glob import glob

from setuptools import setup

package_name = "minipock_gz"

setup(
    name=package_name,
    version="1.0.0",
    description="MiniPock Worlds for Gazebo Sim",
    author="Sébastien Delpeuch",
    author_email="s.delpeuch@catie.fr",
    license="Apache 2.0",
    install_requires=[
        "python>=3.10",
    ],
    packages=[package_name],
    entry_points={
        "console_scripts": [
            "lidar_process = minipock_gz.lidar_process:main",
            "lidar_process_multiple = minipock_gz.lidar_process_multiple:main",
        ],
    },
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "worlds"), glob(os.path.join("worlds", "*.*"))),
        (os.path.join("share", package_name, "launch"), glob(os.path.join("launch", "*.*"))),
    ],
)

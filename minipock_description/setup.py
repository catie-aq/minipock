import os
from glob import glob

from setuptools import setup

package_name = "minipock_description"

setup(
    name=package_name,
    version="1.0.0",
    description="Description package for MiniPock robot",
    author="Sébastien Delpeuch",
    author_email="s.delpeuch@catie.fr",
    license="Apache 2.0",
    install_requires=[
        "python>=3.10",
    ],
    packages=[package_name],
    entry_points={
        "console_scripts": [
            "minipock_generate = minipock_description.model:generate",
        ],
    },
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "models"), glob(os.path.join("models", "*.*"))),
        (os.path.join("share", package_name, "urdf"), glob(os.path.join("urdf", "*.*"))),
        (os.path.join("share", package_name, "launch"), glob(os.path.join("launch", "*.*"))),
    ],
)

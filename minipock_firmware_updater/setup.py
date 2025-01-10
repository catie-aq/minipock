import os

from setuptools import setup

package_name = "minipock_firmware_updater"

setup(
    name=package_name,
    version="0.0.0",
    description="Node for firmware update in µROS",
    author="Sébastien Delpeuch",
    author_email="s.delpeuch@catie.fr",
    license="Apache License 2.0",
    install_requires=[
        "python>=3.10",
    ],
    packages=[package_name],
    entry_points={
        "console_scripts": [
            "firmware_updater = minipock_firmware_updater.firmware_updater:main"
        ],
    },
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (os.path.join("share", package_name), ["package.xml"]),
    ],
)

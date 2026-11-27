"""Packaging for minipock_firmware_updater."""

from setuptools import setup

package_name = "minipock_firmware_updater"

setup(
    name=package_name,
    version="2.2.1",
    description="Node for firmware update in µROS",
    author="Sébastien Delpeuch",
    author_email="s.delpeuch@catie.fr",
    license="Apache-2.0",
    install_requires=["setuptools"],
    # Makes colcon run the tests with pytest instead of unittest.
    tests_require=["pytest"],
    packages=[package_name],
    entry_points={
        "console_scripts": ["firmware_updater = minipock_firmware_updater.firmware_updater:main"],
    },
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml"]),
    ],
)

from setuptools import find_packages, setup

package_name = "minipock_teleop"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sedelpeuch",
    maintainer_email="s.delpeuch@catie.fr",
    description="Teleoperation for MiniPock bot",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "teleop_keyboard = minipock_teleop.teleop_keyboard:main",
            "teleop_fps = minipock_teleop.teleop_fps:main",
        ],
    },
)

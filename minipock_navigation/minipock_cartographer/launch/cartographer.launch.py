# Copyright 2022 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Darby Lim

"""Map with Cartographer from the LiDAR and odometry of one robot."""

import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushROSNamespace, SetRemap
from launch_ros.parameter_descriptions import ParameterValue

NAMESPACE_MARKER = "__NS__"
LUA_BASENAME = "minipock_2d.lua"


def _render(source: Path, namespace: str, directory: Path) -> Path:
    """Write `source` into `directory` with the namespace marker replaced."""
    target = directory / source.name
    target.write_text(source.read_text().replace(NAMESPACE_MARKER, namespace))
    return target


def _cartographer(context: LaunchContext) -> list[GroupAction]:
    """Return the Cartographer nodes of the robot named by the `namespace` argument."""
    share = Path(get_package_share_directory("minipock_cartographer"))
    namespace = LaunchConfiguration("namespace").perform(context)
    directory = Path(tempfile.mkdtemp(prefix=f"cartographer_{namespace}_"))
    lua = _render(share / "config" / LUA_BASENAME, namespace, directory)
    rviz = _render(share / "rviz" / "cartographer.rviz", namespace, directory)
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    return [
        GroupAction(
            [
                PushROSNamespace(namespace),
                SetRemap("/tf", "/tf"),
                SetRemap("/tf_static", "/tf_static"),
                Node(
                    package="cartographer_ros",
                    executable="cartographer_node",
                    output="screen",
                    parameters=[{"use_sim_time": use_sim_time}],
                    arguments=[
                        "-configuration_directory",
                        str(directory),
                        "-configuration_basename",
                        lua.name,
                    ],
                ),
                Node(
                    package="cartographer_ros",
                    executable="cartographer_occupancy_grid_node",
                    output="screen",
                    parameters=[{"use_sim_time": use_sim_time}],
                    arguments=["-resolution", LaunchConfiguration("resolution")],
                    remappings=[("map", LaunchConfiguration("map_topic"))],
                ),
                Node(
                    package="rviz2",
                    executable="rviz2",
                    arguments=["-d", str(rviz)],
                    output="screen",
                    parameters=[{"use_sim_time": use_sim_time}],
                    condition=IfCondition(LaunchConfiguration("start_rviz")),
                ),
            ],
            scoped=True,
        )
    ]


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description of Cartographer SLAM for one robot.

    Returns:
        The launch description. Arguments: `namespace` (without trailing slash),
        `use_sim_time`, `resolution` (occupancy grid cell size, in meters), `map_topic`
        (default `map`, relative: `/<namespace>/map`; `/map` for Nav2 in SLAM mode) and
        `start_rviz`. The map is in the `map` frame.
    """
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="minipock_0"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("resolution", default_value="0.05"),
            DeclareLaunchArgument("map_topic", default_value="map"),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            OpaqueFunction(function=_cartographer),
        ]
    )

"""Run the Nav2 navigation servers of one robot, on the global TF tree."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushROSNamespace, SetRemap
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml

CONTAINER_NAME = "nav2_container"


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description of the Nav2 navigation servers of one robot.

    Localization (map server, AMCL) is shared by the fleet and started elsewhere.

    Returns:
        The launch description. Arguments: `namespace` (without trailing slash),
        `params_file` (already rendered by `minipock_bringup.nav2_params`), `use_sim_time`,
        `autostart` and `use_composition`.
    """
    navigation_launch = (
        Path(get_package_share_directory("nav2_bringup")) / "launch" / "navigation_launch.py"
    )
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    # navigation_launch.py evaluates `not <use_composition>` in Python: pass True or False.
    use_composition = PythonExpression(
        ["'", LaunchConfiguration("use_composition"), "'.lower() in ('true', '1')"]
    )
    # The container gets the parameters too: nodes created inside a server (the costmaps of the
    # controller and planner) read theirs from the process arguments, as in bringup_launch.py.
    container_params = ParameterFile(
        RewrittenYaml(
            source_file=LaunchConfiguration("params_file"),
            root_key=namespace,
            param_rewrites={"autostart": autostart},
            convert_types=True,
        ),
        allow_substs=True,
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="minipock_0"),
            DeclareLaunchArgument("params_file"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("use_composition", default_value="true"),
            GroupAction(
                [
                    # navigation_launch.py nodes take the namespace from the context; its
                    # `namespace` argument only nests the parameters and names the container.
                    PushROSNamespace(namespace),
                    # Global remaps come first and the first matching rule wins: they cancel
                    # the ('/tf', 'tf') remaps of nav2_bringup, frames stay on the global /tf.
                    SetRemap("/tf", "/tf"),
                    SetRemap("/tf_static", "/tf_static"),
                    Node(
                        condition=IfCondition(use_composition),
                        package="rclcpp_components",
                        executable="component_container_isolated",
                        name=CONTAINER_NAME,
                        output="screen",
                        parameters=[
                            container_params,
                            {
                                "autostart": ParameterValue(autostart, value_type=bool),
                                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                            },
                        ],
                    ),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(str(navigation_launch)),
                        launch_arguments={
                            "namespace": namespace,
                            "params_file": LaunchConfiguration("params_file"),
                            "use_sim_time": use_sim_time,
                            "autostart": autostart,
                            "use_composition": use_composition,
                            "container_name": CONTAINER_NAME,
                        }.items(),
                    ),
                ],
                scoped=True,
            ),
        ]
    )

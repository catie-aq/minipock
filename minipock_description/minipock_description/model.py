"""
This module is used to generate the sdf file from the urdf file
"""

import codecs
import os
import pathlib
import re
import subprocess

from launch_ros.substitutions import FindPackageShare

PACKAGE_NAME = "minipock_description"


def xacro_cmd(robot_name, urdf, mode):
    """
    Generate the command to run xacro and gz sdf print

    :param urdf: path to urdf file
    :param mode: mode for xacro command (holonomic or differential)
    """
    if mode not in ["holonomic", "differential"]:
        raise ValueError(f"Invalid mode choice for {robot_name} with mode {mode}.")
    xacro_command = ["xacro", urdf, f"namespace:={robot_name}/", f"mode:={mode}"]
    xacro_process = subprocess.Popen(
        xacro_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE
    )
    stdout = xacro_process.communicate()[0]
    urdf_str = codecs.getdecoder("unicode_escape")(stdout)[0]

    # run gz sdf print to generate sdf file
    model_dir = os.path.join(
        FindPackageShare(PACKAGE_NAME).find(PACKAGE_NAME), "models"
    )
    model_tmp_dir = os.path.join(model_dir, "tmp")
    model_output_file = os.path.join(model_tmp_dir, f"{robot_name}_model.urdf")
    if not os.path.exists(model_tmp_dir):
        pathlib.Path(model_tmp_dir).mkdir(parents=True, exist_ok=True)
    with open(model_output_file, "w", encoding="utf-8") as f:
        f.write(urdf_str)
    command = ["gz", "sdf", "-p", model_output_file]
    return command


def name_from_plugin(plugin_sdf):
    """
    Return the plugin name

    :param plugin_sdf: sdf string of the plugin
    :return: name of the plugin
    """
    result = re.search(r"/*<name>(.*)<\/name>", plugin_sdf)
    if result:
        return result.group(1)


def generate(robot_name, mode):
    """
    Generate the sdf file from the urdf file

    :param robot_name: The name of the robot
    :param mode: The mode for generating the sdf file

    :return: The generated sdf file
    """
    urdf_path = os.path.join(
        FindPackageShare(PACKAGE_NAME).find(PACKAGE_NAME),
        "urdf",
        "minipock.urdf.xacro",
    )
    command = xacro_cmd(robot_name, urdf_path, mode)
    model_sdf = ""
    try:
        process = subprocess.Popen(
            command, stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )
        # evaluate error output for the xacro process
        stderr = process.communicate()[1]
        err_output = codecs.getdecoder("unicode_escape")(stderr)[0]
        for line in err_output.splitlines():
            if line.find("undefined local") > 0:
                raise RuntimeError(line)

        stdout = process.communicate()[0]
        model_sdf = codecs.getdecoder("unicode_escape")(stdout)[0]
    except OSError:
        print("Detecting you're on Minipock, continuing")

    return model_sdf


def spawn_args(
    robot_name="minipock", robot_position_str="0.0 0.0 0.0", mode="differential"
):
    """
    Return the spawning arguments for the create command

    :param robot_name: name of the robot
    :param robot_position_str: position of the robot in the world*
    :param mode: mode for the robot

    :return: list of arguments
    """
    x, y, z = robot_position_str.split(" ")
    model_sdf = generate(robot_name, mode)
    return [
        "-string",
        model_sdf,
        "-name",
        robot_name,
        "-allow_renaming",
        "false",
        "-x",
        x,
        "-y",
        y,
        "-z",
        z,
    ]

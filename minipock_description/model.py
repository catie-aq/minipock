"""
This module is used to generate the sdf file from the urdf file
"""

import codecs
import os
import pathlib
import re
import subprocess

import sdformat13 as sdf
from launch_ros.substitutions import FindPackageShare

PACKAGE_NAME = 'minipock_description'
ROBOT_NAME = 'minipock'


def xacro_cmd(urdf):
    """
    Generate the command to run xacro and gz sdf print

    :param urdf: path to urdf file
    :return: command to run
    """
    xacro_command = ['xacro', urdf, f'namespace:={ROBOT_NAME}']
    xacro_process = subprocess.Popen(xacro_command,
                                     stdout=subprocess.PIPE,
                                     stderr=subprocess.PIPE)
    stdout = xacro_process.communicate()[0]
    urdf_str = codecs.getdecoder('unicode_escape')(stdout)[0]

    # run gz sdf print to generate sdf file
    model_dir = os.path.join(FindPackageShare(PACKAGE_NAME).find(PACKAGE_NAME), 'models')
    model_tmp_dir = os.path.join(model_dir, 'tmp')
    model_output_file = os.path.join(model_tmp_dir, 'model.urdf')
    if not os.path.exists(model_tmp_dir):
        pathlib.Path(model_tmp_dir).mkdir(parents=True, exist_ok=True)
    with open(model_output_file, 'w') as f:
        f.write(urdf_str)
    command = ['gz', 'sdf', '-p', model_output_file]
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


def payload_from_sdf(model_sdf):
    """
    Parse the sdf file for payloads

    :param model_sdf: sdf string of the model
    :return: dictionary of payloads
    """
    payload = {}
    root = sdf.Root()
    root.load_sdf_string(model_sdf)
    model = root.model()
    link = None
    for link_index in range(model.link_count()):
        link = model.link_by_index(link_index)
        for sensor_index in range(link.sensor_count()):
            sensor = link.sensor_by_index(sensor_index)
            payload[sensor.name()] = [link.name(), sensor.type()]
    plugins = model.plugins()
    for plugin in plugins:
        if plugin.name() == 'gz::sim::systems::Thruster':
            name = name_from_plugin(plugin.__str__())
            payload['thruster_thrust_' + name] = [link.name(), name]
        elif plugin.name() == 'gz::sim::systems::JointPositionController':
            name = name_from_plugin(plugin.__str__())
            payload['thruster_rotate_' + name] = [link.name(), name]
        else:
            payload[plugin.name()] = ['', plugin.filename()]
    return payload


def generate():
    """
    Generate the sdf file from the urdf file

    :return: return the sdf
    """
    urdf_path = os.path.join(FindPackageShare(PACKAGE_NAME).find(PACKAGE_NAME),
                             'urdf', ROBOT_NAME + '.urdf.xacro')
    command = xacro_cmd(urdf_path)
    process = subprocess.Popen(command,
                               stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE)

    # evaluate error output for the xacro process
    stderr = process.communicate()[1]
    err_output = codecs.getdecoder('unicode_escape')(stderr)[0]
    for line in err_output.splitlines():
        if line.find('undefined local') > 0:
            raise RuntimeError(line)

    stdout = process.communicate()[0]
    model_sdf = codecs.getdecoder('unicode_escape')(stdout)[0]

    payload_from_sdf(model_sdf)

    return model_sdf


def spawn_args(position):
    """
    Return the spawning arguments for the create command

    :param position: list of position and rotation
    :return: list of arguments
    """
    model_sdf = generate()
    return ['-string', model_sdf,
            '-name', ROBOT_NAME,
            '-allow_renaming', 'false',
            '-x', str(position[0]),
            '-y', str(position[1]),
            '-z', str(position[2]),
            '-R', str(position[3]),
            '-P', str(position[4]),
            '-Y', str(position[5])]

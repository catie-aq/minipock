from ament_index_python.packages import get_package_share_directory
import yaml
import os


def config():
    minipock_package_path = get_package_share_directory("minipock")
    minipocks_yaml_path = os.path.join(minipock_package_path, "minipocks.yaml")
    with open(minipocks_yaml_path, "r") as file:
        yaml_data = yaml.safe_load(file)
    config_dict = {}
    for key, value in yaml_data.items():
        config_dict[key] = value
    return config_dict

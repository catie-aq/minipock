from ament_index_python.packages import get_package_share_directory
import yaml
import os
import jsonschema


def validate_config(config_to_check):
    schema = {
        "type": "object",
        "properties": {
            "namespace": {"type": "string"},
            "bringup": {"type": "boolean"},
            "use_sim_time": {"type": "boolean"},
            "fleet": {
                "type": "object",
                "patternProperties": {
                    "^[a-z0-9]+$": {
                        "type": "object",
                        "properties": {
                            "mode": {
                                "type": "string",
                                "enum": ["differential", "holonomic"],
                            },
                            "position": {
                                "type": ["null", "array"],
                            },
                        },
                        "required": ["mode", "position"],
                    }
                },
            },
        },
        "required": ["namespace", "bringup", "use_sim_time", "fleet"],
    }

    try:
        jsonschema.validate(config_to_check, schema)
    except jsonschema.ValidationError as e:
        raise e
    except TypeError as e:
        raise TypeError("Some fleet keys are not strings") from e


def config():
    minipock_package_path = get_package_share_directory("minipock")
    minipocks_yaml_path = os.path.join(minipock_package_path, "minipocks.yaml")
    with open(minipocks_yaml_path, "r") as file:
        yaml_data = yaml.safe_load(file)
    config_dict = {}
    for key, value in yaml_data.items():
        config_dict[key] = value
    validate_config(config_dict)
    return config_dict

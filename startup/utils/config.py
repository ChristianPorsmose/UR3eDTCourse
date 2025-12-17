from pyhocon import ConfigFactory
from importlib.resources import files, path

def resource_file_path_w_setuptools(package, resource = None):
    package = package.replace("/", ".")
    resource_path = files(package) / resource if resource else files(package)
    return resource_path

def load_config_w_setuptools(config_file_name) -> dict:
    """Function for loading a config, when setuptools is used for absolute imports"""
    with path(config_file_name.split(".")[0], config_file_name) as file_path:
        config = ConfigFactory.parse_file(file_path)
    return config

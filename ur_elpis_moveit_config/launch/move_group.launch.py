from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from ament_index_python.packages import get_package_share_directory
import yaml
import os


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("my_robot_cell", package_name="ur_elpis_moveit_config").to_moveit_configs()
    
    # Load octomap config to get resolution
    octomap_config = load_yaml("ur_elpis_moveit_config", "config/octomap.yaml")
    octomap_resolution = 0.01  # default
    if octomap_config and "octomap_resolution" in octomap_config:
        octomap_resolution = octomap_config["octomap_resolution"]
    
    # Get the config dictionary and add octomap_resolution to occupancy_map_monitor namespace
    config_dict = moveit_config.to_dict()
    
    # Add resolution to occupancy_map_monitor namespace
    # MoveIt's occupancy_map_monitor looks for "resolution" (not "octomap_resolution")
    if "occupancy_map_monitor" not in config_dict:
        config_dict["occupancy_map_monitor"] = {}
    config_dict["occupancy_map_monitor"]["resolution"] = octomap_resolution
    # Also try octomap_resolution as a fallback
    config_dict["occupancy_map_monitor"]["octomap_resolution"] = octomap_resolution
    
    # Try to inject the parameter into moveit_config
    # MoveItConfigsBuilder stores configs in an internal dictionary
    try:
        # Access the internal _MoveItConfigs__moveit_configs_dict
        if hasattr(moveit_config, '_MoveItConfigs__moveit_configs_dict'):
            moveit_config._MoveItConfigs__moveit_configs_dict.update(config_dict)
    except AttributeError:
        # If that doesn't work, try accessing it differently
        try:
            # Some versions might store it differently
            for key, value in config_dict.items():
                if hasattr(moveit_config, key):
                    setattr(moveit_config, key, value)
        except:
            pass
    
    return generate_move_group_launch(moveit_config)

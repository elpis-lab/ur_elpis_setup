from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("my_robot_cell", package_name="ur_elpis_moveit_config")
        .planning_pipelines(pipelines=["ompl", "chomp"])
        .to_moveit_configs()
    )
    # Needed for moveit task constructor
    moveit_config.move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability",
        "disable_capabilities": ""
    }
    
    return generate_move_group_launch(moveit_config)

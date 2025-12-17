from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("my_robot_cell", package_name="ur_elpis_moveit_config")
        .robot_description_semantic(file_path="config/my_robot_cell.srdf")
        .planning_pipelines(pipelines=["ompl", "chomp"])
        .to_moveit_configs()
    )
    return generate_moveit_rviz_launch(moveit_config)

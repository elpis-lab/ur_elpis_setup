# UR ELPIS Setup

This repository contains the complete setup for the UR10 robot with Robotis RH-P12-RN gripper in the ELPIS lab at WPI.

## Packages

This repository contains three ROS2 packages:

### `ur_elpis_description`
Contains the URDF description of the robot cell, including:
- UR10 robot arm
- Robotis RH-P12-RN gripper
- RealSense D435 camera
- Lab environment (table, mounting plate, ceiling slab)

### `ur_elpis_control`
Contains the `ros2_control` configuration and launch files for:
- Robot driver setup
- Controller configuration
- Robot state publishing

### `ur_elpis_moveit_config`
Contains MoveIt configuration files for motion planning:
- SRDF (Semantic Robot Description Format)
- Kinematics configuration
- Joint limits
- MoveIt controllers configuration
- Launch files for MoveIt

## Building

```bash
cd /path/to/workspace
colcon build --packages-select ur_elpis_description ur_elpis_control ur_elpis_moveit_config
source install/setup.bash
```

## Usage

### Launch Robot Driver

```bash
ros2 launch ur_elpis_control start_robot.launch.py
```

### Launch MoveIt

```bash
ros2 launch ur_elpis_moveit_config move_group.launch.py
```

### Launch MoveIt with RViz

```bash
ros2 launch ur_elpis_moveit_config moveit_rviz.launch.py
```

### View Robot in RViz

```bash
ros2 launch ur_elpis_description view_robot.launch.py
```

## Dependencies

- `ur_description` - UR robot description
- `ur_robot_driver` - UR robot driver
- `ur_controllers` - UR controllers
- `moveit2` - MoveIt motion planning framework
- `xacro` - Xacro macro processor

## License

See individual package LICENSE files.


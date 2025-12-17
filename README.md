# UR ELPIS Setup

This repository contains the complete setup for the UR10 robot with Robotis RH-P12-RN gripper in the ELPIS lab at WPI.

## Packages

This repository contains four ROS2 packages:

### `ur_elpis_description`
Contains the URDF description of the robot cell, including:
- UR10 robot arm
- Robotis RH-P12-RN gripper
- RealSense D435 camera

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

### `setup_environment`
Contains the environment setup node for adding static collision objects to the MoveIt planning scene:
- Generic support for mesh and primitive (box/cylinder) objects

## Building

```bash
cd /path/to/workspace
colcon build --packages-select ur_elpis_description ur_elpis_control ur_elpis_moveit_config setup_environment
source install/setup.bash
```

## Usage

### Launch Complete System (Recommended)

Launches MoveIt move_group, RViz, and environment setup (table/ceiling) all together:

```bash
ros2 launch ur_elpis_moveit_config complete_system.launch.py
```

With custom environment config:
```bash
ros2 launch ur_elpis_moveit_config complete_system.launch.py config_file:=/path/to/custom_config.yaml
```

Without RViz:
```bash
ros2 launch ur_elpis_moveit_config complete_system.launch.py launch_rviz:=false
```

### Launch Robot Driver

```bash
ros2 launch ur_elpis_control start_robot.launch.py
```

### Launch Move Group Functionalities

```bash
ros2 launch ur_elpis_moveit_config move_group.launch.py
```

### Launch MoveIt RViz Interface

```bash
ros2 launch ur_elpis_moveit_config moveit_rviz.launch.py
```

### Setup Environment

To add table and other static obstacles to planning scene:

```bash
ros2 run setup_environment setup_environment_node
```

### View Robot in RViz

```bash
ros2 launch ur_elpis_description view_robot.launch.py
```

## Environment Configuration

The `setup_environment` package uses a YAML configuration file to define static objects in the planning scene. The default configuration is located at:

```
setup_environment/config/environment_config.yaml
```

### Configuration Structure

Each object can be either:
- **`type: "mesh"`** - Loads a mesh file (e.g., `.obj`, `.stl`)
- **`type: "primitive"`** - Uses geometric primitives (box or cylinder)

## Dependencies

- `ur_description` - UR robot description
- `ur_robot_driver` - UR robot driver
- `ur_controllers` - UR controllers
- `moveit2` - MoveIt motion planning framework
- `xacro` - Xacro macro processor
- `libyaml-cpp-dev` - YAML-C++ library (for setup_environment)
- `geometric_shapes` - Geometric shapes library (for mesh loading)

## License

See individual package LICENSE files.







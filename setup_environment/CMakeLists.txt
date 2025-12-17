cmake_minimum_required(VERSION 3.8)
project(setup_environment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Try to find yaml-cpp (via vendor package or system)
find_package(yaml-cpp QUIET)
if(NOT yaml-cpp_FOUND)
  message(FATAL_ERROR "yaml-cpp not found. Please install libyaml-cpp-dev or ros-humble-yaml-cpp-vendor")
endif()

# Executable
add_executable(setup_environment_node
  src/setup_environment_node.cpp
)

ament_target_dependencies(setup_environment_node
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  shape_msgs
  geometric_shapes
  geometry_msgs
  ament_index_cpp
)

# Link yaml-cpp library explicitly
# Force use of system library (0.7)
# Find system library - try multiple versions for portability
find_library(YAML_CPP_LIBRARY 
  NAMES 
    libyaml-cpp.so.0.7
    libyaml-cpp.so.0.8
    libyaml-cpp.so
  PATHS 
    /usr/lib/x86_64-linux-gnu
    /usr/lib 
    /usr/local/lib
  REQUIRED
)

get_filename_component(YAML_CPP_LIB_DIR ${YAML_CPP_LIBRARY} DIRECTORY)

# Link directly to the library file (bypass imported targets to avoid version mismatch)
target_link_libraries(setup_environment_node ${YAML_CPP_LIBRARY})

set_target_properties(setup_environment_node PROPERTIES
  INSTALL_RPATH "${YAML_CPP_LIB_DIR}"
  INSTALL_RPATH_USE_LINK_PATH TRUE
  BUILD_WITH_INSTALL_RPATH TRUE
)

message(STATUS "Linked yaml-cpp from: ${YAML_CPP_LIBRARY}")
message(STATUS "Library directory: ${YAML_CPP_LIB_DIR}")

# Install executable
install(TARGETS setup_environment_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()


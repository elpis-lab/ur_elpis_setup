cmake_minimum_required(VERSION 3.5)
project(ur_elpis_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch rviz urdf assets
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

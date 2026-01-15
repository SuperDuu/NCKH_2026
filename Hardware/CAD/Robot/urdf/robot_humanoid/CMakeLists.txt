cmake_minimum_required(VERSION 3.8)

project(robot_humanoid)

find_package(ament_cmake REQUIRED)

# Install resource directories
install(
  DIRECTORY
    config
    launch
    meshes
    urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

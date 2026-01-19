cmake_minimum_required(VERSION 3.8)
project(ros2_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(xacro REQUIRED)
find_package(trajectory_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(imu_sub_node src/imu_sub_node.cpp)
ament_target_dependencies(imu_sub_node rclcpp sensor_msgs)
add_executable(uvc_node src/uvc_node.cpp)
ament_target_dependencies(uvc_node rclcpp sensor_msgs trajectory_msgs geometry_msgs)
add_executable(uvc_ik_node src/uvc_ik_node.cpp)
ament_target_dependencies(uvc_ik_node rclcpp sensor_msgs trajectory_msgs)

install(TARGETS
  imu_sub_node
  uvc_node
  uvc_ik_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  urdf
  rviz
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  urdf/meshes
  DESTINATION share/${PROJECT_NAME}/urdf
)

ament_export_dependencies(rclcpp sensor_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

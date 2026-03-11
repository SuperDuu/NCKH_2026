cmake_minimum_required(VERSION 3.8)
project(ros2_pkg_v2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(gz_ros2_control REQUIRED)

add_executable(imu_process_node src/imu_process_node.cpp)
ament_target_dependencies(imu_process_node rclcpp sensor_msgs)

add_executable(humanoid_llc_node src/humanoid_llc_node.cpp)
ament_target_dependencies(humanoid_llc_node rclcpp sensor_msgs trajectory_msgs geometry_msgs std_msgs)

add_executable(imu_process_node_v2 src/imu_process_node_v2.cpp)
ament_target_dependencies(imu_process_node_v2 rclcpp sensor_msgs)

add_executable(humanoid_llc_node_v2 src/humanoid_llc_node_v2.cpp)
ament_target_dependencies(humanoid_llc_node_v2 rclcpp sensor_msgs trajectory_msgs geometry_msgs std_msgs)

add_executable(imu_process_node_v3 src/imu_process_node_v3.cpp)
ament_target_dependencies(imu_process_node_v3 rclcpp sensor_msgs)

add_executable(humanoid_llc_node_v3 src/humanoid_llc_node_v3.cpp)
ament_target_dependencies(humanoid_llc_node_v3 rclcpp sensor_msgs trajectory_msgs geometry_msgs std_msgs)

# --- PHẦN PYTHON ---
ament_python_install_package(ros2_pkg)

install(TARGETS imu_process_node humanoid_llc_node humanoid_llc_node_v2 imu_process_node_v2 humanoid_llc_node_v3 imu_process_node_v3 DESTINATION lib/${PROJECT_NAME})


install(PROGRAMS ros2_pkg/export_onnx.py DESTINATION lib/${PROJECT_NAME} RENAME export_onnx)
install(PROGRAMS ros2_pkg/visual.py DESTINATION lib/${PROJECT_NAME} RENAME visual)
install(PROGRAMS ros2_pkg/stand_test.py DESTINATION lib/${PROJECT_NAME} RENAME stand_test)
install(PROGRAMS ros2_pkg/reactive_balance.py DESTINATION lib/${PROJECT_NAME} RENAME reactive_balance)
install(PROGRAMS ros2_pkg/ppo_balance_sim_20hz.py DESTINATION lib/${PROJECT_NAME} RENAME ppo_balance_sim_20hz)
install(PROGRAMS ros2_pkg/ppo_balance_sim_20hz_phase2.py DESTINATION lib/${PROJECT_NAME} RENAME ppo_balance_sim_20hz_phase2)
install(PROGRAMS ros2_pkg/cpg_phase_1.py DESTINATION lib/${PROJECT_NAME} RENAME cpg_phase_1)

install(DIRECTORY launch urdf config worlds
  DESTINATION share/${PROJECT_NAME})

# Đảm bảo copy cả meshes nếu có
install(DIRECTORY urdf/meshes
  DESTINATION share/${PROJECT_NAME}/urdf)

ament_export_dependencies(rclcpp rclpy sensor_msgs geometry_msgs trajectory_msgs gz_ros2_control)
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

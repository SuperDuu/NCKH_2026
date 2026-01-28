cmake_minimum_required(VERSION 3.8)
project(ros2_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# urdf và xacro thường dùng trong launch/urdf, không nhất thiết phải find ở đây trừ khi bạn dùng API C++ của chúng

# --- PHẦN C++ ---
add_executable(imu_process_node src/imu_process_node.cpp)
ament_target_dependencies(imu_process_node rclcpp sensor_msgs)

add_executable(uvc_node src/uvc_node.cpp)
ament_target_dependencies(uvc_node rclcpp sensor_msgs trajectory_msgs geometry_msgs std_msgs)

# --- PHẦN PYTHON ---
# Giả sử bạn để file py trong thư mục cùng tên package: ros2_pkg/
ament_python_install_package(${PROJECT_NAME})

# --- INSTALLATION ---
install(TARGETS imu_process_node uvc_node
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  ros2_pkg/rl_training_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME rl_training_node)

install(DIRECTORY launch urdf rviz worlds
  DESTINATION share/${PROJECT_NAME})

# Đảm bảo copy cả meshes nếu có
install(DIRECTORY urdf/meshes
  DESTINATION share/${PROJECT_NAME}/urdf)

ament_export_dependencies(rclcpp rclpy sensor_msgs geometry_msgs trajectory_msgs)
ament_package()
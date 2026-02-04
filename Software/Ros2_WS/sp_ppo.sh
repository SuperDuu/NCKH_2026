#build
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Ép chạy nội bộ (Cực kỳ quan trọng cho máy 32GB RAM để tránh lag)
export ROS_LOCALHOST_ONLY=1
colcon build --symlink-install
source install/setup.bash

chmod +x src/ros2_pkg/ros2_pkg/task_space_training.py
#launch file
ros2 launch ros2_pkg task_space.launch.py


#build
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Ép chạy nội bộ (Cực kỳ quan trọng cho máy 32GB RAM để tránh lag)
export ROS_LOCALHOST_ONLY=1
colcon build --cmake-args -Wno-dev
source install/setup.bash

chmod +x src/ros2_pkg/ros2_pkg/train_balance_ppo.py
#launch file
ros2 launch ros2_pkg humanoid_train.launch.py

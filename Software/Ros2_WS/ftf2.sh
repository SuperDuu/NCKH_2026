colcon build --cmake-args -Wno-dev
source install/setup.bash

chmod +x src/ros2_pkg/ros2_pkg/ft_force_v2.py
#launch file
ros2 launch ros2_pkg ft_force_v2.launch.py

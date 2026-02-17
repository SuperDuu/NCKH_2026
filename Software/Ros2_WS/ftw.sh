colcon build --cmake-args -Wno-dev
source install/setup.bash

chmod +x src/ros2_pkg/ros2_pkg/ft_walk.py
#launch file
ros2 launch ros2_pkg ft_walk.launch.py
colcon build --cmake-args -Wno-dev
source install/setup.bash

chmod +x src/ros2_pkg_v2/ros2_pkg/cpg_phase_1.py
#launch file
ros2 launch ros2_pkg_v2 cpg_phase_1.launch.py

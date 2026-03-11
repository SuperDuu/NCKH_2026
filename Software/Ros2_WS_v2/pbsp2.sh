colcon build --cmake-args -Wno-dev
source install/setup.bash

chmod +x src/ros2_pkg_v2/ros2_pkg/ppo_balance_sim_20hz_phase2.py
#launch file
ros2 launch ros2_pkg_v2 sim_balance_20hz_phase2.launch.py

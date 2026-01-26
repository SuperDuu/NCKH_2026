
#build
colcon build

#source

source install/setup.bash

#launch file
ros2 launch ros2_pkg sim.launch.py

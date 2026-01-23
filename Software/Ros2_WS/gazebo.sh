
#build
colcon build

#source

source install/setup.sh

#launch file
ros2 launch ros2_pkg sim.launch.py

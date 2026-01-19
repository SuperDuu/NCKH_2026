rm -rf \build \install \log
#build
colcon build

#source

source install/setup.sh

#launch file
ros2 launch ros2_pkg sim.launch.py

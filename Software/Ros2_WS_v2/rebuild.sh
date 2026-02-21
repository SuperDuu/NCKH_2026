
pkill -f gz
pkill -f ruby
pkill -f ros2
pkill -f robot_state_publisher
sleep 1 

rm -rf build/ install/ log/

colcon build

source install/setup.bash

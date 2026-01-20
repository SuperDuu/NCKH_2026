
pkill -f gz
pkill -f ruby
pkill -f ros2
pkill -f robot_state_publisher
pkill -f uvc_node
sleep 1 

rm -rf build/ install/ log/

colcon build --symlink-install

source install/setup.bash


#build
colcon build

#source

source install/setup.bash

chmod +x src/ros2_pkg/ros2_pkg/rl_training_node.py
chmod +x src/ros2_pkg/ros2_pkg/rl_inference.py
chmod +x src/ros2_pkg/ros2_pkg/rl_ppo_training.py
chmod +x src/ros2_pkg/ros2_pkg/rl_training_fast.py
#launch file
ros2 launch ros2_pkg sim.launch.py

#!/bin/bash
# =============================================================
# REACTIVE BALANCE TRAINING
# Robot đứng yên, bị đẩy 400N → học bước chân phục hồi cân bằng
# =============================================================

echo "============================================="
echo "  REACTIVE BALANCE TRAINING"
echo "  Push: 250-450N | Lift: 2.5cm | DT: 50ms"
echo "============================================="

# Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Source workspace
WORKSPACE="$HOME/Desktop/NCKH_2026/Software/Ros2_WS_v2"
source "$WORKSPACE/install/setup.bash"

echo ">>> Building ros2_pkg_v2..."
cd "$WORKSPACE"
colcon build --packages-select ros2_pkg_v2
source "$WORKSPACE/install/setup.bash"

echo ""
echo ">>> Launching Reactive Balance Training..."
echo ">>> Gazebo sẽ tự chạy (-r). Training bắt đầu sau 5 giây."
echo ""

ros2 launch ros2_pkg_v2 reactive_balance.launch.py

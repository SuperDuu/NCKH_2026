#!/bin/bash
# =============================================================
# Script chạy Stand Test - Kiểm tra robot đứng yên ổn định
# Bấm Play trong Gazebo sau khi robot spawn xong
# =============================================================

echo "============================================="
echo "  STAND TEST - Kiểm tra ổn định MG996R"
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
echo ">>> Launching Stand Test..."
echo ">>> Nhấn PLAY trong Gazebo khi robot đã spawn xong!"
echo ""

ros2 launch ros2_pkg_v2 stand_test.launch.py

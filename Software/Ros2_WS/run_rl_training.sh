#!/bin/bash

# Kích hoạt ROS 2 environment
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "=========================================="
echo "RL Training - Tối ưu hóa tham số UVC"
echo "=========================================="
echo ""
echo "ACTION SPACE (7 tham số):"
echo "  [0] gain:           0.10 ~ 0.30  (độ lợi toàn cục)"
echo "  [1] scale_base:     0.10 ~ 0.20  (hệ số scale hình học)"
echo "  [2] step_duration:  20.0 ~ 35.0  (độ dài bước, cycle)"
echo "  [3] landing_phase:  5.0 ~ 8.0    (pha hạ chân, cycle)"
echo "  [4] stance_width:   30.0 ~ 45.0  (khoảng cách hai chân, mm)"
echo "  [5] max_foot_lift:  8.0 ~ 15.0   (độ cao nâng chân, mm)"
echo "  [6] recovery_rate:  0.05 ~ 0.20  (tốc độ phục hồi)"
echo ""
echo "REWARD FUNCTION:"
echo "  - Cân bằng tốt (tilt nhỏ) → reward cao"
echo "  - Robot ngã (tilt > 45°) → penalty -2.0"
echo "  - Learning rate: 0.001"
echo ""
echo "Starting RL Training Node..."
echo ""

# Chạy RL training node
ros2 run ros2_pkg rl_training_node

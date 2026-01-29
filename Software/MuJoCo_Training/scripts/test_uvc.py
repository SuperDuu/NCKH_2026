import mujoco
import mujoco.viewer
import numpy as np
import time
from uvc_math import RobotMath

# 1. Load model
model = mujoco.MjModel.from_xml_path("../assets/Robot.xml")
data = mujoco.MjData(model)
math = RobotMath()

# 2. Tham số UVC cố định (Lấy từ mặc định gazebo của bạn)
# [gain, scale_base, step_duration, landing_phase, stance_width, max_foot_lift, recovery_rate]
test_params = [0.18, 0.12, 25.0, 6.0, 20.0, 12.0, 0.1]

# Biến điều khiển chu kỳ
fwct = 0.0
support_leg = 0 # 0: Phải trụ, 1: Trái trụ

with mujoco.viewer.launch_passive(model, data) as v:
    print("🚀 Đang test UVC logic... Nhấn 'Space' để tạm dừng.")
    
    # Hạ thấp độ cao khởi tạo để tránh robot bị rơi tự do quá mạnh
    data.qpos[2] = 0.26 # Hạ từ 0.35 xuống 0.26m
    
    while v.is_running():
        step_start = time.time()

        # Đọc IMU (Giả lập robot đứng thẳng nếu chưa có nhiễu)
        pitch, roll = 0.0, 0.0 
        # Nếu muốn test độ nhạy, bạn có thể thử cho pitch = 0.1 (nghiêng 5 độ)
        
        # Tính toán 10 góc khớp
        fwctEnd = test_params[2]
        joint_angles = math.compute_steps(test_params, [pitch, roll], fwct, fwctEnd, support_leg)
        
        # Gửi lệnh đến Actuators
        data.ctrl[:10] = joint_angles
        
        # Chạy mô phỏng vật lý
        mujoco.mj_step(model, data)
        
        # Cập nhật chu kỳ
        fwct += 1.0
        if fwct >= fwctEnd:
            fwct = 0.0
            support_leg = 1 - support_leg
            print(f"🔄 Đổi chân trụ: {'TRÁI' if support_leg==1 else 'PHẢI'}")

        v.sync()
        
        # Duy trì tốc độ thời gian thực (100Hz)
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
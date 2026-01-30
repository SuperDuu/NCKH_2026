import mujoco
import mujoco.viewer
import numpy as np
import time
from uvc_math import RobotMath

# 1. Load model chuẩn vật lý
model = mujoco.MjModel.from_xml_path("../assets/Robot.xml")
data = mujoco.MjData(model)
math = RobotMath()

# 2. Tham số test (Đã chuyển đổi sang hệ mét cho các tham số kích thước)
# Index: 0:gain, 1:scale_b, 2:fwctEnd, 3:landP, 4:stance_w(m), 5:fhMax(m), 6:rr
# stance_w để 20mm = 0.02m
test_params = [0.0, 0.0, 40.0, 6.0, 0.02, 0.0, 0.0] 

with mujoco.viewer.launch_passive(model, data) as v:
    print(f"🚀 TEST STANDING STILL (METRIC): Robot đứng yên, Target H: {math.HEIGHT_STD}m")
    
    # Ép vị trí gốc (đơn vị Mét)
    data.qpos[2] = 0.28 
    
    while v.is_running():
        step_start = time.time()

        # Giữ góc nghiêng bằng 0 để kiểm tra IK tĩnh
        pitch, roll = 0.0, 0.0 
        
        # 3. Tính toán góc khớp (Sử dụng hàm đã chuẩn hóa Mét)
        # Với pitch/roll = 0 và gain/rr = 0, autoH sẽ giữ nguyên ở 0.2m (200mm)
        joint_angles = math.compute_joints(pitch, roll, test_params)
        
        # 4. Gửi lệnh điều khiển 10 motor chân
        data.ctrl[:10] = joint_angles
        
        # 5. Quan sát trạng thái
        if int(time.time()*10) % 10 == 0:
            # In ra mm để bạn dễ theo dõi nhưng tính toán vẫn là mét
            print(f"H: {math.autoH*1000:.1f}mm | dxi: {math.dxi*1000:.1f}mm | Support: {math.support_leg} | ht: {joint_angles[1]*180/np.pi:.2f}°", flush=True)

        # Chạy vật lý
        mujoco.mj_step(model, data)
        v.sync()
        
        # Duy trì 100Hz
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
import mujoco
import mujoco.viewer
import time
import numpy as np

# 1. Load model từ assets
model = mujoco.MjModel.from_xml_path("../assets/Robot.xml")
data = mujoco.MjData(model)

def launch_inspector():
    # 2. "Treo" robot lên không trung để dễ quan sát (giống Rviz)
    # Chúng ta sẽ tắt trọng lực tạm thời hoặc ép vị trí gốc
    model.opt.gravity[2] = 0 # Tắt trọng lực để robot không rơi
    
    # Đặt robot ở độ cao vừa tầm mắt
    data.qpos[2] = 0.5 
    
    print("🚀 Đang mở Robot Inspector...")
    print("💡 Mẹo: Dùng các thanh trượt 'Actuators' bên phải để xoay từng khớp.")
    print("💡 Nhấn '0' để hiện hệ trục, 'J' để hiện tên khớp.")

    with mujoco.viewer.launch_passive(model, data) as v:
        while v.is_running():
            step_start = time.time()
            
            # Giữ robot cố định tại một điểm trong không gian
            data.qpos[0:3] = [0, 0, 0.5] # X, Y, Z
            data.qpos[3:7] = [1, 0, 0, 0] # Quaternion cân bằng
            
            # Chạy bước mô phỏng để cập nhật các thanh trượt điều khiển
            mujoco.mj_step(model, data)
            
            # Đồng bộ hình ảnh
            v.sync()
            
            # Duy trì tần số 60Hz để mượt mà
            time_until_next_step = 1.0/60.0 - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

if __name__ == "__main__":
    launch_inspector()
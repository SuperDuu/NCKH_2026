import mujoco
import mujoco.viewer
import numpy as np
import time

# ====== LOAD MODEL ======
model = mujoco.MjModel.from_xml_path("../assets/Robot.xml")
data  = mujoco.MjData(model)

# ====== IN THÔNG TIN KHỚP & ACTUATOR ======
print("=== ACTUATORS ===")
for i in range(model.nu):
    print(f"[{i}] actuator: {model.actuator(i).name}")

print("\n=== JOINTS ===")
for i in range(model.njnt):
    print(f"[{i}] joint: {model.joint(i).name}")

# ====== RESET ======
mujoco.mj_resetData(model, data)
data.qpos[:] = model.key_qpos[0] if model.nkey > 0 else data.qpos
mujoco.mj_forward(model, data)

# ====== VIEWER ======
with mujoco.viewer.launch_passive(
    model,
    data,
    show_left_ui=True,    # 👈 có slider
    show_right_ui=True
) as viewer:

    print("\n🎮 DÙNG SLIDER BÊN TRÁI ĐỂ ĐIỀU KHIỂN ACTUATOR")
    print("ESC để thoát")

    while viewer.is_running():
        # giữ ctrl = giá trị hiện tại (slider override)
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.002)

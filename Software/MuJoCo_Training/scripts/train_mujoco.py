import mujoco
import mujoco.viewer
import numpy as np
import torch
import os
import time
from uvc_math import RobotMath
from rl_ppo_training import PPOAgent

class MuJoCoTrainingNode:
    def __init__(self):
        # --- MÔI TRƯỜNG ---
        self.model = mujoco.MjModel.from_xml_path("../assets/Robot.xml")
        self.data = mujoco.MjData(self.model)
        self.math = RobotMath()
        
        # --- PPO SETUP (6 State, 7 Action) ---
        self.state_dim, self.action_dim = 6, 7
        self.ppo_agent = PPOAgent(self.state_dim, self.action_dim, 0.0001, 0.0005, 0.99, 10, 0.2)
        
        # [QUAN TRỌNG] CHUYỂN DẢI THAM SỐ SANG MÉT (m)
        # Index: 0:gain, 1:scale, 2:fwctEnd, 3:landP, 4:stance(m), 5:lift(m), 6:rr
        # Chỉnh 4 và 5 về đơn vị mét (ví dụ 15mm -> 0.015m)
        self.param_mins = np.array([0.05, 0.05, 30.0, 3.0, 0.015, 0.005, 0.01])
        self.param_maxs = np.array([0.25, 0.30, 60.0, 8.0, 0.035, 0.020, 0.15])
        
        # Khởi tạo Std thấp để giảm nhiễu
        self.ppo_agent.set_action_std(0.3)
        
        if not os.path.exists("../checkpoints"): os.makedirs("../checkpoints")

    def get_observation(self):
        """Quan sát IMU (Radians) và Tiến độ chu kỳ"""
        q = self.data.sensor('quat').data
        # Tính Pitch/Roll từ Quaternion chuẩn MuJoCo
        sinp = 2 * (q[0] * q[2] - q[3] * q[1])
        pitch = np.arcsin(np.clip(sinp, -1, 1))
        roll = np.arctan2(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1]**2 + q[2]**2))
        
        # State gồm Pitch, Roll, Progress (0.0-1.0) và 3 giá trị trống
        obs = np.array([pitch, roll, self.math.fwct / 100.0])
        return np.concatenate([obs, [0, 0, 0]]).astype(np.float32)

    def train(self):
        # BẬT GIAO DIỆN QUAN SÁT
        with mujoco.viewer.launch_passive(self.model, self.data) as v:
            print("🚀 TRAINING IN METRIC UNITS: Final Convergence Attempt...")
            total_steps = 0
            
            for episode in range(15000):
                mujoco.mj_resetData(self.model, self.data)
                # Độ cao an toàn khởi tạo (mét)
                self.data.qpos[2] = 0.28 
                self.math.__init__() # Reset class RobotMath về trạng thái STANDING_STD
                
                state = self.get_observation()
                ep_reward = 0
                
                for t in range(5000):
                    if not v.is_running(): break

                    # Chọn action và map vào dải vật lý (Mét)
                    action = self.ppo_agent.select_action(state)
                    phys_act = self.param_mins + (action + 1.0) * 0.5 * (self.param_maxs - self.param_mins)
                    
                    # 1. Tính toán góc khớp (Input: pitch, roll, params)
                    joint_angles = self.math.compute_joints(state[0], state[1], phys_act)
                    
                    # 2. Gửi lệnh (Đảm bảo mapping 10 khớp chân chuẩn xác)
                    self.data.ctrl[0] = 0.0 # Giữ khớp eo đứng yên
                    self.data.ctrl[1:11] = joint_angles
                    
                    # 3. Bước mô phỏng và cập nhật chu kỳ UVC
                    mujoco.mj_step(self.model, self.data)
                    self.math.update_cycle(phys_act[2]) 
                    
                    # --- HÀM REWARD TUYẾN TÍNH (Dễ hội tụ hơn bản cũ) ---
                    next_state = self.get_observation()
                    # Lấy Pitch/Roll từ state mới
                    tilt_mag = np.sqrt(next_state[0]**2 + next_state[1]**2)
                    
                    # Thưởng tối đa 1.0 khi đứng thẳng, phạt dần khi nghiêng
                    reward = max(0, 1.0 - (tilt_mag / 0.5)) + 0.1 
                    
                    # Ngã ~40 độ hoặc robot lún sàn quá sâu
                    done = (tilt_mag > 0.7) or (self.data.qpos[2] < 0.12)
                    if done: reward = -50.0 
                    
                    self.ppo_agent.buffer_rewards.append(reward)
                    self.ppo_agent.buffer_is_terminals.append(done)
                    
                    state = next_state
                    ep_reward += reward
                    total_steps += 1
                    
                    # Cập nhật hình ảnh MuJoCo mỗi 15 frame
                    if t % 15 == 0: v.sync()
                    
                    # Update mạng Neural sau mỗi 2000 steps
                    if total_steps % 2000 == 0: self.ppo_agent.update()
                    if done: break
                
                # --- LOGGING VÀ SAVE MODEL ---
                if episode % 10 == 0:
                    print(f"Ep {episode} | Reward: {ep_reward:.2f} | Std: {self.ppo_agent.action_std:.3f}")
                    self.ppo_agent.save("../best_mujoco.pt")
                
                if episode % 2000 == 0:
                    self.ppo_agent.save(f"../checkpoints/model_ep_{episode}.pt")
                    
                # Giảm nhiễu (Decay) theo tiến trình
                if episode > 0 and episode % 50 == 0: 
                    self.ppo_agent.decay_action_std(0.002, 0.05)

if __name__ == "__main__":
    MuJoCoTrainingNode().train()
import mujoco
import numpy as np
import torch
import os
import csv
from uvc_math import RobotMath
from rl_ppo_training import PPOAgent

class MuJoCoTurboNode:
    def __init__(self):
        # --- MÔI TRƯỜNG ---
        self.model = mujoco.MjModel.from_xml_path("../assets/Robot.xml")
        self.data = mujoco.MjData(self.model)
        self.math = RobotMath()
        
        # --- PPO SETUP (6 State, 7 Action) ---
        self.state_dim, self.action_dim = 6, 7
        self.ppo_agent = PPOAgent(self.state_dim, self.action_dim, 0.0001, 0.0005, 0.99, 10, 0.2)
        
        # --- RANGE THAM SỐ MG996R ---
        self.param_mins = np.array([0.01, 0.01, 40.0, 4.0, 10.0, 1.0, 0.001])
        self.param_maxs = np.array([0.50, 0.50, 100.0, 10.0, 40.0, 20.0, 0.25])
        
        # --- LOGGING SETUP ---
        self.log_file = "../logs/train_history.csv"
        if not os.path.exists("../logs"): os.makedirs("../logs")
        with open(self.log_file, "w") as f:
            writer = csv.writer(f)
            writer.writerow(["Episode", "Reward", "Steps", "Action_Std"])

    def get_observation(self):
        """Trích xuất IMU và chuẩn hóa"""
        q = self.data.sensor('quat').data
        sinp = 2 * (q[0] * q[2] - q[3] * q[1])
        pitch = np.arcsin(np.clip(sinp, -1, 1))
        roll = np.arctan2(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1]**2 + q[2]**2))
        
        obs = np.array([pitch * 180/np.pi / 45.0, roll * 180/np.pi / 45.0, self.math.fwct / 100.0])
        return np.concatenate([obs, [0, 0, 0]]).astype(np.float32) # [State + Delta]

    def train_overnight(self):
        print("🌙 TURBO MODE STARTED. No GUI, maximum speed...")
        total_steps = 0
        
        for episode in range(10000):
            mujoco.mj_resetData(self.model, self.data)
            self.data.qpos[2] = 0.26 # Hạ thấp robot
            self.math.__init__()
            
            state = self.get_observation()
            ep_reward = 0
            
            for t in range(5000):
                # 1. Action & UVC
                action = self.ppo_agent.select_action(state)
                phys_act = self.param_mins + (action + 1.0) * 0.5 * (self.param_maxs - self.param_mins)
                
                joint_angles = self.math.compute_joints(self.get_observation()[0], self.get_observation()[1], phys_act)
                
                # 2. Step mô phỏng (Không render giúp CPU chạy cực nhanh)
                self.data.ctrl[:10] = joint_angles
                mujoco.mj_step(self.model, self.data)
                self.math.update_cycle(phys_act[2])
                
                # 3. Reward & Done
                next_state = self.get_observation()
                tilt = np.sqrt(next_state[0]**2 + next_state[1]**2)
                reward = np.exp(-(tilt*45/15.0)**2) + 0.1
                done = tilt * 45 > 40.0
                
                self.ppo_agent.buffer_rewards.append(reward)
                self.ppo_agent.buffer_is_terminals.append(done)
                
                state = next_state
                ep_reward += reward
                total_steps += 1
                
                if total_steps % 2000 == 0: self.ppo_agent.update()
                if done: break
            
            # Lưu Log
            with open(self.log_file, "a") as f:
                csv.writer(f).writerow([episode, round(ep_reward, 2), t, self.ppo_agent.action_std])
            
            if episode % 10 == 0:
                print(f"Ep {episode} | Reward: {ep_reward:.2f} | Std: {self.ppo_agent.action_std}")
                self.ppo_agent.save("../best_mujoco.pt")
                
            if episode % 100 == 0: self.ppo_agent.decay_action_std(0.001, 0.05)

if __name__ == "__main__":
    MuJoCoTurboNode().train_overnight()
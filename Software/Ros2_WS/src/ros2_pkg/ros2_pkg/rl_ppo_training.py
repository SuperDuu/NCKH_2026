#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool, Float64
from geometry_msgs.msg import Vector3
import numpy as np
import torch
import torch.nn as nn
from torch.distributions import Normal
import threading
import time
import subprocess
import signal
import os

# ==============================================================================
# 1. MẠNG PPO (Actor - Critic) - GIỮ NGUYÊN CẤU TRÚC CŨ
# ==============================================================================
class ActorCritic(nn.Module):
    def __init__(self, state_dim, action_dim, action_std_init=0.6):
        super(ActorCritic, self).__init__()
        self.action_dim = action_dim
        self.action_var = torch.full((action_dim,), action_std_init * action_std_init)
        
        # ACTOR
        self.actor = nn.Sequential(
            nn.Linear(state_dim, 256),
            nn.Tanh(),
            nn.Linear(256, 256),
            nn.Tanh(),
            nn.Linear(256, action_dim),
            nn.Tanh()
        )
        
        # CRITIC
        self.critic = nn.Sequential(
            nn.Linear(state_dim, 256),
            nn.Tanh(),
            nn.Linear(256, 256),
            nn.Tanh(),
            nn.Linear(256, 1)
        )

    def set_action_std(self, new_action_std):
        self.action_var = torch.full((self.action_dim,), new_action_std * new_action_std)

    def act(self, state):
        action_mean = self.actor(state)
        # Chặn NaN
        if torch.isnan(action_mean).any():
            action_mean = torch.nan_to_num(action_mean, nan=0.0)
            
        cov_mat = torch.diag(self.action_var).unsqueeze(dim=0)
        dist = Normal(action_mean, torch.sqrt(self.action_var))
        
        action = dist.sample()
        action_logprob = dist.log_prob(action).sum(dim=-1)
        state_val = self.critic(state)

        return action.detach(), action_logprob.detach(), state_val.detach()
    
    def evaluate(self, state, action):
        action_mean = self.actor(state)
        action_var = self.action_var.expand_as(action_mean)
        dist = Normal(action_mean, torch.sqrt(action_var))
        
        action_logprobs = dist.log_prob(action).sum(dim=-1)
        dist_entropy = dist.entropy().sum(dim=-1)
        state_values = self.critic(state)
        
        return action_logprobs, state_values, dist_entropy

# ==============================================================================
# 2. PPO AGENT (CÓ THÊM HÀM DECAY)
# ==============================================================================
class PPOAgent:
    def __init__(self, state_dim, action_dim, lr_actor, lr_critic, gamma, K_epochs, eps_clip):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        print(f"✅ PPOAgent is using: {self.device}", flush=True)
        self.gamma = gamma
        self.eps_clip = eps_clip
        self.K_epochs = K_epochs
        
        # Lưu giá trị hiện tại để decay dần
        self.action_std = 0.6 
        
        self.buffer_states = []
        self.buffer_actions = []
        self.buffer_logprobs = []
        self.buffer_rewards = []
        self.buffer_is_terminals = []
        
        self.policy = ActorCritic(state_dim, action_dim).float()
        self.optimizer = torch.optim.Adam([
                        {'params': self.policy.actor.parameters(), 'lr': lr_actor},
                        {'params': self.policy.critic.parameters(), 'lr': lr_critic}
                    ])
        
        self.policy_old = ActorCritic(state_dim, action_dim).float()
        self.policy_old.load_state_dict(self.policy.state_dict())
        self.MseLoss = nn.MSELoss()

    # [MỚI] Hàm thiết lập độ rung thủ công
    def set_action_std(self, new_action_std):
        self.action_std = new_action_std
        self.policy.set_action_std(new_action_std)
        self.policy_old.set_action_std(new_action_std)
        print(f"🔧 Manually set Action Std to: {self.action_std}", flush=True)

    # [MỚI] Hàm giảm dần độ rung
    def decay_action_std(self, action_std_decay_rate, min_action_std):
        self.action_std = self.action_std - action_std_decay_rate
        self.action_std = round(self.action_std, 4)
        
        if (self.action_std <= min_action_std):
            self.action_std = min_action_std
            
        print(f"📉 Action Std decayed to: {self.action_std}", flush=True)
        self.policy.set_action_std(self.action_std)
        self.policy_old.set_action_std(self.action_std)

    def select_action(self, state):
        with torch.no_grad():
            state = torch.FloatTensor(state)
            action, action_logprob, _ = self.policy_old.act(state)
            
        self.buffer_states.append(state)
        self.buffer_actions.append(action)
        self.buffer_logprobs.append(action_logprob)
        return action.detach().cpu().numpy().flatten()

    def update(self):
        rewards = []
        discounted_reward = 0
        for reward, is_terminal in zip(reversed(self.buffer_rewards), reversed(self.buffer_is_terminals)):
            if is_terminal: discounted_reward = 0
            discounted_reward = reward + (self.gamma * discounted_reward)
            rewards.insert(0, discounted_reward)
            
        rewards = torch.tensor(rewards, dtype=torch.float32)
        rewards = (rewards - rewards.mean()) / (rewards.std() + 1e-7)

        # old_states = torch.squeeze(torch.stack(self.buffer_states, dim=0)).detach()
        # old_actions = torch.squeeze(torch.stack(self.buffer_actions, dim=0)).detach()
        # old_logprobs = torch.squeeze(torch.stack(self.buffer_logprobs, dim=0)).detach()
        old_states = torch.squeeze(torch.stack(self.buffer_states, dim=0)).to(self.device).detach()
        old_actions = torch.squeeze(torch.stack(self.buffer_actions, dim=0)).to(self.device).detach()
        old_logprobs = torch.squeeze(torch.stack(self.buffer_logprobs, dim=0)).to(self.device).detach()

        for _ in range(self.K_epochs):
            logprobs, state_values, dist_entropy = self.policy.evaluate(old_states, old_actions)
            state_values = torch.squeeze(state_values)
            ratios = torch.exp(logprobs - old_logprobs)
            advantages = rewards - state_values.detach()
            surr1 = ratios * advantages
            surr2 = torch.clamp(ratios, 1-self.eps_clip, 1+self.eps_clip) * advantages
            loss = -torch.min(surr1, surr2) + 0.5 * self.MseLoss(state_values, rewards) - 0.01 * dist_entropy
            
            self.optimizer.zero_grad()
            loss.mean().backward()
            torch.nn.utils.clip_grad_norm_(self.policy.parameters(), 0.5) 
            self.optimizer.step()
            
        self.policy_old.load_state_dict(self.policy.state_dict())
        del self.buffer_states[:]
        del self.buffer_actions[:]
        del self.buffer_logprobs[:]
        del self.buffer_rewards[:]
        del self.buffer_is_terminals[:]

    def save(self, checkpoint_path):
        torch.save(self.policy_old.state_dict(), checkpoint_path)
    
    def load(self, checkpoint_path):
        self.policy_old.load_state_dict(torch.load(checkpoint_path))
        self.policy.load_state_dict(torch.load(checkpoint_path))

# ==============================================================================
# 3. NODE HUẤN LUYỆN
# ==============================================================================
class RLTrainingNode(Node):
    def __init__(self):
        super().__init__('rl_training_node')
        
        # --- PATH SETUP ---
        user_home = os.path.expanduser("~")
        self.weights_dir = os.path.join(user_home, "ros2_weights")
        if not os.path.exists(self.weights_dir):
            os.makedirs(self.weights_dir, exist_ok=True)

        # --- ROS COMM ---
        self.param_pub = self.create_publisher(Float64MultiArray, '/uvc_parameters', 10)
        self.feedback_sub = self.create_subscription(Vector3, '/uvc_rl_feedback', self.feedback_callback, 10)
        self.reset_pub = self.create_publisher(Bool, '/uvc_reset', 10)

        # Joint Publishers (Để reset khớp về 0)
        self.joint_pubs = {}
        joint_names = [
            'base_hip_left_joint', 'hip_hip_left_joint', 'hip_knee_left_joint', 'knee_ankle_left_joint', 'ankle_ankle_left_joint',
            'base_hip_right_joint', 'hip_hip_right_joint', 'hip_knee_right_joint', 'knee_ankle_right_joint', 'ankle_ankle_right_joint',
            'hip_shoulder_left_joint', 'shoulder_shoulder_left_joint', 'shoulder_elbow_left_joint',
            'hip_shoulder_right_joint', 'shoulder_shoulder_right_joint', 'shoulder_elbow_right_joint',
            'base_hip_middle_joint',
        ]
        for j in joint_names:
            topic = f'/model/humanoid_robot/joint/{j}/cmd_pos'
            self.joint_pubs[j] = self.create_publisher(Float64, topic, 10)

        # --- VARIABLES ---
        self.current_obs = np.array([0.0, 0.0, 0.0]) 
        self.prev_obs = np.array([0.0, 0.0, 0.0]) 
        self.data_received = False
        self.is_falling = False
        self.reset_count = 0
        self.prev_phys_act = None # Cho Action Smoothing

        # --- PPO SETUP ---
        self.state_dim = 6 # [Pitch, Roll, Mag, dPitch, dRoll, dMag]
        self.action_dim = 7
        self.ppo_agent = PPOAgent(self.state_dim, self.action_dim, 0.0001, 0.0005, 0.99, 10, 0.2)
        
        self.best_reward = -float('inf')
        self.update_timestep = 2000
        self.time_step = 0

        # --- PARAMS RANGE (Đã tối ưu cho MG996R - GIỮ NGUYÊN CỦA BẠN) ---
        self.param_mins = np.array([0.01, 0.01, 40.0, 4.0, 10.0, 1.0, 0.001])
        self.param_maxs = np.array([0.50, 0.50, 100.0, 10.0, 40.0, 20.0, 0.25])

        # --- THREAD ---
        self.running = True
        self.train_thread = threading.Thread(target=self.train_loop, daemon=True)
        self.train_thread.start()

    def feedback_callback(self, msg):
        self.prev_obs = self.current_obs.copy()
        self.current_obs = np.array([msg.x, msg.y, msg.z])
        self.data_received = True 
        if abs(msg.x) > 45.0 or abs(msg.y) > 45.0:
            self.is_falling = True

    def get_state_vector(self):
        delta = self.current_obs - self.prev_obs
        norm_obs = self.current_obs / np.array([45.0, 45.0, 1.0])
        norm_delta = delta / np.array([5.0, 5.0, 0.1]) 
        return np.concatenate([norm_obs, norm_delta], axis=0).astype(np.float32)

    def scale_action(self, action):
        # Scale về giá trị thực
        phys_act = self.param_mins + (action + 1.0) * 0.5 * (self.param_maxs - self.param_mins)
        # Action Smoothing (Quan trọng cho Servo)
        alpha = 0.3 
        if self.prev_phys_act is None: self.prev_phys_act = phys_act
        smoothed = alpha * phys_act + (1 - alpha) * self.prev_phys_act
        self.prev_phys_act = smoothed
        return smoothed

    # --- SUBPROCESS HELPERS ---
    def run_gz_command(self, service_name, req_type, rep_type, req_data, timeout=3.0, max_retries=3):
        for attempt in range(max_retries):
            try:
                cmd = ['gz', 'service', '-s', service_name, '--reqtype', req_type, '--reptype', rep_type, '--timeout', '2000', '--req', req_data]
                process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, preexec_fn=lambda: signal.signal(signal.SIGPIPE, signal.SIG_DFL))
                try:
                    stdout, stderr = process.communicate(timeout=timeout)
                    if process.returncode == 0: return True
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
                time.sleep(0.2)
            except: time.sleep(0.2)
        return False

    def pause_simulation(self):
        return self.run_gz_command('/world/empty/control', 'gz.msgs.WorldControl', 'gz.msgs.Boolean', 'pause: true')

    def unpause_simulation(self):
        return self.run_gz_command('/world/empty/control', 'gz.msgs.WorldControl', 'gz.msgs.Boolean', 'pause: false')

    def reset_robot_pose(self):
        # [FIX] Nâng độ cao lên 0.26 và dốc người để tránh kẹt đất/ngã ngửa
        pose_msg = 'name: "humanoid_robot" position { x: 0.0 y: 0.0 z: 0.28 } orientation { x: 0.0 y: 0.0 z: 0.0 w: 0.999 }'
        return self.run_gz_command('/world/empty/set_pose', 'gz.msgs.Pose', 'gz.msgs.Boolean', pose_msg, timeout=4.0)

    def reset_all_joints(self):
        msg = Float64()
        msg.data = 0.0
        for _ in range(5):
            for pub in self.joint_pubs.values(): pub.publish(msg)
            time.sleep(0.01)

    # --- RESET LOGIC (SIÊU AN TOÀN - CHỐNG NGÃ OAN) ---
    # def reset_simulation(self):
    #     max_retries = 5
    #     for attempt in range(max_retries):
    #         self.reset_count += 1
            
    #         # 1. Báo UVC chuẩn bị
    #         self.reset_pub.publish(Bool(data=True))
    #         time.sleep(0.05) 
            
    #         # 2. Pause Vật lý ngay
    #         self.pause_simulation()
            
    #         # 3. Reset Pose & Khớp khi đang Pause
    #         self.reset_robot_pose()
    #         self.reset_all_joints()
            
    #         # 4. Mở lại vật lý
    #         self.unpause_simulation()
            
    #         # 5. Chờ rơi xuống và ổn định (1.2s là đủ)
    #         time.sleep(1.2)
            
    #         # 6. Kiểm tra nếu robot nghiêng quá mức ngay khi vừa sinh ra
    #         # Nới lỏng check lên 45 độ
    #         # if abs(self.current_obs[0]) < 45.0 and abs(self.current_obs[1]) < 45.0:
    #         #     break # Ổn, thoát vòng lặp
    #         # else:
        #     #     print(f"⚠️ Bad Reset detected (Attempt {attempt+1}). Retrying...", flush=True)

        # # 7. Bắt đầu Training
        # self.reset_pub.publish(Bool(data=False))
        # self.is_falling = False
        # self.data_received = False 
        # self.prev_obs = np.zeros(3)
        # self.prev_phys_act = None
        # --- RESET LOGIC (PHIÊN BẢN ĐƠN GIẢN - KHÔNG CHECK) ---
    def reset_simulation(self):
        self.reset_count += 1
        
        # 1. Báo UVC chuẩn bị
        self.reset_pub.publish(Bool(data=True))
        time.sleep(0.5) 
        # 2. Pause & Reset Pose
        self.pause_simulation()
        time.sleep(0.2) 
        self.reset_robot_pose()
        self.reset_all_joints()
        time.sleep(0.2) 
        # 3. Unpause
        self.unpause_simulation()
        
        # 4. Chờ ngắn (0.5s là đủ để rơi xuống sàn)
        time.sleep(0.2) 
        
        # 5. Bắt đầu luôn (Không thèm check xem có ngã không)
        self.reset_pub.publish(Bool(data=False))
        self.is_falling = False
        self.data_received = False 
        self.prev_obs = np.zeros(3)
        self.prev_phys_act = None

    # --- TRAINING LOOP (FINE-TUNING MODE CHO MG996R) ---
    def train_loop(self):
        time.sleep(2)
        print("🚀 FINE-TUNING MODE STARTED (Aiming for Stability with MG996R)", flush=True)
        
        # 1. LOAD MODEL CŨ & GIẢM RUNG NGAY LẬP TỨC
        best_path = os.path.join(self.weights_dir, "best.pt")
        current_std = 0.6 # Mặc định nếu chưa có file

        if os.path.exists(best_path):
            try: 
                self.ppo_agent.load(best_path)
                print("🔄 Loaded existing best weights! Switching to FINE-TUNING.")
                # [QUAN TRỌNG] Bắt đầu tinh chỉnh ở mức 0.3 thay vì 0.6
                current_std = 0.25
                self.ppo_agent.set_action_std(current_std)
            except: 
                print("⚠️ Load failed, starting fresh (std=0.6)")
        else:
            print("⚠️ No weights found, starting fresh.")

        for episode in range(10000):
            if not self.running: break
            self.reset_simulation()
            
            # Chờ data đầu tiên
            wait_start = time.time()
            while not self.data_received and self.running:
                if time.time() - wait_start > 2.0: break
                time.sleep(0.01)
            
            state = self.get_state_vector()
            ep_reward = 0
            
            # [NÂNG CẤP] Tăng max steps lên 6000 (60s) để test độ bền
            for t in range(100000):
                if not self.running: break
                
                # Busy wait (Turbo Mode)
                wait_s = time.time()
                while not self.data_received:
                    if time.time() - wait_s > 0.2: break
                if not self.data_received: continue
                self.data_received = False

                # 1. Action
                action = self.ppo_agent.select_action(state)
                phys_act = self.scale_action(action)
                
                msg = Float64MultiArray()
                msg.data = list(phys_act)
                self.param_pub.publish(msg)

                # 2. Reward (Ưu tiên ổn định)
                next_state = self.get_state_vector()
                pitch, roll = self.current_obs[0], self.current_obs[1]
                tilt_mag = np.sqrt(pitch**2 + roll**2)
                
                # Reward: Đứng càng thẳng điểm càng cao
                reward = np.exp(-(tilt_mag/15.0)**2)
                
                # Bonus sống sót
                reward += 0.1 

                done = False
                if self.is_falling or tilt_mag > 40.0:
                    reward = -5.0
                    done = True
                
                # 3. Store & Update
                self.ppo_agent.buffer_rewards.append(reward)
                self.ppo_agent.buffer_is_terminals.append(done)
                
                state = next_state
                ep_reward += reward
                self.time_step += 1
                
                if self.time_step % self.update_timestep == 0:
                    self.ppo_agent.update()

                if done: break
            
            print(f"Ep {episode} | Reward: {ep_reward:.2f} | Steps: {t} | Best: {self.best_reward}", flush=True)
            
            # [CHIẾN THUẬT DECAY] 
            # Giảm dần độ rung để ép mạng học cách bù sai số cơ khí (Lún tải)
            if episode > 0 and episode % 25 == 0:
                # Giảm mỗi lần 0.02, đáy là 0.05
                current_std = max(0.05, current_std - 0.001)
                # self.ppo_agent.decay_action_std(0.0, current_std) # Set trực tiếp giá trị mới
                self.ppo_agent.set_action_std(current_std)
                print(f"action_std: {current_std}", flush=True)

            if ep_reward > self.best_reward:
                self.best_reward = ep_reward
                self.ppo_agent.save(best_path)
                print(f"🌟 NEW BEST! ({t/100:.1f}s) Saved.", flush=True)

def main():
    rclpy.init()
    node = RLTrainingNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.running = False
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
# RL Training Guide - Tối Ưu Hóa Tham Số UVC Robot

## Mục Đích
Sử dụng học tăng cường (Reinforcement Learning) để robot tự động tìm các tham số điều khiển UVC tối ưu sao cho robot có thể cân bằng tốt nhất khi bị tilt/push.

## Kiến Trúc Hệ Thống

```
┌─────────────────────────────────────────────────────────┐
│  RL Training Node (Python)                              │
│  - Policy Network: State(3) → Action(7)                 │
│  - Reward: stability_score - fall_penalty               │
│  - Publish: /uvc_parameters [gain, scale_base, ...]    │
└──────────────────────┬──────────────────────────────────┘
                       │ Float64MultiArray
                       ▼
┌─────────────────────────────────────────────────────────┐
│  UVC Controller (C++)                                   │
│  - Subscribe: /uvc_parameters                           │
│  - Update: gain, scale_base, step_duration, ...         │
│  - Publish: /robot_orientation (IMU angles)             │
│  - Control: Joint positions at 20Hz                     │
└──────────────────────┬──────────────────────────────────┘
                       │ Vector3 (pitch, roll, tilt_mag)
                       ▼
┌─────────────────────────────────────────────────────────┐
│  Gazebo/Ignition Simulator                              │
│  - Physics: Humanoid robot balance                      │
│  - Feedback: IMU sensor data                            │
└─────────────────────────────────────────────────────────┘
```

## ACTION SPACE (7 Tham Số)

| Index | Tham Số | Min | Max | Mô Tả |
|-------|---------|-----|-----|-------|
| 0 | **gain** | 0.10 | 0.30 | Độ lợi toàn cục, kiểm soát phản ứng tổng quát |
| 1 | **scale_base** | 0.10 | 0.20 | Hệ số scale hình học (0.1 = 10%, 0.2 = 20%) |
| 2 | **step_duration** | 20.0 | 35.0 | Độ dài bước chân (cycle, mỗi cycle = 50ms) |
| 3 | **landing_phase** | 5.0 | 8.0 | Số cycle để hạ chân từ độ cao sang sàn |
| 4 | **stance_width** | 30.0 | 45.0 | Khoảng cách giữa 2 chân (mm) |
| 5 | **max_foot_lift** | 8.0 | 15.0 | Độ cao nâng chân khi di chuyển (mm) |
| 6 | **recovery_rate** | 0.05 | 0.20 | Tốc độ phục hồi sau cách động |

## STATE SPACE (3 Biến Trạng Thái)

```
state = [pitch, roll, tilt_magnitude]
```
- **pitch**: Góc quay trước-sau (độ)
- **roll**: Góc quay trái-phải (độ)
- **tilt_magnitude**: sqrt(pitch² + roll²) - độ lớn tilt tổng hợp

## REWARD FUNCTION

```python
stability_score = 1.0 / (1.0 + 0.05 * tilt_mag)  # [0, 1]
fall_penalty = 1.0 if tilt_mag > 45.0 else 0.0  # Detect fall
reward = stability_score - 2.0 * fall_penalty
```

**Giải thích:**
- Tilt càng nhỏ → reward càng cao (→ 1.0)
- Tilt càng lớn → reward càng thấp (→ 0)
- **Nếu robot ngã (tilt > 45°)** → reward = -1.0 (penalty)

## Cách Chạy

### 1. Terminal 1: Chạy Gazebo Simulation
```bash
cd ~/Desktop/NCKH_2026/Software/Ros2_WS
source install/setup.bash
ros2 launch ros2_pkg humanoid_balance.launch.py
```

### 2. Terminal 2: Chạy RL Training
```bash
cd ~/Desktop/NCKH_2026/Software/Ros2_WS
source install/setup.bash
./run_rl_training.sh
# Hoặc trực tiếp:
ros2 run ros2_pkg rl_training_node
```

## Output Monitoring

### RL Training Node Output
```
Ep    0 Step   0 | Gain: 0.180 | Scale: 0.121 | StepDur: 25.0 | Tilt: 0.5° | Reward: 0.971
Ep    0 Step  50 | Gain: 0.185 | Scale: 0.119 | StepDur: 24.8 | Tilt: 2.3° | Reward: 0.879
...
🎉 NEW BEST! Episode 45 | Reward: 42.35 | AvgTilt: 1.2° | MaxTilt: 5.3° | BestParams: G=0.190 S=0.128 SD=23.5 LP=6.2

Checkpoint saved at episode 100
  Best Params: Gain=0.190, Scale=0.128, StepDur=23.5, LandingPhase=6.2
```

### UVC Controller Output
```
[RL UPDATE] gain=0.190 scale=0.128 step_dur=23.5 land_phase=6.2 stance=35.0 fh_max=12.0 rr=0.100
[IMU RAW] pitch=-0.52° roll=1.24°
[AFTER THRESHOLD] pitch=-0.48° roll=1.18° | tilt_mag=1.25°
[GEOM-Y] roll=0.021 rad | scale_factor=0.128 | roll_scaled=0.0027 | ks=-0.0025 | dyi_new=-0.5
```

## Các Giai Đoạn Training

### Phase 1: Early Learning (Episodes 0-100)
- Robot học các hành động cơ bản
- Reward dao động lớn
- Tilt trung bình: 5-15°

### Phase 2: Improvement (Episodes 100-500)
- Policy ổn định hơn
- Reward tăng dần
- Tilt trung bình: 2-5°

### Phase 3: Fine-tuning (Episodes 500+)
- Hội tụ đến local optimum
- Reward ổn định ở cao
- Tilt trung bình: < 2°

## Khi Nào Dừng Training?

1. **Reward Plateau**: Reward không tăng trong 50-100 episode → RL đã hội tụ
2. **Good Performance**: AvgTilt < 2° trong 20 episode liên tiếp
3. **Time Limit**: Sau 1000-2000 episode là đủ tốt

## Sau Khi Training

### 1. Lấy Best Parameters
Xem log cuối cùng:
```
Best Params: Gain=0.195, Scale=0.130, StepDur=22.5, LandingPhase=6.5
```

### 2. Update vào config
Cập nhật trong `launch/humanoid_balance.launch.py`:
```python
'gain': 0.195,
'scale_base': 0.130,
'step_duration': 22.5,
'landing_phase': 6.5,
```

### 3. Test Performance
Chạy robot với parameters mới, quan sát:
- Tilt angles trong simulation
- Foot response timing
- Balance stability

## Troubleshooting

### 1. Robot ngã liên tục
- **Nguyên nhân**: Learning rate quá cao hoặc action bounds sai
- **Cách sửa**: Giảm learning rate (0.001 → 0.0005) hoặc điều chỉnh action bounds

### 2. Reward không tăng
- **Nguyên nhân**: Reward function không phù hợp
- **Cách sửa**: Thay đổi hệ số: `0.05 * tilt_mag` → `0.1 * tilt_mag`

### 3. Robot chậm
- **Nguyên nhân**: step_duration quá dài
- **Cách sửa**: Giảm max value: 35.0 → 30.0

## Hyperparameters Có Thể Điều Chỉnh (RL node)

```python
# Learning rate
self.optimizer = optim.Adam(self.policy.parameters(), lr=0.001)

# Reward coefficients
stability_score = 1.0 / (1.0 + 0.05 * tilt_mag)  # Tăng 0.05 → 0.1 để strict hơn
fall_penalty = 1.0 if tilt_mag > 45.0 else 0.0   # Giảm 45.0 → 30.0 để strict hơn

# Episode length
for step in range(500):  # Tăng lên 1000 để test lâu hơn
```

## References

- **RL Algorithm**: Policy Gradient (REINFORCE variant)
- **Network**: 2-layer MLP (256 units each)
- **Distribution**: Gaussian (Normal distribution for continuous actions)
- **Loss**: Cross-entropy loss × reward

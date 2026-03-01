import torch
import torch.nn as nn
import numpy as np

# ==========================================
# 1. ĐỊNH NGHĨA LẠI MẠNG ACTOR (v2 — 4 Trụ Cột)
# (Phải giống hệt cấu trúc lúc train)
# ==========================================

class SteFloor(torch.autograd.Function):
    """STE Floor — chỉ cần định nghĩa lại cho load_state_dict."""
    @staticmethod
    def forward(ctx, x):
        return x.floor()
    @staticmethod
    def backward(ctx, grad):
        return grad

class InputDiscretizer(nn.Module):
    """Ép kiểu số nguyên roll/pitch. Scale = 57.29 (1°)."""
    def __init__(self, scale=57.29):
        super().__init__()
        self.scale = scale
    def forward(self, x):
        return SteFloor.apply(x * self.scale) / self.scale


class ActorForExport(nn.Module):
    """Actor-only cho ONNX export (không cần Sigma Head / Critic).
    Bao gồm InputDiscretizer + Shared Extractor + Mean Head."""
    def __init__(self, state_dim, action_dim):
        super().__init__()
        self.imu_discretizer = InputDiscretizer(scale=57.29)
        self.actor_extractor = nn.Sequential(
            nn.Linear(state_dim, 512),
            nn.ELU(),
            nn.GroupNorm(32, 512),
            nn.Linear(512, 256),
            nn.ELU(),
            nn.Dropout(0.1),
        )
        self.mean_head = nn.Sequential(
            nn.Linear(256, 128),
            nn.ELU(),
            nn.Linear(128, action_dim),
            nn.Tanh(),
        )

    def forward(self, state):
        s = state.clone()
        s[:, :4] = self.imu_discretizer(s[:, :4])
        features = self.actor_extractor(s)
        return self.mean_head(features)


# ==========================================
# 2. CẤU HÌNH THÔNG SỐ
# ==========================================
state_dim = 18
action_dim = 6

input_path = "/home/du/Desktop/NCKH_2026/Software/Ros2_WS_v2/src/ros2_pkg_v2/weights/reactive_balance/best_model.pth"
output_path = "/home/du/Desktop/NCKH_2026/Software/Ros2_WS_v2/src/ros2_pkg_v2/weights/reactive_balance/robot_brain_v2.onnx"


# ==========================================
# 3. QUY TRÌNH CHUYỂN ĐỔI
# ==========================================
def convert():
    print(f"🔄 Đang đọc file: {input_path}...")

    model = ActorForExport(state_dim, action_dim)

    try:
        checkpoint = torch.load(input_path, map_location=torch.device('cpu'))
        actor_weights = {}
        for key, value in checkpoint.items():
            # Chỉ lấy: imu_discretizer.*, actor_extractor.*, mean_head.*
            if any(prefix in key for prefix in ['imu_discretizer', 'actor_extractor', 'mean_head']):
                new_key = key
                # Xoá prefix policy nếu có
                for prefix in ['policy_old.', 'policy.']:
                    if new_key.startswith(prefix):
                        new_key = new_key[len(prefix):]
                actor_weights[new_key] = value

        if len(actor_weights) > 0:
            model.load_state_dict(actor_weights)
            print("✅ Đã trích xuất và nạp trọng số Actor v2 thành công!")
        else:
            model.load_state_dict(checkpoint)
            print("✅ Đã nạp trực tiếp state dict!")

    except Exception as e:
        print(f"❌ Lỗi khi load file .pt: {e}")
        return

    model.eval()
    dummy_input = torch.randn(1, state_dim)

    print("🔄 Đang xuất file ONNX...")
    torch.onnx.export(
        model,
        dummy_input,
        output_path,
        export_params=True,
        opset_version=13,
        do_constant_folding=True,
        input_names=['input_state'],
        output_names=['output_action']
    )

    print(f"🚀 XONG! File đã lưu tại: {output_path}")
    print("👉 Bước tiếp theo: Nạp file này vào STM32CubeMX (X-CUBE-AI).")


if __name__ == '__main__':
    convert()
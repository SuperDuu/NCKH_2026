import torch
import torch.nn as nn
import numpy as np
import os

# ==========================================
# 1. ĐỊNH NGHĨA LẠI MẠNG ACTOR (Phase 3: 126D)
# (Phải giống hệt cấu trúc Actor trong cpg_phase_1.py)
# ==========================================

class ActorForExport(nn.Module):
    """Actor-only cho ONNX export (không cần Sigma Head / Critic)."""
    def __init__(self, state_dim, action_dim):
        super().__init__()
        self.actor = nn.Sequential(
            nn.Linear(state_dim, 256), nn.ELU(),
            nn.Linear(256, 256), nn.ELU(),
            nn.Linear(256, action_dim), nn.Tanh())

    def forward(self, state):
        return self.actor(state)


# ==========================================
# 2. CẤU HÌNH THÔNG SỐ
# ==========================================
state_dim = 126
action_dim = 6

# Đường dẫn bộ trọng số mới cho Phase 3 (Stepping Balance)
weight_dir = "/home/du/Desktop/NCKH_2026/Software/Ros2_WS_v2/src/ros2_pkg_v2/ros2_pkg/weights/balancing_step_ppo"
input_path = os.path.join(weight_dir, "best_model.pth")
output_path = os.path.join(weight_dir, "model.onnx")


# ==========================================
# 3. QUY TRÌNH CHUYỂN ĐỔI
# ==========================================
def convert():
    if not os.path.exists(input_path):
        print(f"❌ Không tìm thấy file: {input_path}")
        return

    print(f"🔄 Đang đọc file: {input_path}...")

    model = ActorForExport(state_dim, action_dim)

    try:
        checkpoint = torch.load(input_path, map_location=torch.device('cpu'))
        actor_weights = {}
        
        # Trích xuất trọng số của component "actor" từ checkpoint PPO
        # Checkpoint thường chứa { 'actor.0.weight', 'actor.0.bias', ... }
        # hoặc { 'policy.actor.0.weight', ... }
        
        for key, value in checkpoint.items():
            new_key = key
            # Xoá prefix 'policy.' hoặc 'policy_old.'
            for prefix in ['policy_old.', 'policy.']:
                if new_key.startswith(prefix):
                    new_key = new_key[len(prefix):]
            
            # Chỉ lấy các key bắt đầu bằng 'actor.'
            if new_key.startswith('actor.'):
                actor_weights[new_key] = value

        if len(actor_weights) > 0:
            model.load_state_dict(actor_weights)
            print("✅ Đã trích xuất và nạp trọng số Actor (Phase 3) thành công!")
        else:
            # Thử nạp trực tiếp nếu checkpoint đã là actor state dict
            try:
                model.load_state_dict(checkpoint)
                print("✅ Đã nạp trực tiếp state dict!")
            except:
                print("❌ Không tìm thấy trọng số 'actor' phù hợp trong checkpoint.")
                return

    except Exception as e:
        print(f"❌ Lỗi khi load file .pt: {e}")
        return

    model.eval()
    # Dummy input với shape (batch_size, state_dim)
    dummy_input = torch.randn(1, state_dim)

    print("🔄 Đang xuất file ONNX...")
    try:
        torch.onnx.export(
            model,
            dummy_input,
            output_path,
            export_params=True,
            opset_version=17,
            do_constant_folding=True,
            input_names=['input_state'],
            output_names=['output_action']
        )
        print(f"🚀 XONG! File đã lưu tại: {output_path}")
        print("👉 Bước tiếp theo: Sử dụng file ONNX này để kiểm thử hoặc nạp vào X-CUBE-AI.")
    except Exception as e:
        print(f"❌ Lỗi khi export sang ONNX: {e}")


if __name__ == '__main__':
    convert()
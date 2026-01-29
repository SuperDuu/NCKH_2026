import torch
import torch.nn as nn
import numpy as np

# ==========================================
# 1. ĐỊNH NGHĨA LẠI MẠNG ACTOR
# (Phải giống hệt cấu trúc lúc train)
# ==========================================
class Actor(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(Actor, self).__init__()
        self.actor = nn.Sequential(
            nn.Linear(state_dim, 256),
            nn.Tanh(),
            nn.Linear(256, 256),
            nn.Tanh(),
            nn.Linear(256, action_dim),
            nn.Tanh()
        )

    def forward(self, state):
        return self.actor(state)

# ==========================================
# 2. CẤU HÌNH THÔNG SỐ
# ==========================================
# Input: [Pitch, Roll, Mag, dPitch, dRoll, dMag]
state_dim = 6   
# Output: 7 Servo
action_dim = 7  

input_path = "/home/du/Documents/best.pt"
output_path = "/home/du/Documents/robot_brain.onnx"

# ==========================================
# 3. QUY TRÌNH CHUYỂN ĐỔI
# ==========================================
def convert():
    print(f"🔄 Đang đọc file: {input_path}...")
    
    # Khởi tạo mô hình rỗng
    model = Actor(state_dim, action_dim)
    
    try:
        # Load toàn bộ checkpoint (thường chứa cả Actor, Critic, Optimizer...)
        checkpoint = torch.load(input_path, map_location=torch.device('cpu'))
        
        # Dictionary mới để chứa trọng số sạch
        actor_weights = {}
        
        # LỌC TRỌNG SỐ: Chỉ lấy phần của Actor
        # File .pt của PPO thường lưu key kiểu: "actor.0.weight" hoặc "policy.actor.0.weight"
        for key, value in checkpoint.items():
            # Nếu key có chứa chữ 'actor' (hoặc nếu file chỉ lưu mỗi actor thì lấy hết)
            if "actor" in key:
                # Xóa các tiền tố thừa để khớp với model khai báo ở trên
                # Ví dụ: "policy_old.actor.0.weight" -> "actor.0.weight"
                new_key = key
                if "policy_old." in new_key:
                    new_key = new_key.replace("policy_old.", "")
                if "policy." in new_key:
                    new_key = new_key.replace("policy.", "")
                    
                # Trong class Actor ở trên, mình khai báo self.actor = nn.Sequential...
                # Nên key phải bắt đầu bằng "actor."
                if not new_key.startswith("actor."):
                     new_key = "actor." + new_key
                     
                actor_weights[new_key] = value
                
        # Nạp trọng số vào mô hình
        if len(actor_weights) > 0:
            model.load_state_dict(actor_weights)
            print("✅ Đã trích xuất và nạp trọng số Actor thành công!")
        else:
            # Trường hợp file .pt lưu trực tiếp state_dict của Actor mà không có prefix
            model.load_state_dict(checkpoint)
            print("✅ Đã nạp trực tiếp state dict!")

    except Exception as e:
        print(f"❌ Lỗi khi load file .pt: {e}")
        print("Gợi ý: Kiểm tra xem file best.pt có đúng đường dẫn không.")
        return

    # Chuyển sang chế độ lại (quan trọng để tắt Dropout/Batchnorm nếu có)
    model.eval()

    # Tạo dữ liệu giả (Dummy Input) đúng kích thước để vẽ đồ thị
    # Batch size = 1, Input = 6
    dummy_input = torch.randn(1, state_dim)

    # Xuất ra ONNX
    print("🔄 Đang xuất file ONNX...")
    torch.onnx.export(
        model,                      # Mô hình đang chạy
        dummy_input,                # Đầu vào giả
        output_path,                # Tên file xuất
        export_params=True,         # Lưu trọng số bên trong file
        opset_version=13,           # Version ổn định cho Embedded/STM32
        do_constant_folding=True,   # Tối ưu hóa các hằng số
        input_names=['input_state'],  # Đặt tên đầu vào (dễ gọi trong C code)
        output_names=['output_action'] # Đặt tên đầu ra
    )
    
    print(f"🚀 XONG! File đã lưu tại: {output_path}")
    print("👉 Bước tiếp theo: Nạp file này vào STM32CubeMX (X-CUBE-AI).")

if __name__ == "__main__":
    convert()
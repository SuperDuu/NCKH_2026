import pandas as pd
import matplotlib.pyplot as plt
import os

def plot_training_history(csv_path='training_history.csv'):
    if not os.path.exists(csv_path):
        print(f"❌ Không tìm thấy file: {csv_path}")
        return

    # 1. Đọc dữ liệu
    data = pd.read_csv(csv_path)
    
    # Ép kiểu dữ liệu để tránh lỗi định dạng
    data['Episode'] = pd.to_numeric(data['Episode'])
    data['Reward'] = pd.to_numeric(data['Reward'])
    data['Steps'] = pd.to_numeric(data['Steps'])

    # 2. Tạo khung ảnh
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=True)
    plt.subplots_adjust(hspace=0.3)

    # --- ĐỒ THỊ 1: REWARD ---
    ax1.plot(data['Episode'], data['Reward'], color='#1f77b4', alpha=0.3, label='Raw Reward')
    # Vẽ đường trung bình trượt (Moving Average) để thấy xu hướng rõ hơn
    if len(data) > 10:
        ax1.plot(data['Episode'], data['Reward'].rolling(window=10).mean(), 
                 color='red', linewidth=2, label='Trend (MA 10)')
    
    ax1.set_title('Training Reward over Episodes', fontsize=14, fontweight='bold')
    ax1.set_ylabel('Total Reward')
    ax1.grid(True, linestyle='--', alpha=0.7)
    ax1.legend()

    # --- ĐỒ THỊ 2: STEPS (SỐ BƯỚC SỐNG SÓT) ---
    ax2.plot(data['Episode'], data['Steps'], color='#2ca02c', alpha=0.3, label='Raw Steps')
    if len(data) > 10:
        ax2.plot(data['Episode'], data['Steps'].rolling(window=10).mean(), 
                 color='darkgreen', linewidth=2, label='Trend (MA 10)')
    
    ax2.set_title('Survival Steps per Episode', fontsize=14, fontweight='bold')
    ax2.set_xlabel('Episode')
    ax2.set_ylabel('Steps Count')
    ax2.grid(True, linestyle='--', alpha=0.7)
    ax2.legend()

    # Lưu ảnh và hiển thị
    output_img = 'training_performance.png'
    plt.savefig(output_img)
    print(f"✅ Đã lưu biểu đồ tại: {output_img}")
    plt.show()

if __name__ == "__main__":
    plot_training_history()
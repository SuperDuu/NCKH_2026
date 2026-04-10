import pandas as pd
import matplotlib.pyplot as plt
import os

# Đường dẫn tới file CSV
csv_path = '/home/du/Desktop/NCKH_2026/Software/Ros2_WS_v2/src/ros2_pkg_v2/ros2_pkg/weights/balancing_step_ppo/training_history.csv'
output_dir = os.path.dirname(csv_path)

def plot_training_history(csv_file):
    # Đọc dữ liệu
    try:
        df = pd.read_csv(csv_file)
        # Chỉ lấy 1100 tập đầu tiên
        df = df.head(300)
    except Exception as e:
        print(f"Lỗi khi đọc file: {e}")
        return

    # Kiểm tra các cột cần thiết
    required_columns = ['Episode', 'Reward', 'Steps']
    if not all(col in df.columns for col in required_columns):
        print(f"File CSV thiếu các cột cần thiết: {required_columns}")
        return

    # Tạo biểu đồ
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=True)

    # 1. Biểu đồ Reward
    ax1.plot(df['Episode'], df['Reward'], color='blue', alpha=0.3, label='Raw Reward')
    ax1.plot(df['Episode'], df['Reward'].rolling(window=50).mean(), color='red', linewidth=2, label='Moving Average (50)')
    ax1.set_ylabel('Reward')
    ax1.set_title('Training Reward Over Episodes')
    ax1.legend()
    ax1.grid(True, linestyle='--', alpha=0.7)

    # 2. Biểu đồ Steps
    ax2.plot(df['Episode'], df['Steps'], color='green', alpha=0.3, label='Raw Steps')
    ax2.plot(df['Episode'], df['Steps'].rolling(window=50).mean(), color='orange', linewidth=2, label='Moving Average (50)')
    ax2.set_xlabel('Episode')
    ax2.set_ylabel('Steps')
    ax2.set_title('Training Steps Over Episodes')
    ax2.legend()
    ax2.grid(True, linestyle='--', alpha=0.7)

    plt.tight_layout()
    
    # Lưu biểu đồ
    save_path = os.path.join(output_dir, 'training_plot.png')
    plt.savefig(save_path)
    print(f"Biểu đồ đã được lưu tại: {save_path}")
    plt.show()

if __name__ == "__main__":
    plot_training_history(csv_path)

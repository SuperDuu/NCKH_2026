#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>

// Định nghĩa các hằng số từ code gốc
#define HEIGHT 185.0f      // Chiều cao chân tiêu chuẩn
#define UVC_GAIN 0.25f     // Hệ số bù trừ góc nghiêng
#define RECOVERY_RATE 0.07f // Tốc độ phục hồi chiều cao

class UvcControllerNode : public rclcpp::Node {
    public:
        UvcControllerNode() : Node("uvc_controller_node") {
            // Khởi tạo biến trạng thái giống globals_t/core_t trong code C
            dxi = 0.0f; dyi = 0.0f;
            autoH = HEIGHT;
            sw = 0.0f; // Đứng yên nên step width = 0

            subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
                "/imu", 10, std::bind(&UvcControllerNode::imu_callback, this, std::placeholders::_1)
            );

            // Publisher để gửi tọa độ chân sau khi tính toán UVC
            // Trong thực tế, bạn sẽ gửi dxi, dyi này qua bộ Inverse Kinematics (IK)
            publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
                "/model/humanoid_robot/joint_trajectory", 10
            );

            RCLCPP_INFO(this->get_logger(), "UVC Geometric Node has started.");
        }

    private:
        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
            // 1. Chuyển đổi Quaternion sang Roll/Pitch (radian)
            double roll, pitch;
            get_euler_from_quat(msg->orientation, roll, pitch);

            // 2. Thuật toán UVC (Dựa trên hàm uvc() dòng 321-358 trong code C)
            float k_tilt = std::sqrt(pitch * pitch + roll * roll);
            float eff_pitch = pitch;
            float eff_roll = roll;

            // Áp dụng offset/deadband (dòng 324-331)
            if (k_tilt > 0.033f) {
                float k1 = (k_tilt - 0.033f) / k_tilt;
                eff_pitch *= k1;
                eff_roll *= k1;
            } else {
                eff_pitch = 0.0f;
                eff_roll = 0.0f;
            }

            float rollt = UVC_GAIN * eff_roll;
            float pitcht = UVC_GAIN * eff_pitch;

            // --- Tính toán trục Y (Roll) (dòng 339-343) ---
            float ky = std::atan((dyi - sw) / autoH);
            float kly = autoH / std::cos(ky);
            float ksy = ky + rollt;
            dyi = kly * std::sin(ksy) + sw;
            autoH = kly * std::cos(ksy);

            // --- Tính toán trục X (Pitch) (dòng 346-350) ---
            float kx = std::atan(dxi / autoH);
            float klx = autoH / std::cos(kx);
            float ksx = kx + pitcht;
            dxi = klx * std::sin(ksx);
            autoH = klx * std::cos(ksx);

            // Giới hạn (Limit) (dòng 352-355)
            dxi = std::clamp(dxi, -45.0f, 45.0f);
            dyi = std::clamp(dyi, -45.0f, 45.0f);

            // 3. Phục hồi (Recovery - Dựa trên hàm uvcSub() dòng 296-298)
            if (HEIGHT > autoH) {
                autoH += (HEIGHT - autoH) * RECOVERY_RATE;
            } else {
                autoH = HEIGHT;
            }

            // In kết quả kiểm tra
            std::cout << std::fixed << std::setprecision(2)
                      << "UVC Output -> dX: " << dxi << " | dY: " << dyi << " | H: " << autoH << "\r" << std::flush;
        }

        void get_euler_from_quat(const geometry_msgs::msg::Quaternion & q, double & roll, double & pitch) {
            double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
            double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
            roll = std::atan2(sinr_cosp, cosr_cosp);

            double sinp = 2 * (q.w * q.y - q.z * q.x);
            if (std::abs(sinp) >= 1) pitch = std::copysign(M_PI / 2, sinp);
            else pitch = std::asin(sinp);
        }

        // Biến trạng thái
        float dxi, dyi, autoH, sw;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UvcControllerNode>());
    rclcpp::shutdown();
    return 0;
}
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include <cmath>
#include <algorithm>
#include <vector>

// Thông số vật lý từ bảng dữ liệu của bạn (mm)
const double L1 = 60.0;    // Thân đùi (Hip to Knee)
const double L2 = 103.4;   // Bắp chân (Knee to Ankle)
const double HEIGHT_STD = 155.0; // Chiều cao đứng (thấp hơn tổng 163.4 để tránh khóa khớp)

class UvcIkNode : public rclcpp::Node {
public:
    UvcIkNode() : Node("uvc_ik_node") {
        // Khởi tạo biến trạng thái UVC
        dxi = 0.0; dyi = 0.0; autoH = HEIGHT_STD;
        gain = 0.25; recovery = 0.07;

        // Publisher gửi lệnh khớp đến Gazebo Bridge
        joint_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/model/humanoid_robot/joint_trajectory", 10);

        // Subscriber nhận dữ liệu IMU
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&UvcIkNode::imu_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Node UVC + IK da san sang điều khiển Robot PETG.");
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // 1. Tính góc Pitch từ Quaternion (Góc nghiêng trước/sau)
        double sinp = 2.0 * (msg->orientation.w * msg->orientation.y - msg->orientation.z * msg->orientation.x);
        double pitch = std::asin(std::clamp(sinp, -1.0, 1.0));

        // 2. Thuật toán UVC Hình học (Dr. Guero) - Trục X (Pitch)
        if (std::abs(pitch) > 0.033) { // Deadband ~1.9 độ
            double pitcht = gain * pitch;
            double k = std::atan2(dxi, autoH);
            double kl = autoH / std::cos(k);
            double ks = k + pitcht;
            
            dxi = kl * std::sin(ks);
            autoH = kl * std::cos(ks);
            dxi = std::clamp(dxi, -40.0, 40.0); // Giới hạn bước bù
        }

        // 3. Phục hồi trạng thái (Recovery)
        autoH += (HEIGHT_STD - autoH) * recovery;
        if (std::abs(pitch) < 0.05) dxi -= dxi * recovery;

        // 4. Giải Nghịch đảo động học (IK)
        double hp, kp, ap;
        solve_ik(dxi, autoH, hp, kp, ap);

        // 5. Gửi lệnh JointTrajectory
        publish_command(hp, kp, ap);
    }

    void solve_ik(double dx, double h, double &hp, double &kp, double &ap) {
        double L_sq = dx * dx + h * h;
        double L = std::sqrt(L_sq);

        // Luật hàm Cos cho khớp Gối
        double cos_k = (L1 * L1 + L2 * L2 - L_sq) / (2.0 * L1 * L2);
        kp = std::acos(std::clamp(cos_k, -1.0, 1.0)); 

        // Góc Hông
        double alpha = std::atan2(dx, h);
        double cos_beta = (L1 * L1 + L_sq - L2 * L2) / (2.0 * L1 * L);
        double beta = std::acos(std::clamp(cos_beta, -1.0, 1.0));
        hp = alpha + beta;

        // Góc cổ chân (giữ bàn chân song song mặt đất)
        ap = kp - hp;
    }

    void publish_command(double hp, double kp, double ap) {
        auto msg = trajectory_msgs::msg::JointTrajectory();
        msg.joint_names = {
            "hip_hip_left_joint", "hip_knee_left_joint", "knee_ankle_left_joint",
            "hip_hip_right_joint", "hip_knee_right_joint", "knee_ankle_right_joint"
        };

        trajectory_msgs::msg::JointTrajectoryPoint point;
        // Chú ý: Dấu âm (-) ở Gối và Cổ chân để khớp với hướng quay trong URDF
        point.positions = {hp, -kp, -ap, hp, -kp, -ap};
        point.time_from_start = rclcpp::Duration::from_nanoseconds(20000000); // 20ms

        msg.points.push_back(point);
        joint_pub_->publish(msg);
    }

    double dxi, dyi, autoH, gain, recovery;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UvcIkNode>());
    rclcpp::shutdown();
    return 0;
}
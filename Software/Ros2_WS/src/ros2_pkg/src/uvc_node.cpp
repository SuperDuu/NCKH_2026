#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include <cmath>
#include <algorithm>
#include <vector>

#define L1 60.0f
#define L2 103.4f
#define HEIGHT_STD 150.0f

class UvcControllerNode : public rclcpp::Node {
public:
    UvcControllerNode() : Node("uvc_node") {
        dxi = 0.0f; autoH = HEIGHT_STD;

        joint_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/model/humanoid_robot/joint_trajectory", 10);

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&UvcControllerNode::imu_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Node UVC da san sang. Dang doi du lieu IMU...");
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // 1. Tính toán Pitch từ Quaternion
        double sinp = 2.0 * (msg->orientation.w * msg->orientation.y - msg->orientation.z * msg->orientation.x);
        double pitch = std::asin(std::clamp(sinp, -1.0, 1.0));

        // 2. Logic UVC đơn giản (để test luồng dữ liệu)
        // Nếu muốn dùng thuật toán đầy đủ, hãy thay phần này bằng logic dxi, autoH của bạn
        float pitch_f = static_cast<float>(pitch);
        if (std::abs(pitch_f) > 0.033f) {
            dxi = 20.0f * pitch_f; // Giả lập bù trừ đơn giản
        } else {
            dxi = 0.0f;
        }

        // 3. Giải IK
        double hp, kp, ap;
        solve_ik(dxi, autoH, hp, kp, ap);

        // 4. Publish lệnh
        publish_command(hp, kp, ap);

        // In log để xác nhận node đang chạy
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100, 
            "Dang chay: Pitch=%.2f -> dX=%.2f | Lệnh gui: hp=%.2f", pitch, dxi, hp);
    }

    void solve_ik(float dx, float h, double &hp, double &kp, double &ap) {
        double L_sq = (double)dx * dx + (double)h * h;
        double L = std::sqrt(L_sq);
        double cos_k = (L1 * L1 + L2 * L2 - L_sq) / (2.0 * L1 * L2);
        kp = std::acos(std::clamp(cos_k, -1.0, 1.0)); 
        double alpha = std::atan2((double)dx, (double)h);
        double cos_beta = (L1 * L1 + L_sq - L2 * L2) / (2.0 * L1 * L);
        double beta = std::acos(std::clamp(cos_beta, -1.0, 1.0));
        hp = alpha + beta;
        ap = kp - hp;
    }

    void publish_command(double hp, double kp, double ap) {
        auto msg = trajectory_msgs::msg::JointTrajectory();
        
        // DANH SÁCH TOÀN BỘ CÁC KHỚP CÓ TRONG URDF CỦA BẠN
        msg.joint_names = {
            "hip_hip_left_joint", "hip_knee_left_joint", "knee_ankle_left_joint",
            "hip_hip_right_joint", "hip_knee_right_joint", "knee_ankle_right_joint",
            "base_hip_left_joint", "base_hip_right_joint", "base_hip_middle_joint"
        };

        trajectory_msgs::msg::JointTrajectoryPoint point;
        
        // Gán góc cho 6 khớp chân (UVC tính toán) và 3 khớp hông (để 0.0 để đứng thẳng)
        // Thứ tự phải khớp với mảng joint_names ở trên
        // point.positions = {
        //     hp, -kp, -ap,    // Left Leg
        //     hp, -kp, -ap,    // Right Leg
        //     0.0, 0.0, 0.0    // Hip yaw/roll joints
        // };

        point.positions = {
            1.57, 1.57, 1.57,    // Left Leg
            1.57, 1.57, 1.57,    // Right Leg
            0.0, 0.0, 0.0    // Hip yaw/roll joints
        };

        point.time_from_start = rclcpp::Duration::from_nanoseconds(10000000); // 10ms
        msg.points.push_back(point);
        joint_pub_->publish(msg);
    }

    float dxi, autoH;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    // Tạo node
    auto node = std::make_shared<UvcControllerNode>();
    
    // ÉP SỬ DỤNG SIM TIME: Giúp Node đồng bộ với Gazebo, sửa lỗi trễ log
    node->set_parameter(rclcpp::Parameter("use_sim_time", true));
    
    // Trong hàm in Log (RCLCPP_INFO_THROTTLE), hãy đổi 1000 thành 100
    // RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 100, "...");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
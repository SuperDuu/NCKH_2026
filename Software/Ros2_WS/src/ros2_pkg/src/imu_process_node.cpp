// File: src/ros2_pkg/src/imu_process_node.cpp
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/vector3.hpp" // Dùng để gửi 3 góc R/P/Y
#include <cmath>
#include <algorithm>

class ImuProcessNode : public rclcpp::Node {
public:
    ImuProcessNode() : Node("imu_process_node") {
        // Đăng ký nhận dữ liệu IMU thô
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&ImuProcessNode::imu_callback, this, std::placeholders::_1));

        // Tạo Publisher để gửi góc ĐỘ đã tính toán
        angle_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
            "/robot_orientation", 10);

        RCLCPP_INFO(this->get_logger(), "--> IMU PROCESSOR STARTED: Outputting Degrees");
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        double x = msg->orientation.x;
        double y = msg->orientation.y;
        double z = msg->orientation.z;
        double w = msg->orientation.w;

        // 1. Tính Roll (Nghiêng Trái/Phải)
        double sinr_cosp = 2 * (w * x + y * z);
        double cosr_cosp = 1 - 2 * (x * x + y * y);
        double roll_rad = std::atan2(sinr_cosp, cosr_cosp);

        // 2. Tính Pitch (Nghiêng Trước/Sau) - Xử lý Gimbal Lock
        double sinp = 2 * (w * y - z * x);
        double pitch_rad;
        if (std::abs(sinp) >= 1)
            pitch_rad = std::copysign(M_PI / 2, sinp); 
        else
            pitch_rad = std::asin(sinp);

        // 3. Tính Yaw (Xoay)
        double siny_cosp = 2 * (w * z + x * y);
        double cosy_cosp = 1 - 2 * (y * y + z * z);
        double yaw_rad = std::atan2(siny_cosp, cosy_cosp);

        // 4. Đổi sang ĐỘ và Gửi đi
        auto angle_msg = geometry_msgs::msg::Vector3();
        angle_msg.x = pitch_rad * (180.0 / M_PI);; // X đại diện cho PITCH
        angle_msg.y = roll_rad * (180.0 / M_PI);;  // Y đại diện cho ROLL
        angle_msg.z = yaw_rad * (180.0 / M_PI);;   // Z đại diện cho YAW
        std::cout<<pitch_rad<<"   "<<roll_rad<<std::endl;
        angle_pub_->publish(angle_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr angle_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuProcessNode>());
    rclcpp::shutdown();
    return 0;
}
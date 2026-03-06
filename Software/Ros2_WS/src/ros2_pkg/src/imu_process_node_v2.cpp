// File: src/ros2_pkg/src/imu_process_node_v2.cpp
// Phase 2: IMU Processor V2 — Normalized Quaternion + Angular Velocity
// Publishes sensor_msgs/Imu on /imu/data at 100Hz (zero-order hold from /imu raw)
//
// Tối ưu CPU: Loại bỏ tính toán Pure Acceleration vì kiến trúc 70D
// chỉ sử dụng Quaternion (4D) và Angular Velocity (3D).
// Giữ nguyên chuẩn hóa Quaternion trên manifold S³.

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <cmath>
#include <mutex>

class ImuProcessNodeV2 : public rclcpp::Node {
public:
    ImuProcessNodeV2() : Node("imu_process_node_v2") {
        // Subscribe to raw IMU from Gazebo bridge
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10,
            std::bind(&ImuProcessNodeV2::imu_callback, this, std::placeholders::_1));

        // Publish processed IMU at 100Hz
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);

        // 100Hz republish timer (10ms period)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&ImuProcessNodeV2::timer_callback, this));

        has_data_ = false;

        RCLCPP_INFO(this->get_logger(),
            "--> IMU PROCESSOR V2: Publishing NormQuat + Gyro on /imu/data at 100Hz (no accel)");
    }

private:
    // =========================================================================
    // Callback: nhận dữ liệu IMU thô từ Gazebo
    // =========================================================================
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);

        // --- Đọc Quaternion thô (x, y, z, w) ---
        qx_ = msg->orientation.x;
        qy_ = msg->orientation.y;
        qz_ = msg->orientation.z;
        qw_ = msg->orientation.w;

        // --- Chuẩn hóa Quaternion (Normalized Quaternion) ---
        // Lý do: BNO055 thực tế có thể trả quaternion lệch khỏi unit sphere
        // do nhiễu số. PPO cần input nằm trên manifold S³ (||q|| = 1) để
        // không gian học liên tục, tránh Gimbal Lock và hội tụ nhanh.
        // Công thức: q_norm = q / ||q||, với ||q|| = sqrt(qw² + qx² + qy² + qz²)
        double norm = std::sqrt(qw_ * qw_ + qx_ * qx_ + qy_ * qy_ + qz_ * qz_);
        if (norm > 1e-9) {
            qw_ /= norm;
            qx_ /= norm;
            qy_ /= norm;
            qz_ /= norm;
        }

        // --- Giữ nguyên Angular Velocity (rad/s) ---
        gx_ = msg->angular_velocity.x;
        gy_ = msg->angular_velocity.y;
        gz_ = msg->angular_velocity.z;

        // NOTE: Pure Acceleration KHÔNG CÒN TÍNH ở đây.
        // Kiến trúc 70D chỉ dùng Quaternion + Gyro → tiết kiệm CPU cho embedded.

        has_data_ = true;
    }

    // =========================================================================
    // Timer 100Hz: republish latest processed data (zero-order hold)
    // =========================================================================
    void timer_callback() {
        if (!has_data_) return;

        std::lock_guard<std::mutex> lock(data_mutex_);

        auto out = sensor_msgs::msg::Imu();
        out.header.stamp = this->now();
        out.header.frame_id = "imu_link";

        // Quaternion đã chuẩn hóa: giữ nguyên thứ tự x, y, z, w (sensor_msgs chuẩn)
        out.orientation.x = qx_;
        out.orientation.y = qy_;
        out.orientation.z = qz_;
        out.orientation.w = qw_;

        // Angular velocity (rad/s)
        out.angular_velocity.x = gx_;
        out.angular_velocity.y = gy_;
        out.angular_velocity.z = gz_;

        // Linear acceleration = zero (không tính, tiết kiệm CPU)
        out.linear_acceleration.x = 0.0;
        out.linear_acceleration.y = 0.0;
        out.linear_acceleration.z = 0.0;

        imu_pub_->publish(out);
    }

    // --- ROS interfaces ---
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // --- Cached latest data (không còn pure accel) ---
    std::mutex data_mutex_;
    bool has_data_;
    double qx_{0}, qy_{0}, qz_{0}, qw_{1};
    double gx_{0}, gy_{0}, gz_{0};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuProcessNodeV2>());
    rclcpp::shutdown();
    return 0;
}

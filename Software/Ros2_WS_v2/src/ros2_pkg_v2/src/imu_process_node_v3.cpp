// File: src/ros2_pkg_v2/src/imu_process_node_v3.cpp
// Phase 3: IMU Processor V3 — Normalized Quaternion + Angular Velocity + Linear Acceleration (BNO055 logic)
// Publishes sensor_msgs/Imu on /imu/data at 100Hz

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <cmath>
#include <mutex>

class ImuProcessNodeV3 : public rclcpp::Node {
public:
    ImuProcessNodeV3() : Node("imu_process_node_v3") {
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", rclcpp::SensorDataQoS(),
            std::bind(&ImuProcessNodeV3::imu_callback, this, std::placeholders::_1));

        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&ImuProcessNodeV3::timer_callback, this));

        has_data_ = false;

        RCLCPP_INFO(this->get_logger(),
            "--> IMU PROCESSOR V3: Publishing NormQuat + Gyro + LinAcc on /imu/data at 100Hz");
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);

        qx_ = msg->orientation.x;
        qy_ = msg->orientation.y;
        qz_ = msg->orientation.z;
        qw_ = msg->orientation.w;

        double norm = std::sqrt(qw_ * qw_ + qx_ * qx_ + qy_ * qy_ + qz_ * qz_);
        if (norm > 1e-9) {
            qw_ /= norm;
            qx_ /= norm;
            qy_ /= norm;
            qz_ /= norm;
        }

        gx_ = msg->angular_velocity.x;
        gy_ = msg->angular_velocity.y;
        gz_ = msg->angular_velocity.z;

        lax_ = msg->linear_acceleration.x;
        lay_ = msg->linear_acceleration.y;
        laz_ = msg->linear_acceleration.z;

        has_data_ = true;
    }

    void timer_callback() {
        if (!has_data_) return;

        std::lock_guard<std::mutex> lock(data_mutex_);

        auto out = sensor_msgs::msg::Imu();
        out.header.stamp = this->now();
        out.header.frame_id = "imu_link";

        out.orientation.x = qx_;
        out.orientation.y = qy_;
        out.orientation.z = qz_;
        out.orientation.w = qw_;

        out.angular_velocity.x = gx_;
        out.angular_velocity.y = gy_;
        out.angular_velocity.z = gz_;

        out.linear_acceleration.x = lax_;
        out.linear_acceleration.y = lay_;
        out.linear_acceleration.z = laz_;

        imu_pub_->publish(out);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::mutex data_mutex_;
    bool has_data_;
    double qx_{0}, qy_{0}, qz_{0}, qw_{1};
    double gx_{0}, gy_{0}, gz_{0};
    double lax_{0}, lay_{0}, laz_{0};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuProcessNodeV3>());
    rclcpp::shutdown();
    return 0;
}

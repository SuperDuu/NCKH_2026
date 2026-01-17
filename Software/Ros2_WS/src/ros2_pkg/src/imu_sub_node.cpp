#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <iostream>
#include <iomanip>
#include <cmath> 

class ImuSubNode : public rclcpp::Node {
    public:
        ImuSubNode() : Node("imu_sub_node") {
            subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
                "/imu", 
                10, 
                std::bind(&ImuSubNode::sub_callback, this, std::placeholders::_1)
            );
            std::cout << "Dang doc goc nghieng IMU (don vi: DO)..." << std::endl;
        }

    private:
        void sub_callback(const sensor_msgs::msg::Imu::SharedPtr msg) const {
            double x = msg->orientation.x;
            double y = msg->orientation.y;
            double z = msg->orientation.z;
            double w = msg->orientation.w;

            double sinr_cosp = 2 * (w * x + y * z);
            double cosr_cosp = 1 - 2 * (x * x + y * y);
            double roll = std::atan2(sinr_cosp, cosr_cosp);

            double sinp = 2 * (w * y - z * x);
            double pitch;
            if (std::abs(sinp) >= 1)
                pitch = std::copysign(M_PI / 2, sinp); 
            else
                pitch = std::asin(sinp);
            double siny_cosp = 2 * (w * z + x * y);
            double cosy_cosp = 1 - 2 * (y * y + z * z);
            double yaw = std::atan2(siny_cosp, cosy_cosp);

            double r_deg = roll * 180.0 / M_PI;
            double p_deg = pitch * 180.0 / M_PI;
            double y_deg = yaw * 180.0 / M_PI;

            std::cout << std::fixed << std::setprecision(2)
                      << "Goc (Do) -> Roll: " << r_deg 
                      << " | Pitch: " << p_deg 
                      << " | Yaw: " << y_deg 
                      << "   \r" << std::flush;
        }
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuSubNode>());
    rclcpp::shutdown();
    return 0;
}
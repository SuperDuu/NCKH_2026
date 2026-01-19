// File: src/ros2_pkg/src/uvc_node.cpp
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp" // Nhận góc từ Node kia
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include <cmath>
#include <algorithm>
#include <vector>

const double L1 = 60.0;     
const double L2 = 103.4;    
const double HEIGHT_STD = 135.0; 

class UvcControllerNode : public rclcpp::Node {
public:
    UvcControllerNode() : Node("uvc_node") {
        dxi = 0.0; dyi = 0.0; 
        autoH = HEIGHT_STD;
        
        gain_pitch = 0.5;   
        gain_roll = 0.5;    
        recovery = 0.05;    

        joint_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/model/humanoid_robot/joint_trajectory", 10);

        // THAY ĐỔI: Nghe topic /robot_orientation thay vì /imu
        angle_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/robot_orientation", 10, std::bind(&UvcControllerNode::control_callback, this, std::placeholders::_1));

        this->set_parameter(rclcpp::Parameter("use_sim_time", true));
        RCLCPP_INFO(this->get_logger(), "--> UVC CONTROLLER STARTED (Waiting for angles...)");
    }

private:
    // Callback này giờ nhận thẳng Vector3 (Độ)
    void control_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        
        // Lấy dữ liệu ĐỘ trực tiếp (không cần tính toán nữa)
        double pitch_deg = msg->x;
        double roll_deg  = msg->y;

        // --- Logic UVC ---
        // Pitch
        if (std::abs(pitch_deg) > 1.0) { 
            dxi = gain_pitch * pitch_deg; 
            dxi = std::clamp(dxi, -60.0, 60.0); 
        } else {
            dxi -= dxi * recovery;
        }

        // Roll
        if (std::abs(roll_deg) > 1.0) {
            dyi = gain_roll * roll_deg;
            dyi = std::clamp(dyi, -40.0, 40.0);
        } else {
            dyi -= dyi * recovery;
        }

        // --- Giải IK và Gửi lệnh ---
        double hp, kp, ap, hr, ar;
        solve_ik(dxi, dyi, autoH, hp, kp, ap, hr, ar);
        publish_command(hp, kp, ap, hr, ar);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200, 
            "Pitch: %.1f | Roll: %.1f | Action -> dxi: %.1f dyi: %.1f", pitch_deg, roll_deg, dxi, dyi);
    }

    void solve_ik(double dx, double dy, double h, 
                  double &hp, double &kp, double &ap, double &hr, double &ar) {
        // double L_sq = dx * dx + h * h;
        double L = std::sqrt(dx * dx + h * h + dy*dy);
        double cos_k = (L1 * L1 + L2 * L2 - L_sq) / (2.0 * L1 * L2);
        kp = std::acos(std::clamp(cos_k, -1.0, 1.0)); 
        double alpha = std::atan2(dx, h);
        double cos_beta = (L1 * L1 + L_sq - L2 * L2) / (2.0 * L1 * L);
        double beta = std::acos(std::clamp(cos_beta, -1.0, 1.0));
        hp = alpha + beta;
        ap = kp - hp;
        
        hr = std::atan2(dy, h); 
        ar = -hr;
    }

    void publish_command(double hp, double kp, double ap, double hr, double ar) {
        auto msg = trajectory_msgs::msg::JointTrajectory();
        msg.joint_names = {
            "hip_hip_left_joint", "base_hip_left_joint", "hip_knee_left_joint","knee_knee_left_joint","knee_ankle_left_joint",
            "hip_hip_right_joint", "base_hip_right_joint","hip_knee_right_joint","ankle_ankle_right_joint",
            
            
              
              // Cổ chân Roll
            "base_hip_middle_joint",
            "hip_shoulder_left_joint", "shoulder_shoulder_left_joint", "shoulder_elbow_left_joint",
            "hip_shoulder_right_joint", "shoulder_shoulder_right_joint", "shoulder_elbow_right_joint"
        };
        //hip_hip_left_joint hong thang
        //base_hip_left_joint hong ngang
        //hip_knee_right_joint dau goi
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {
            -hp, kp, ap, hp, -kp, -ap, 
            hr, -hr, ar, -ar, 
            0.0, 
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        };
        point.time_from_start = rclcpp::Duration::from_nanoseconds(10000000); 
        msg.points.push_back(point);
        joint_pub_->publish(msg);
    }

    double dxi, dyi, autoH;
    double gain_pitch, gain_roll, recovery;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr angle_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UvcControllerNode>());
    rclcpp::shutdown();
    return 0;
}
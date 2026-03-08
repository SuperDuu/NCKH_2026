// File: src/ros2_pkg/src/humanoid_llc_node_v2.cpp
// Phase 2: Humanoid LLC V2 | L3=0.08 L4=0.15 L5=0.065 | Z=0.265
// Dead Zone 0.015 rad (~1°) cho MG996R chống rung

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp> 
#include <cmath>
#include <map>
#include <vector>
#include <string>
#include <algorithm>

namespace TrajectoryMath {
    double linear_interp(double start, double end, double t_norm) {
        t_norm = std::clamp(t_norm, 0.0, 1.0);
        return start + (end - start) * t_norm;
    }
    double bezier_quadratic(double p0, double p1, double p2, double t_norm) {
        t_norm = std::clamp(t_norm, 0.0, 1.0);
        double u = 1.0 - t_norm;
        return (u * u * p0) + (2.0 * u * t_norm * p1) + (t_norm * t_norm * p2);
    }
}

class LegStepper {
public:
    double curr_x, curr_y, curr_z;
    LegStepper(double init_x, double init_y, double init_z) {
        d_x = init_x; d_y = init_y; d_z = init_z;
        reset_state();
    }
    void reset_state() {
        curr_x = s_x = t_x = d_x;
        curr_y = s_y = t_y = d_y;
        curr_z = s_z = t_z = d_z;
        phase = 1.0; duration = 0.05; is_sw = false;
    }
    void set_target(double x, double y, double z, double dur, double lift) {
        s_x = curr_x; s_y = curr_y; s_z = curr_z;
        t_x = x; t_y = y; t_z = z;
        phase = 0.0;
        duration = std::max(dur, 0.01);
        is_sw = (lift > 0.005); s_h = lift;
    }
    void update(double dt) {
        if (phase < 1.0) { phase += dt / duration; if (phase > 1.0) phase = 1.0; }
        curr_x = TrajectoryMath::linear_interp(s_x, t_x, phase);
        curr_y = TrajectoryMath::linear_interp(s_y, t_y, phase);
        if (is_sw) {
            double mid_z = std::min(s_z, t_z) - s_h;
            curr_z = TrajectoryMath::bezier_quadratic(s_z, mid_z, t_z, phase);
        } else { curr_z = TrajectoryMath::linear_interp(s_z, t_z, phase); }
    }
private:
    double d_x, d_y, d_z, s_x, s_y, s_z, t_x, t_y, t_z, phase, duration, s_h;
    bool is_sw;
};

class HumanoidIKControllerV2 : public rclcpp::Node {
public:
    const double L3 = 0.08;
    const double L4 = 0.15;
    const double L5 = 0.065;
    const double ALPHA = 0.6;
    // Dead Zone: MG996R rơ 1-5°. Lệnh < 0.015 rad (~1°) không làm servo
    // phản ứng, chỉ gây rung (buzzing) và mài mòn bánh răng nhựa.
    const double DEAD_ZONE = 0.0; // Tắt trong môi trường mô phỏng

    HumanoidIKControllerV2() : Node("humanoid_llc_node"),
        left_leg(0, 0.01, 0.265), right_leg(0, -0.01, 0.265) {
        std::vector<std::string> j_names = {
            "base_hip_left_joint", "hip_hip_left_joint", "hip_knee_left_joint", "knee_ankle_left_joint", "ankle_ankle_left_joint",
            "base_hip_right_joint", "hip_hip_right_joint", "hip_knee_right_joint", "knee_ankle_right_joint", "ankle_ankle_right_joint",
            "base_hip_middle_joint",
            "hip_shoulder_left_joint", "shoulder_shoulder_left_joint", "shoulder_elbow_left_joint",
            "hip_shoulder_right_joint", "shoulder_shoulder_right_joint", "shoulder_elbow_right_joint"
        };
        
        traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_trajectory_controller/joint_trajectory", 10);
        
        for (auto &n : j_names) {
            last_pos_[n] = 0.0;
        }
        rl_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>("/rl/leg_command", 10,
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                if (is_res_ || msg->data.size() < 9) return;
                left_leg.set_target(msg->data[0], msg->data[1], msg->data[2], msg->data[8], msg->data[3]);
                right_leg.set_target(msg->data[4], msg->data[5], msg->data[6], msg->data[8], msg->data[7]);
            });
        res_sub_ = create_subscription<std_msgs::msg::Bool>("/uvc_reset", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                is_res_ = msg->data;
                if (is_res_) {
                    left_leg.reset_state(); right_leg.reset_state();
                    for (auto const& [name, val] : last_pos_) last_pos_[name] = 0.0;
                }
            });
        timer_ = create_wall_timer(std::chrono::milliseconds(10),
            std::bind(&HumanoidIKControllerV2::loop, this));
        RCLCPP_INFO(this->get_logger(),
            "LLC V2 (Trajectory Mode): L3=%.3f L4=%.3f L5=%.3f | Z=0.265 | Alpha=%.1f | DeadZone=%.3f",
            L3, L4, L5, ALPHA, DEAD_ZONE);
    }

private:
    LegStepper left_leg, right_leg;
    bool is_res_ = false;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
    std::map<std::string, double> last_pos_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rl_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr res_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double filter_joint(std::string name, double target) {
        if (std::abs(target - last_pos_[name]) < DEAD_ZONE) return last_pos_[name];
        double filtered = last_pos_[name] + ALPHA * (target - last_pos_[name]);
        last_pos_[name] = filtered;
        return filtered;
    }

    bool solve_ik(double dx, double dy, double dz, double *a, bool is_r) {
        double d_target = sqrt(dx*dx + dy*dy + dz*dz);
        double MAX_REACH = L3 + L4 + L5 - 0.002;
        if (d_target > MAX_REACH) {
            double scale = MAX_REACH / d_target;
            dx *= scale; dy *= scale; dz *= scale;
        }
        double hn = atan2(dy, dz);
        double d_yz = sqrt(dz * dz + dy * dy);
        double c3_raw = (pow(d_yz - L5, 2) + dx * dx - L3 * L3 - L4 * L4) / (2.0 * L3 * L4);
        if (c3_raw < -1.05 || c3_raw > 1.05) return false;
        double c3 = std::clamp(c3_raw, -1.0, 1.0);
        double s3 = sqrt(1.0 - c3 * c3);
        double dg = atan2(s3, c3);
        double ht = atan2(s3 * L4, L3 + c3 * L4) + atan2(dx, d_yz - L5);
        a[0] = hn;
        a[1] = is_r ? ht : -ht;
        a[2] = is_r ? -dg : dg;
        a[3] = is_r ? -(ht - dg) : (ht - dg);
        a[4] = -hn;
        return true;
    }

    void loop() {
        if (is_res_) return;
        left_leg.update(0.01); right_leg.update(0.01);
        double l_a[5], r_a[5];
        if (solve_ik(left_leg.curr_x, left_leg.curr_y, left_leg.curr_z, l_a, false) &&
            solve_ik(right_leg.curr_x, right_leg.curr_y, right_leg.curr_z, r_a, true)) {
            
            trajectory_msgs::msg::JointTrajectory traj_msg;
            traj_msg.header.stamp = this->now();
            traj_msg.joint_names = {
                "base_hip_left_joint", "hip_hip_left_joint", "hip_knee_left_joint", "knee_ankle_left_joint", "ankle_ankle_left_joint",
                "base_hip_right_joint", "hip_hip_right_joint", "hip_knee_right_joint", "knee_ankle_right_joint", "ankle_ankle_right_joint",
                "base_hip_middle_joint",
                "hip_shoulder_left_joint", "shoulder_shoulder_left_joint", "shoulder_elbow_left_joint",
                "hip_shoulder_right_joint", "shoulder_shoulder_right_joint", "shoulder_elbow_right_joint"
            };

            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.positions.reserve(17);

            // Chân trái
            point.positions.push_back(filter_joint("base_hip_left_joint", l_a[0]));
            point.positions.push_back(filter_joint("hip_hip_left_joint", l_a[1]));
            point.positions.push_back(filter_joint("hip_knee_left_joint", l_a[2]));
            point.positions.push_back(filter_joint("knee_ankle_left_joint", l_a[3]));
            point.positions.push_back(filter_joint("ankle_ankle_left_joint", l_a[4]));

            // Chân phải
            point.positions.push_back(filter_joint("base_hip_right_joint", r_a[0]));
            point.positions.push_back(filter_joint("hip_hip_right_joint", r_a[1]));
            point.positions.push_back(filter_joint("hip_knee_right_joint", r_a[2]));
            point.positions.push_back(filter_joint("knee_ankle_right_joint", r_a[3]));
            point.positions.push_back(filter_joint("ankle_ankle_right_joint", r_a[4]));

            // Thân và Tay: nội suy ở mức 0.0 để giữ nguyên tư thế thẳng, không rũ
            point.positions.push_back(filter_joint("base_hip_middle_joint", 0.0));
            point.positions.push_back(filter_joint("hip_shoulder_left_joint", 0.0));
            point.positions.push_back(filter_joint("shoulder_shoulder_left_joint", 0.0));
            point.positions.push_back(filter_joint("shoulder_elbow_left_joint", 0.0));
            point.positions.push_back(filter_joint("hip_shoulder_right_joint", 0.0));
            point.positions.push_back(filter_joint("shoulder_shoulder_right_joint", 0.0));
            point.positions.push_back(filter_joint("shoulder_elbow_right_joint", 0.0));

            point.time_from_start = rclcpp::Duration(0, 10000000); // 0.01s (10ms)
            traj_msg.points.push_back(point);
            
            traj_pub_->publish(traj_msg);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HumanoidIKControllerV2>());
    rclcpp::shutdown();
    return 0;
}

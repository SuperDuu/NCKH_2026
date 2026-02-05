#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
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
        phase = 1.0; 
        duration = 0.05; 
        is_sw = false;
    }

    void set_target(double x, double y, double z, double dur, double lift) {
        s_x = curr_x; s_y = curr_y; s_z = curr_z;
        t_x = x; t_y = y; t_z = z;
        phase = 0.0;
        duration = std::max(dur, 0.01);
        is_sw = (lift > 0.005);
        s_h = lift;
    }

    void update(double dt) {
        if (phase < 1.0) {
            phase += dt / duration;
            if (phase > 1.0) phase = 1.0;
        }
        curr_x = TrajectoryMath::linear_interp(s_x, t_x, phase);
        curr_y = TrajectoryMath::linear_interp(s_y, t_y, phase);
        if (is_sw) {
            double mid_z = std::min(s_z, t_z) - s_h;
            curr_z = TrajectoryMath::bezier_quadratic(s_z, mid_z, t_z, phase);
        } else {
            curr_z = TrajectoryMath::linear_interp(s_z, t_z, phase);
        }
    }

private:
    double d_x, d_y, d_z, s_x, s_y, s_z, t_x, t_y, t_z, phase, duration, s_h;
    bool is_sw;
};

class HumanoidIKController : public rclcpp::Node {
public:
    // Chiều dài các đốt chân (m)
    const double L3 = 0.06;    
    const double L4 = 0.1034;  
    const double L5 = 0.057;   
    const double ALPHA = 0.6; // Lọc thông thấp (0.1: rất chậm, 1.0: không lọc)

    HumanoidIKController() : Node("humanoid_llc_node"), 
        left_leg(0, 0.01, 0.195), right_leg(0, -0.01, 0.195) {
        
        std::vector<std::string> j_names = {
            "base_hip_left", "hip_hip_left", "hip_knee_left", "knee_ankle_left", "ankle_ankle_left",
            "base_hip_right", "hip_hip_right", "hip_knee_right", "knee_ankle_right", "ankle_ankle_right"
        };

        for (auto &n : j_names) {
            pubs_[n] = create_publisher<std_msgs::msg::Float64>("/model/humanoid_robot/joint/" + n + "_joint/cmd_pos", 10);
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
                    left_leg.reset_state(); 
                    right_leg.reset_state(); 
                    // Reset bộ lọc
                    for (auto const& [name, val] : last_pos_) last_pos_[name] = 0.0;
                }
            });

        timer_ = create_wall_timer(std::chrono::milliseconds(10), std::bind(&HumanoidIKController::loop, this));
        RCLCPP_INFO(this->get_logger(), "LLC NODE: Ready at Z=0.195 with Alpha Filter 0.6");
    }

private:
    LegStepper left_leg, right_leg;
    bool is_res_ = false;
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> pubs_;
    std::map<std::string, double> last_pos_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rl_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr res_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void publish_filtered(std::string name, double target) {
        // Bộ lọc thông thấp giúp servo không bị sốc khi AI đổi Action
        double filtered = last_pos_[name] + ALPHA * (target - last_pos_[name]);
        std_msgs::msg::Float64 m; m.data = filtered;
        pubs_[name]->publish(m);
        last_pos_[name] = filtered;
    }

    bool solve_ik(double dx, double dy, double dz, double *a, bool is_r) {
        // Tối ưu hóa: Nếu chân bị duỗi quá dài (vượt giới hạn vật lý), thu hẹp Y để cứu Z
        double d_target = sqrt(dx*dx + dy*dy + dz*dz);
        double MAX_REACH = L3 + L4 + L5 - 0.002; // Chừa 2mm an toàn cho gối

        if (d_target > MAX_REACH) {
            double scale = MAX_REACH / d_target;
            dx *= scale; dy *= scale; dz *= scale;
        }

        double hn = atan2(dy, dz);
        double d_yz = sqrt(dz * dz + dy * dy);
        double c3_raw = (pow(d_yz - L5, 2) + dx * dx - L3 * L3 - L4 * L4) / (2.0 * L3 * L4);
        
        // Clamp lỏng hơn một chút (-1.05) để tránh mất giải nghiệm khi chân duỗi thẳng ở Z=0.195
        if (c3_raw < -1.05 || c3_raw > 1.05) return false;
        
        double c3 = std::clamp(c3_raw, -1.0, 1.0);
        double s3 = sqrt(1.0 - c3 * c3);
        double dg = atan2(s3, c3); // Joint Knee
        double ht = atan2(s3 * L4, L3 + c3 * L4) + atan2(dx, d_yz - L5); // Joint Hip pitch
        
        a[0] = hn;             // Hip roll
        a[1] = is_r ? ht : -ht; // Hip pitch (đảo dấu cho chân trái)
        a[2] = is_r ? -dg : dg; // Knee (đảo dấu cho chân phải)
        a[3] = is_r ? -(ht - dg) : (ht - dg); // Ankle pitch
        a[4] = -hn;            // Ankle roll
        return true;
    }

    void loop() {
        if (is_res_) return;
        
        left_leg.update(0.01); 
        right_leg.update(0.01);
        
        double l_a[5], r_a[5];
        if (solve_ik(left_leg.curr_x, left_leg.curr_y, left_leg.curr_z, l_a, false) &&
            solve_ik(right_leg.curr_x, right_leg.curr_y, right_leg.curr_z, r_a, true)) {
            
            publish_filtered("base_hip_left", l_a[0]);
            publish_filtered("hip_hip_left", l_a[1]);
            publish_filtered("hip_knee_left", l_a[2]);
            publish_filtered("knee_ankle_left", l_a[3]);
            publish_filtered("ankle_ankle_left", l_a[4]);

            publish_filtered("base_hip_right", r_a[0]);
            publish_filtered("hip_hip_right", r_a[1]);
            publish_filtered("hip_knee_right", r_a[2]);
            publish_filtered("knee_ankle_right", r_a[3]);
            publish_filtered("ankle_ankle_right", r_a[4]);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HumanoidIKController>());
    rclcpp::shutdown();
    return 0;
}
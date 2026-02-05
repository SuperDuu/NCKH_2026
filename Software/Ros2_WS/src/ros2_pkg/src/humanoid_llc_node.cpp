#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp> 
#include <geometry_msgs/msg/vector3.hpp> 
#include <cmath>
#include <map>
#include <vector>
#include <string>
#include <algorithm>

namespace TrajectoryMath {
    // Nội suy tuyến tính để giữ vận tốc hằng số, tránh khựng ở cuối pha
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
        default_x = init_x; default_y = init_y; default_z = init_z;
        reset_state();
    }

    void reset_state() {
        curr_x = start_x = target_x = default_x;
        curr_y = start_y = target_y = default_y;
        curr_z = start_z = target_z = default_z;
        phase = 1.0; 
        step_duration = 1.0;
        swing_h = 0.0;
        is_swinging = false;
    }

    void set_target(double x, double y, double z, double duration, double lift_h) {
        bool new_swing_state = (lift_h > 0.005);
        if (new_swing_state && !is_swinging) {
            start_x = curr_x; 
            start_y = curr_y; 
            start_z = curr_z;
            phase = 0.0; 
        }
        target_x = x; target_y = y; target_z = z;
        step_duration = std::max(duration, 0.1); 
        swing_h = lift_h;
        is_swinging = new_swing_state;
    }

    void update(double dt) {
        if (phase < 1.0) {
            phase += dt / step_duration;
            if (phase > 1.0) phase = 1.0;
        }

        // Dùng mũ 0.5 để cân bằng giữa giật nhanh và mượt cho trục Z
        double fast = std::pow(phase, 0.5); 

        // SỬA: Dùng Linear thay vì Cosine cho X để không bị giảm tốc độ ở cuối bước
        curr_x = TrajectoryMath::linear_interp(start_x, target_x, phase);

        if (is_swinging) {
            // CHÂN LĂNG: Di chuyển Y tuyến tính để giữ đà văng
            curr_y = TrajectoryMath::linear_interp(start_y, target_y, phase);
            double p1 = std::min(start_z, target_z) - swing_h;
            curr_z = TrajectoryMath::bezier_quadratic(start_z, p1, target_z, fast);
        } 
        else {
            // CHÂN TRỤ: Dùng fast_phase cho Y để ép hông lấn tâm cực nhanh ngay lập tức
            curr_y = start_y + (target_y - start_y) * fast; 
            curr_z = TrajectoryMath::linear_interp(start_z, target_z, phase);
        }
    }

private:
    double default_x, default_y, default_z;
    double start_x, start_y, start_z;
    double target_x, target_y, target_z;
    double phase, step_duration, swing_h;
    bool is_swinging;
};

class HumanoidIKController : public rclcpp::Node {
public:
    const double L3 = 0.06;    
    const double L4 = 0.1034;  
    const double L5 = 0.057;   

    HumanoidIKController() : Node("humanoid_llc_node"),
        left_leg(0.0, 0.02, 0.18),   
        right_leg(0.0, -0.02, 0.18), 
        is_resetting(false)
    {
        std::vector<std::string> joints = {
            "base_hip_left", "hip_hip_left", "hip_knee_left", "knee_ankle_left", "ankle_ankle_left",
            "base_hip_right", "hip_hip_right", "hip_knee_right", "knee_ankle_right", "ankle_ankle_right"
        };

        for (const auto & name : joints) {
            pubs_[name] = this->create_publisher<std_msgs::msg::Float64>(
                "/model/humanoid_robot/joint/" + name + "_joint/cmd_pos", 10);
        }

        reset_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/uvc_reset", 10, std::bind(&HumanoidIKController::reset_callback, this, std::placeholders::_1));

        rl_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/rl/leg_command", 10, std::bind(&HumanoidIKController::rl_cmd_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&HumanoidIKController::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "--> LLC NODE READY: LINEAR-SYNC ACTIVATED (0.5s Support)");
    }

    bool solve_ik_safe(double dx, double dy, double dz, double &hn, double &ht, double &dg, double &mct, double &mcn) {
        hn = std::atan2(dy, dz);
        double d_yz = std::sqrt(dz * dz + dy * dy);
        double cos3_raw = (std::pow(d_yz - L5, 2) + dx * dx - L3 * L3 - L4 * L4) / (2.0 * L3 * L4);
        
        if (cos3_raw < -1.0 || cos3_raw > 1.0) return false; 

        double cos3 = std::clamp(cos3_raw, -1.0, 1.0);
        double sin3 = std::sqrt(1.0 - cos3 * cos3);

        dg = std::atan2(sin3, cos3);
        ht = std::atan2(sin3 * L4, L3 + cos3 * L4) + std::atan2(dx, d_yz - L5);
        mct = -dg + ht ; 
        mcn = -hn;
        return true;
    }

private:
    LegStepper left_leg;
    LegStepper right_leg;
    bool is_resetting;
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> pubs_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rl_cmd_sub_; 
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void reset_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        is_resetting = msg->data;
        if (is_resetting) {
            left_leg.reset_state();
            right_leg.reset_state();
        }
    }

    void rl_cmd_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (is_resetting || msg->data.size() < 9) return;
        left_leg.set_target(msg->data[0], msg->data[1], msg->data[2], msg->data[8], msg->data[3]);
        right_leg.set_target(msg->data[4], msg->data[5], msg->data[6], msg->data[8], msg->data[7]);
    }

    void send_cmd(std::string name, double pos) {
        auto msg = std_msgs::msg::Float64();
        msg.data = pos;
        pubs_[name]->publish(msg);
    }

    void control_loop() {
        if (is_resetting) return; 

        double l_hn, l_ht, l_dg, l_mct, l_mcn;
        double r_hn, r_ht, r_dg, r_mct, r_mcn;
        
        left_leg.update(0.01);
        right_leg.update(0.01);

        if (solve_ik_safe(left_leg.curr_x, left_leg.curr_y, left_leg.curr_z, l_hn, l_ht, l_dg, l_mct, l_mcn) &&
            solve_ik_safe(right_leg.curr_x, right_leg.curr_y, right_leg.curr_z, r_hn, r_ht, r_dg, r_mct, r_mcn)) {
            
            send_cmd("base_hip_left", l_hn); 
            send_cmd("hip_hip_left", -l_ht); 
            send_cmd("hip_knee_left", l_dg); 
            send_cmd("knee_ankle_left", l_mct); 
            send_cmd("ankle_ankle_left", l_mcn); 

            send_cmd("base_hip_right", r_hn); 
            send_cmd("hip_hip_right", r_ht); 
            send_cmd("hip_knee_right", -r_dg); 
            send_cmd("knee_ankle_right", -r_mct); 
            send_cmd("ankle_ankle_right", -r_mcn); 
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HumanoidIKController>());
    rclcpp::shutdown();
    return 0;
}
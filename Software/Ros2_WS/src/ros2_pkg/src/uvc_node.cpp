#include "std_msgs/msg/float64_multi_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>
#include <algorithm>
#include <map>
#include <string>
#include <memory>
#include <chrono>
#include <thread>

// =================== HẰNG SỐ ĐỘNG HỌC ===================
const double L3 = 60.0;
const double L4 = 103.4;
const double L5 = 57.0;
const double HEIGHT_STD = 220.4 - 15.0;  
const double RESET_HEIGHT = 200.0;        
double rol = 0, pit = 0;

class UvcControllerNode : public rclcpp::Node {
public:
    UvcControllerNode() : Node("uvc_node") {
        // =================== THAM SỐ UVC CƠ BẢN ===================
        default_stance_width = 25.0;
        default_gain = 0.1;
        
        declare_parameter("gain", default_gain);
        declare_parameter("recovery_rate", 0.1);
        declare_parameter("step_duration", 40.0);
        declare_parameter("stance_width", default_stance_width);
        declare_parameter("max_foot_lift", 15.0);
        declare_parameter("scale_base", 0.15);
        declare_parameter("tilt_threshold", 2.0);
        declare_parameter("landing_phase", 6.0);
        declare_parameter("min_clearance", 15.0);
        declare_parameter("ankle_gain", 0.45);
        declare_parameter("gyro_gain", 0.25);
        declare_parameter("roll_step_gain", 140.0);
        
        gain = get_parameter("gain").as_double();
        rr = get_parameter("recovery_rate").as_double();
        fwctEnd = get_parameter("step_duration").as_double();
        stance_width = get_parameter("stance_width").as_double();
        fhMax = get_parameter("max_foot_lift").as_double();
        scale_base = get_parameter("scale_base").as_double();
        tilt_threshold_deg = get_parameter("tilt_threshold").as_double();
        landing_phase = get_parameter("landing_phase").as_double();
        min_clearance = get_parameter("min_clearance").as_double();
        ankle_gain = get_parameter("ankle_gain").as_double();
        gyro_gain = get_parameter("gyro_gain").as_double();
        roll_step_gain = get_parameter("roll_step_gain").as_double();
        
        tilt_threshold = tilt_threshold_deg * M_PI / 180.0;
        
        // =================== KHỞI TẠO BIẾN TRẠNG THÁI ===================
        fwct = 0.0;
        fwctUp = 1.0;
        autoH = RESET_HEIGHT;
        fh = 0.0;
        dxi = 0.0;      
        dyi = default_stance_width;  
        dxis = 0.0;     
        dyis = -default_stance_width;
        dxi_before = dxi;
        dyi_before = dyi;
        dxib = 0.0;
        dyib = 0.0;
        support_leg = 0;
        ankle_pitch = 0.0;
        ankle_roll = 0.0;
        pitch = 0.0;
        roll = 0.0;
        pitch_filtered = 0.0;
        roll_filtered = 0.0;
        pitch_prev = 0.0;
        roll_prev = 0.0;
        pitch_derivative = 0.0;
        roll_derivative = 0.0;
        last_tilt_magnitude = 0.0;
        pitch_offset = 0.0;
        roll_offset = 0.0;
        mode = 0;
        stable_count = 0;
        warmup_counter = 0;
        imu_spike_count = 0;
        last_valid_pitch = 0.0;
        last_valid_roll = 0.0;
        
        std::vector<std::string> joints = {
            "base_hip_left", "hip_hip_left", "hip_knee_left", "knee_ankle_left", "ankle_ankle_left",
            "base_hip_right", "hip_hip_right", "hip_knee_right", "knee_ankle_right", "ankle_ankle_right"
        };
        for (const auto& j : joints) pubs_[j] = create_joint_pub(j + "_joint");
        
        angle_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>("/robot_orientation", 10, std::bind(&UvcControllerNode::imu_callback, this, std::placeholders::_1));
        rl_param_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/uvc_parameters", 10, std::bind(&UvcControllerNode::rl_param_callback, this, std::placeholders::_1));
        reset_sub_ = this->create_subscription<std_msgs::msg::Bool>("/uvc_reset", 10, std::bind(&UvcControllerNode::reset_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&UvcControllerNode::control_loop, this));
        rl_feedback_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/uvc_rl_feedback", 10);
    }

private:
    void imu_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        double pitch_raw = -msg->x * M_PI / 180.0;
        double roll_raw = msg->y * M_PI / 180.0;
        rol = roll_raw;
        pit = pitch_raw;
        
        double pitch_change = std::abs(pitch_raw - last_valid_pitch);
        double roll_change = std::abs(roll_raw - last_valid_roll);
        const double SPIKE_THRESHOLD = 0.5;
        
        if (pitch_change > SPIKE_THRESHOLD || roll_change > SPIKE_THRESHOLD) {
            imu_spike_count++;
            if (imu_spike_count < 3) { pitch_raw = last_valid_pitch; roll_raw = last_valid_roll; }
            else { last_valid_pitch = pitch_raw; last_valid_roll = roll_raw; imu_spike_count = 0; }
        } else { last_valid_pitch = pitch_raw; last_valid_roll = roll_raw; imu_spike_count = 0; }
        
        double alpha = 0.3;
        pitch_filtered = alpha * pitch_raw + (1.0 - alpha) * pitch_filtered;
        roll_filtered = alpha * roll_raw + (1.0 - alpha) * roll_filtered;
        
        double dt = 0.01;
        pitch_derivative = std::clamp((pitch_filtered - pitch_prev) / dt, -50.0, 50.0);
        roll_derivative = std::clamp((roll_filtered - roll_prev) / dt, -50.0, 50.0);
        
        pitch_prev = pitch_filtered;
        roll_prev = roll_filtered;
    }

    void rl_param_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 10) {
            gain = msg->data[0]; scale_base = msg->data[1]; fwctEnd = msg->data[2];
            landing_phase = msg->data[3]; stance_width = msg->data[4]; fhMax = msg->data[5];
            rr = msg->data[6]; ankle_gain = msg->data[7]; gyro_gain = msg->data[8];
            roll_step_gain = msg->data[9];
        }
    }

    void reset_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            mode = -1; fwct = 0.0; fh = 0.0; support_leg = 0;
            dxi = 0.0; dyi = stance_width; dxis = 0.0; dyis = -stance_width;
            autoH = RESET_HEIGHT; pitch_filtered = 0.0; roll_filtered = 0.0;
            stable_count = 0; publish_safe_stance();
        } else {
            if (mode == -1) { warmup_counter = 50; mode = 1; }
        }
    }

    void control_loop() {
        if (warmup_counter > 0) {
            warmup_counter--;
            if (warmup_counter == 1) {
                pitch_filtered = 0.0; roll_filtered = 0.0;
                dxi = 0.0; dyi = stance_width; dxis = 0.0; dyis = -stance_width;
            }
            publish_safe_stance(); return;
        }
        if (mode == -1) publish_safe_stance();
        else if (mode == 1) standing_mode();
        else if (mode == 2) uvc_active_mode();
    }

    void standing_mode() {
        pitch = pitch_filtered - pitch_offset; roll = roll_filtered - roll_offset;
        calculate_ankle_compensation();
        if (std::abs(pitch) < 0.017 && std::abs(roll) < 0.017) {
            stable_count++;
            if (stable_count > 50) {
                autoH += (HEIGHT_STD - autoH) * 0.01;
                publish_safe_stance(); return;
            }
        } else stable_count = 0;

        if (std::sqrt(pitch*pitch + roll*roll) > tilt_threshold) {
            mode = 2; fwct = 0.0;
            support_leg = (roll > 0) ? 1 : 0; // 1: Chân trái trụ, 0: Chân phải trụ
        }
        publish_safe_stance();
    }

    void calculate_ankle_compensation() {
        ankle_pitch += (pitch * ankle_gain + pitch_derivative * gyro_gain) * 0.05;
        ankle_roll += (roll * ankle_gain + roll_derivative * gyro_gain) * 0.05;
        ankle_pitch = std::clamp(ankle_pitch, -0.26, 0.26);
        ankle_roll = std::clamp(ankle_roll, -0.26, 0.26);
    }

    void uvc_active_mode() {
        pitch = pitch_filtered - pitch_offset; roll = roll_filtered - roll_offset;
        calculate_ankle_compensation();
        
        double threshold = 0.033;
        double tilt = std::sqrt(pitch*pitch + roll*roll);
        if (tilt < threshold) { pitch = 0.0; roll = 0.0; }
        else { double k = (tilt - threshold) / tilt; pitch *= k; roll *= k; }

        dyi_before = dyi; dxi_before = dxi;
        apply_uvc_geometry_control();
        calculate_foot_lift();
        calculate_and_publish_legs();
        update_cycle_counter();
        publish_rl_feedback();

        if (fwct >= fwctEnd * 0.8 && std::sqrt(pitch*pitch + roll*roll) < tilt_threshold * 0.5) mode = 1;
    }

    void apply_uvc_geometry_control() {
        double tilt_mag = std::sqrt(pitch*pitch + roll*roll);
        double scale = scale_base * (1.0 + 0.8 * std::min(tilt_mag, 1.0));
        
        for (double* val : {&dyi, &dxi}) {
            double k = std::atan(*val / autoH);
            double kl = autoH / std::cos(k);
            double deriv = (val == &dyi) ? roll_derivative : pitch_derivative;
            double angle = (val == &dyi) ? roll : pitch;
            *val = kl * std::sin(k - (scale * angle + gain * deriv));
            autoH = kl * std::cos(k - (scale * angle + gain * deriv));
        }

        dxi = std::clamp(dxi, dxi_before - 5.0, dxi_before + 5.0);
        dyi = std::clamp(dyi, dyi_before - 5.0, dyi_before + 5.0);
        dxi = std::clamp(dxi, -70.0, 70.0);
        dyi = std::clamp(dyi, -75.0, 75.0); 

        if (autoH < HEIGHT_STD) autoH += (HEIGHT_STD - autoH) * rr;
        autoH = std::clamp(autoH, 140.0, 225.0);
    }

    void calculate_foot_lift() {
        if (fwct > landing_phase && fwct <= (fwctEnd - landing_phase)) {
            double progress = (fwct - landing_phase) / (fwctEnd - 2 * landing_phase);
            fh = fhMax * std::sin(M_PI * progress);
            if (autoH - fh < min_clearance) fh = std::max(0.0, autoH - min_clearance);
        } else fh = 0.0;
    }

    void calculate_and_publish_legs() {
        double l_hn, l_ht, l_dg, l_mct, l_mcn, r_hn, r_ht, r_dg, r_mct, r_mcn;
        double swing_dz = (fh > 0.0) ? std::max(autoH - fh, min_clearance) : autoH;
        
        // Cố định dấu dựa trên hàm reset thành công của bạn
        // Reset: Trái = stance_width, Phải = -stance_width
        
        if (support_leg == 0) { // CHÂN PHẢI TRỤ (Y âm), CHÂN TRÁI LĂNG (Y dương)
            solve_ik(dxi, -dyi, autoH, r_hn, r_ht, r_dg, r_mct, r_mcn); // dyi áp vào chân phải (âm)
            solve_ik(dxis, stance_width, swing_dz, l_hn, l_ht, l_dg, l_mct, l_mcn); 
        } else { // CHÂN TRÁI TRỤ (Y dương), CHÂN PHẢI LĂNG (Y âm)
            solve_ik(dxi, dyi, autoH, l_hn, l_ht, l_dg, l_mct, l_mcn); // dyi áp vào chân trái (dương)
            solve_ik(dxis, -stance_width, swing_dz, r_hn, r_ht, r_dg, r_mct, r_mcn);
        }

        l_mct-= ankle_pitch; r_mct += ankle_pitch;
        l_mcn -= ankle_roll; r_mcn += ankle_roll;
        publish_legs(l_hn, l_ht, l_dg, l_mct, l_mcn, r_hn, r_ht, r_dg, r_mct, r_mcn);
    }

    void update_cycle_counter() {
        fwct += fwctUp;
        if (fwct >= fwctEnd) {
            support_leg ^= 1; 
            fwct = 0.0; 
            fh = 0.0;
            std::swap(dxi, dxis);
            // Không gán lại dyi ở đây để tránh giật chân, UVC sẽ tự điều chỉnh tiếp
        }
    }

    void publish_safe_stance() {
        double l_hn, l_ht, l_dg, l_mct, l_mcn, r_hn, r_ht, r_dg, r_mct, r_mcn;
        solve_ik(0.0, stance_width, autoH, l_hn, l_ht, l_dg, l_mct, l_mcn);
        solve_ik(0.0, -stance_width, autoH, r_hn, r_ht, r_dg, r_mct, r_mcn);
        publish_legs(l_hn, l_ht, l_dg, l_mct, l_mcn, r_hn, r_ht, r_dg, r_mct, r_mcn);
    }

    void publish_rl_feedback() {
        auto msg = geometry_msgs::msg::Vector3();
        msg.x = pitch * 180.0 / M_PI; msg.y = roll * 180.0 / M_PI; msg.z = fwct / fwctEnd;
        rl_feedback_pub_->publish(msg);
    }

    void solve_ik(double dx, double dy, double dz, double &hn, double &ht, double &dg, double &mct, double &mcn) {
        hn = std::atan2(dy, dz);
        double d_yz = std::sqrt(dz * dz + dy * dy);
        double cos3 = (std::pow(d_yz - L5, 2) + dx * dx - L3 * L3 - L4 * L4) / (2.0 * L3 * L4);
        cos3 = std::clamp(cos3, -1.0, 1.0);
        double sin3 = std::sqrt(1.0 - cos3 * cos3);
        dg = std::atan2(sin3, cos3);
        ht = std::atan2(sin3 * L4, L3 + cos3 * L4) + std::atan2(dx, d_yz - L5);
        mct = -dg + ht - M_PI / 65.0 ;
        mcn = -hn ;
    }

    void publish_legs(double l_hn, double l_ht, double l_dg, double l_mct, double l_mcn, double r_hn, double r_ht, double r_dg, double r_mct, double r_mcn) {
        send_cmd("base_hip_left", l_hn); send_cmd("hip_hip_left", -l_ht); send_cmd("hip_knee_left", l_dg); send_cmd("knee_ankle_left", l_mct); send_cmd("ankle_ankle_left", l_mcn);
        send_cmd("base_hip_right", r_hn); send_cmd("hip_hip_right", r_ht); send_cmd("hip_knee_right", -r_dg); send_cmd("knee_ankle_right", -r_mct); send_cmd("ankle_ankle_right", -r_mcn);
    }

    void send_cmd(const std::string& key, double value) {
        if (pubs_.count(key)) {
            auto msg = std_msgs::msg::Float64(); msg.data = value;
            pubs_[key]->publish(msg);
        }
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr create_joint_pub(const std::string& joint_name) {
        return this->create_publisher<std_msgs::msg::Float64>("/model/humanoid_robot/joint/" + joint_name + "/cmd_pos", 10);
    }

    double gain, rr, fwctEnd, stance_width, fhMax, tilt_threshold, tilt_threshold_deg, landing_phase, min_clearance, scale_base, ankle_gain, gyro_gain, roll_step_gain;
    double fwct, fwctUp, autoH, fh, dxi, dyi, dxis, dyis, dxib, dyib, dxi_before, dyi_before, ankle_pitch, ankle_roll, pitch, roll, pitch_filtered, roll_filtered, pitch_prev, roll_prev, pitch_derivative, roll_derivative, last_tilt_magnitude, imu_spike_count, last_valid_pitch, last_valid_roll, pitch_offset, roll_offset;
    int support_leg, mode, stable_count, warmup_counter;
    double default_stance_width, default_gain;
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> pubs_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr rl_feedback_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr angle_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rl_param_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UvcControllerNode>());
    rclcpp::shutdown();
    return 0;
}
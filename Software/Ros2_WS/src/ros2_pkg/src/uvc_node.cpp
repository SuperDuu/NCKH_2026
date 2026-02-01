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
const double HEIGHT_STD = 220.4 - 10.0;  // 210.4mm
const double RESET_HEIGHT = 205.0;        // autoH ban đầu khi reset

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
        
        // Lấy giá trị tham số
        gain = get_parameter("gain").as_double();
        rr = get_parameter("recovery_rate").as_double();
        fwctEnd = get_parameter("step_duration").as_double();
        stance_width = get_parameter("stance_width").as_double();
        fhMax = get_parameter("max_foot_lift").as_double();
        scale_base = get_parameter("scale_base").as_double();
        tilt_threshold_deg = get_parameter("tilt_threshold").as_double();
        landing_phase = get_parameter("landing_phase").as_double();
        min_clearance = get_parameter("min_clearance").as_double();
        
        tilt_threshold = tilt_threshold_deg * M_PI / 180.0;
        
        // =================== KHỞI TẠO BIẾN TRẠNG THÁI ===================
        fwct = 0.0;
        fwctUp = 1.0;
        autoH = RESET_HEIGHT;  // Khởi tạo = 205mm
        fh = 0.0;
        
        // Tích phân vị trí - khởi tạo tư thế đứng song song
        dxi = 0.0;      
        dyi = stance_width;  
        dxis = 0.0;     
        dyis = -stance_width;
        
        dxi_before = dxi;
        dyi_before = dyi;
        
        dxib = 0.0;
        dyib = 0.0;
        
        support_leg = 0;
        
        // Biến IMU
        pitch = 0.0;
        roll = 0.0;
        pitch_filtered = 0.0;
        roll_filtered = 0.0;
        pitch_prev = 0.0;
        roll_prev = 0.0;
        pitch_derivative = 0.0;
        roll_derivative = 0.0;
        last_tilt_magnitude = 0.0;
        
        // Offset hiệu chuẩn
        pitch_offset = 0.0;
        roll_offset = 0.0;
        
        // Trạng thái hệ thống
        mode = 0;
        calibration_samples = 0;
        stable_count = 0;
        warmup_counter = 0;
        
        // Biến chống spike IMU
        imu_spike_count = 0;
        last_valid_pitch = 0.0;
        last_valid_roll = 0.0;
        
        // =================== KHỞI TẠO PUBLISHER ===================
        std::vector<std::string> joints = {
            "base_hip_left", "hip_hip_left", "hip_knee_left", "knee_ankle_left", "ankle_ankle_left",
            "base_hip_right", "hip_hip_right", "hip_knee_right", "knee_ankle_right", "ankle_ankle_right"
        };
        
        for (const auto& j : joints) {
            pubs_[j] = create_joint_pub(j + "_joint");
        }
        
        // =================== SUBSCRIBER VÀ TIMER ===================
        angle_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/robot_orientation", 10, 
            std::bind(&UvcControllerNode::imu_callback, this, std::placeholders::_1));
        
        rl_param_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/uvc_parameters", 10,
            std::bind(&UvcControllerNode::rl_param_callback, this, std::placeholders::_1));

        reset_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/uvc_reset", 10,
            std::bind(&UvcControllerNode::reset_callback, this, std::placeholders::_1));
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&UvcControllerNode::control_loop, this));
        
        rl_feedback_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
            "/uvc_rl_feedback", 10);
        
        // RCLCPP_INFO(this->get_logger(), "========================================");
        // RCLCPP_INFO(this->get_logger(), "UVC CONTROLLER - 7 PARAM VERSION");
        // RCLCPP_INFO(this->get_logger(), "Initial autoH: %.1f mm", RESET_HEIGHT);
        // RCLCPP_INFO(this->get_logger(), "========================================");
    }

private:
    ///////////////////////////////////////////////////////////////////////////////////
    //// IMU CALLBACK - Xử lý dữ liệu IMU với bộ lọc chống spike ////
    ///////////////////////////////////////////////////////////////////////////////////
    void imu_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        double pitch_raw = -msg->x * M_PI / 180.0;
        double roll_raw = -msg->y * M_PI / 180.0;
        
        // ============ CHỐNG SPIKE ============
        double pitch_change = std::abs(pitch_raw - last_valid_pitch);
        double roll_change = std::abs(roll_raw - last_valid_roll);
        
        const double SPIKE_THRESHOLD = 0.5;  // ~28.6°
        
        if (pitch_change > SPIKE_THRESHOLD || roll_change > SPIKE_THRESHOLD) {
            imu_spike_count++;
            
            if (imu_spike_count < 3) {
                pitch_raw = last_valid_pitch;
                roll_raw = last_valid_roll;
            } else {
                last_valid_pitch = pitch_raw;
                last_valid_roll = roll_raw;
                imu_spike_count = 0;
            }
        } else {
            last_valid_pitch = pitch_raw;
            last_valid_roll = roll_raw;
            imu_spike_count = 0;
        }
        
        // ============ LỌC LOW-PASS ============
        double alpha = 0.3;
        pitch_filtered = alpha * pitch_raw + (1.0 - alpha) * pitch_filtered;
        roll_filtered = alpha * roll_raw + (1.0 - alpha) * roll_filtered;
        
        // ============ TÍNH ĐẠO HÀM với LIMITER ============
        double pitch_deriv_raw = (pitch_filtered - pitch_prev) / 0.01;
        double roll_deriv_raw = (roll_filtered - roll_prev) / 0.01;
        
        const double MAX_DERIVATIVE = 50.0;
        pitch_derivative = std::clamp(pitch_deriv_raw, -MAX_DERIVATIVE, MAX_DERIVATIVE);
        roll_derivative = std::clamp(roll_deriv_raw, -MAX_DERIVATIVE, MAX_DERIVATIVE);
        
        pitch_prev = pitch_filtered;
        roll_prev = roll_filtered;
        ppp = pitch_filtered;
        rrr = roll_filtered;
    }
     
    ///////////////////////////////////////////////////////////////////////////////////
    //// RL PARAMETER UPDATE - Nhận 7 tham số từ RL ////
    ///////////////////////////////////////////////////////////////////////////////////
    void rl_param_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        // Nhận 7 tham số: [Gain, Scale, StepDur, LandPhase, StanceW, FootH, Recovery]
        if (msg->data.size() >= 7) {
            gain = msg->data[0];
            scale_base = msg->data[1];
            fwctEnd = msg->data[2];
            landing_phase = msg->data[3];
            stance_width = msg->data[4];
            fhMax = msg->data[5];
            rr = msg->data[6];  // Recovery Rate
            
            static int rl_callback_cnt = 0;
            // if (++rl_callback_cnt % 100 == 0) {
            //     RCLCPP_INFO(this->get_logger(),
            //         "[RL] G=%.2f S=%.2f D=%.0f L=%.0f W=%.0f H=%.0f R=%.3f",
            //         gain, scale_base, fwctEnd, landing_phase, stance_width, fhMax, rr);
            // }
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////
    //// RESET CALLBACK ////
    ///////////////////////////////////////////////////////////////////////////////////
    void reset_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            // ============ TRUE: CHUẨN BỊ RESET ============
            // RCLCPP_INFO(this->get_logger(), "┌─────────────────────────────────────┐");
            // RCLCPP_INFO(this->get_logger(), "│  RESET - Entering Safe Mode         │");
            // RCLCPP_INFO(this->get_logger(), "└─────────────────────────────────────┘");
            
            mode = -1;
            
            // Reset biến UVC
            fwct = 0.0;
            fh = 0.0;
            support_leg = 0;
            
            // Reset vị trí
            dxi = 0.0;
            dyi = default_stance_width;
            dxis = 0.0;
            dyis = -default_stance_width;
            dxi_before = dxi;
            dyi_before = dyi;
            
            // [QUAN TRỌNG] Reset autoH về giá trị ban đầu
            autoH = RESET_HEIGHT;  // 205mm
            
            // Reset biến IMU
            pitch = 0.0;
            roll = 0.0;
            pitch_filtered = 0.0;
            roll_filtered = 0.0;
            pitch_prev = 0.0;
            roll_prev = 0.0;
            pitch_derivative = 0.0;
            roll_derivative = 0.0;
            last_tilt_magnitude = 0.0;
            
            // Reset bộ lọc spike
            imu_spike_count = 0;
            last_valid_pitch = 0.0;
            last_valid_roll = 0.0;
            
            // Reset offset
            pitch_offset = 0.0;
            roll_offset = 0.0;
            
            calibration_samples = 0;
            stable_count = 0;
            
            publish_safe_stance();

        } else {
            // ============ FALSE: BẮT ĐẦU EPISODE ============
            if (mode == -1) {
                // RCLCPP_INFO(this->get_logger(), "┌─────────────────────────────────────┐");
                // RCLCPP_INFO(this->get_logger(), "│  EPISODE START - Warmup 500ms       │");
                // RCLCPP_INFO(this->get_logger(), "└─────────────────────────────────────┘");
                
                pitch_offset = 0.0;
                roll_offset = 0.0;
                
                warmup_counter = 50;  // 500ms
                mode = 1;
                
                // RCLCPP_INFO(this->get_logger(), "→ autoH: %.1f mm | Ready!", autoH);
            }
        }
    }
    
    ///////////////////////////////////////////////////////////////////////////////////
    //// CONTROL LOOP ////
    ///////////////////////////////////////////////////////////////////////////////////
    void control_loop() {
        // ============ WARMUP PHASE ============
        if (warmup_counter > 0) {
            warmup_counter--;
            
            if (warmup_counter == 1) {
                // Reset sạch biến IMU
                pitch = 0.0;
                roll = 0.0;
                pitch_filtered = 0.0;
                roll_filtered = 0.0;
                pitch_prev = 0.0;
                roll_prev = 0.0;
                pitch_derivative = 0.0;
                roll_derivative = 0.0;
                
                ppp = 0.0;
                rrr = 0.0;
                imu_spike_count = 0;
                last_valid_pitch = 0.0;
                last_valid_roll = 0.0;
                
                // Vị trí giữ nguyên
                dxi = 0.0;
                dyi = default_stance_width;
                dxis = 0.0;
                dyis = -default_stance_width;
                dxi_before = dxi;
                dyi_before = dyi;
                
                // autoH GIỮ NGUYÊN = RESET_HEIGHT (205mm)
                
                // RCLCPP_INFO(this->get_logger(), "✓ Warmup Done | autoH=%.1f mm", autoH);
            }
            
            publish_safe_stance();
            return;
        }
        
        // ============ STATE MACHINE ============
        switch(mode) {
            case -1:
                publish_safe_stance();
                break;
            case 0:
                break;
            case 1:
                standing_mode();
                break;
            case 2:
                uvc_active_mode();
                break;
        }
    }
    
    ///////////////////////////////////////////////////////////////////////////////////
    //// STANDING MODE ////
    ///////////////////////////////////////////////////////////////////////////////////
    void standing_mode() {
        pitch = pitch_filtered - pitch_offset;
        roll = roll_filtered - roll_offset;
        
        if (std::abs(pitch) < 0.017 && std::abs(roll) < 0.017) {
            stable_count++;
            if (stable_count > 50) {
                dxi = 0.0;
                dyi = default_stance_width;
                dxis = 0.0;
                dyis = -default_stance_width;
                dxi_before = dxi;
                dyi_before = dyi;
                fwct = 0.0;
                support_leg = 0;
                
                // Hồi phục chiều cao từ từ về HEIGHT_STD
                if (autoH < HEIGHT_STD) {
                    autoH += (HEIGHT_STD - autoH) * 0.01;
                } else if (autoH > HEIGHT_STD) {
                    autoH -= (autoH - HEIGHT_STD) * 0.01;
                }
                
                publish_safe_stance();
                return;
            }
        } else {
            stable_count = 0;
        }
        
        double tilt_magnitude = std::sqrt(pitch * pitch + roll * roll);
        if (tilt_magnitude > tilt_threshold) {
            mode = 2;
            fwct = 0.0;
            
            if (roll > 0) {
                support_leg = 1;
                dyi = stance_width;
                dyis = -stance_width;
            } else {
                support_leg = 0;
                dyi = stance_width;
                dyis = -stance_width;
            }
            
            // RCLCPP_INFO(this->get_logger(), "⚡ UVC ON | Tilt: %.1f° | Leg: %s",
            //            tilt_magnitude * 180.0 / M_PI,
            //            support_leg == 0 ? "R" : "L");
        }
        
        publish_safe_stance();
    }
    
    ///////////////////////////////////////////////////////////////////////////////////
    //// UVC ACTIVE MODE ////
    ///////////////////////////////////////////////////////////////////////////////////
    void uvc_active_mode() {
        pitch = pitch_filtered - pitch_offset;
        roll = roll_filtered - roll_offset;
        
        double threshold_offset = 0.033;
        if (std::abs(pitch) < threshold_offset && std::abs(roll) < threshold_offset) {
            pitch = 0.0;
            roll = 0.0;
        } else {
            double k = std::sqrt(pitch * pitch + roll * roll);
            if (k > threshold_offset) {
                k = (k - threshold_offset) / k;
                pitch *= k;
                roll *= k;
            }
        }
        
        double tilt_magnitude = std::sqrt(pitch * pitch + roll * roll);
        last_tilt_magnitude = tilt_magnitude;
        
        dyi_before = dyi;
        dxi_before = dxi;
        apply_uvc_geometry_control();
        
        calculate_foot_lift();
        calculate_and_publish_legs();
        update_cycle_counter();
        publish_rl_feedback();
        
        static int debug_counter = 0;
        // if (debug_counter++ % 20 == 0) {
        //     RCLCPP_INFO(this->get_logger(), 
        //             //    "[UVC] t=%.0f/%.0f | H=%.0f | p=%.1f° r=%.1f°",
        //                fwct, fwctEnd, autoH, 
        //                ppp * 180.0 / M_PI, rrr * 180.0 / M_PI);
        // }
        
        if (fwct >= fwctEnd * 0.8) {
            if (tilt_magnitude < tilt_threshold * 0.5) {
                mode = 1;
                stable_count = 0;
                // RCLCPP_INFO(this->get_logger(), "→ Standing Mode");
            }
        }
    }
    
    ///////////////////////////////////////////////////////////////////////////////////
    //// UVC GEOMETRY CONTROL ////
    ///////////////////////////////////////////////////////////////////////////////////
    void apply_uvc_geometry_control() {
        double tilt_mag = std::sqrt(pitch * pitch + roll * roll);
        double scale_factor = this->scale_base * (1.0 + 0.8 * std::min(tilt_mag, 1.0));
        
        // ============ ROLL ============
        double k = std::atan(dyi / autoH);
        double kl = autoH / std::cos(k);
        
        double p_term_roll = scale_factor * roll;
        double d_term_roll = this->gain * this->roll_derivative;
        
        double ks = k - (p_term_roll + d_term_roll);
        dyi = kl * std::sin(ks);
        autoH = kl * std::cos(ks);
        
        // ============ PITCH ============
        k = std::atan(dxi / autoH);
        kl = autoH / std::cos(k);
        
        double p_term_pitch = scale_factor * pitch;
        double d_term_pitch = this->gain * this->pitch_derivative;
        
        ks = k - (p_term_pitch + d_term_pitch);
        dxi = kl * std::sin(ks);
        autoH = kl * std::cos(ks);
        
        // ============ LIMITER ============
        double max_change = 0.5 + (tilt_mag * 10.0);
        
        dxi = std::clamp(dxi, dxi_before - max_change, dxi_before + max_change);
        dyi = std::clamp(dyi, dyi_before - max_change, dyi_before + max_change);
        
        dxi = std::clamp(dxi, -70.0, 70.0);
        dyi = std::clamp(dyi, -30.0, 65.0);
        
        dyis = dyi;
        dxis = -dxi;
        
        // ============ DYNAMIC HEIGHT ============
        double leg_extension_sq = dxi * dxi + dyi * dyi;
        double max_leg_length = 225.0;
        double max_safe_H_sq = max_leg_length * max_leg_length - leg_extension_sq;
        
        if (max_safe_H_sq < 0) {
            max_safe_H_sq = 100.0;
        }
        
        double max_possible_H = std::sqrt(max_safe_H_sq) - 2.0;
        
        // [QUAN TRỌNG] Hồi phục dùng tham số rr từ RL
        if (autoH < HEIGHT_STD) {
            autoH += (HEIGHT_STD - autoH) * this->rr;
        }
        
        if (autoH > max_possible_H) {
            autoH = max_possible_H;
        }
        
        if (autoH < 140.0) {
            autoH = 140.0;
        }
    }
    
    ///////////////////////////////////////////////////////////////////////////////////
    //// FOOT LIFT ////
    ///////////////////////////////////////////////////////////////////////////////////
    void calculate_foot_lift() {
        double landF = landing_phase;
        double landB = landing_phase;
        
        if (fwct > landF && fwct <= (fwctEnd - landB)) {
            double swing_progress = (fwct - landF) / (fwctEnd - (landF + landB));
            fh = fhMax * std::sin(M_PI * swing_progress);
            
            if (autoH - fh < min_clearance) {
                fh = std::max(0.0, autoH - min_clearance);
            }
        } else {
            fh = 0.0;
        }
    }
    
    ///////////////////////////////////////////////////////////////////////////////////
    //// LEGS CALCULATION ////
    ///////////////////////////////////////////////////////////////////////////////////
    void calculate_and_publish_legs() {
        double l_hn, l_ht, l_dg, l_mct, l_mcn;
        double r_hn, r_ht, r_dg, r_mct, r_mcn;
        
        double swing_dz_left = autoH;
        double swing_dz_right = autoH;
        
        if (fh > 0.0) {
            swing_dz_left = std::max(autoH - fh, min_clearance);
            swing_dz_right = std::max(autoH - fh, min_clearance);
        }
        
        double swing_end = fwctEnd - landing_phase;
        if (fwct >= swing_end) {
            double landing_progress = 0.0;
            double landing_duration = fwctEnd - swing_end;
            
            if (landing_duration > 0.0) {
                landing_progress = std::clamp((fwct - swing_end) / landing_duration, 0.0, 1.0);
            } else {
                landing_progress = 1.0;
            }
            
            if (support_leg == 0) {
                swing_dz_left = autoH + (swing_dz_left - autoH) * (1.0 - landing_progress);
            } else {
                swing_dz_right = autoH + (swing_dz_right - autoH) * (1.0 - landing_progress);
            }
        }
        
        double support_dz = autoH;
        double sway_offset = 15.0;
        
        if (support_leg == 0) {
            double support_dy_right = dyi - sway_offset;
            double swing_dy_left = dyis;
            
            solve_ik(dxi, support_dy_right, support_dz, r_hn, r_ht, r_dg, r_mct, r_mcn);
            solve_ik(dxis, swing_dy_left, swing_dz_left, l_hn, l_ht, l_dg, l_mct, l_mcn);
        } else {
            double support_dy_left = dyis + sway_offset;
            double swing_dy_right = dyi;
            
            solve_ik(dxis, support_dy_left, support_dz, l_hn, l_ht, l_dg, l_mct, l_mcn);
            solve_ik(dxi, swing_dy_right, swing_dz_right, r_hn, r_ht, r_dg, r_mct, r_mcn);
        }
        
        publish_legs(l_hn, l_ht, l_dg, l_mct, l_mcn, r_hn, r_ht, r_dg, r_mct, r_mcn);
    }
    
    ///////////////////////////////////////////////////////////////////////////////////
    //// CYCLE COUNTER ////
    ///////////////////////////////////////////////////////////////////////////////////
    void update_cycle_counter() {
        fwct += fwctUp;
        
        if (fwct >= fwctEnd) {
            support_leg ^= 1;
            fwct = 0.0;
            fh = 0.0;
            
            std::swap(dxi, dxis);
            std::swap(dyi, dyis);
            
            if (support_leg == 0) {
                dyi = stance_width;
                dyis = -stance_width;
            } else {
                dyis = -stance_width;
                dyi = stance_width;
            }
            
            dxi_before = dxi;
            dyi_before = dyi;
        }
    }
    
    ///////////////////////////////////////////////////////////////////////////////////
    //// SAFE STANCE ////
    ///////////////////////////////////////////////////////////////////////////////////
    void publish_safe_stance() {
        double l_hn, l_ht, l_dg, l_mct, l_mcn;
        double r_hn, r_ht, r_dg, r_mct, r_mcn;
        
        // Dùng autoH hiện tại (205mm ban đầu, sau đó thay đổi)
        solve_ik(0.0, stance_width, autoH, l_hn, l_ht, l_dg, l_mct, l_mcn);
        solve_ik(0.0, -stance_width, autoH, r_hn, r_ht, r_dg, r_mct, r_mcn);
        
        publish_legs(l_hn, l_ht, l_dg, l_mct, l_mcn,
                    r_hn, r_ht, r_dg, r_mct, r_mcn);
    }
    
    ///////////////////////////////////////////////////////////////////////////////////
    //// RL FEEDBACK ////
    ///////////////////////////////////////////////////////////////////////////////////
    void publish_rl_feedback() {
        auto msg = geometry_msgs::msg::Vector3();
        msg.x = pitch * 180.0 / M_PI;
        msg.y = roll * 180.0 / M_PI;
        msg.z = fwct / fwctEnd;
        
        rl_feedback_pub_->publish(msg);
    }
    
    ///////////////////////////////////////////////////////////////////////////////////
    //// INVERSE KINEMATICS ////
    ///////////////////////////////////////////////////////////////////////////////////
    void solve_ik(double dx, double dy, double dz, 
                  double &hn, double &ht, double &dg, double &mct, double &mcn) {
        hn = std::atan2(dy, dz);
        double d_yz = std::sqrt(dz * dz + dy * dy);
        
        double cos3 = (std::pow(d_yz - L5, 2) + dx * dx - L3 * L3 - L4 * L4) / (2.0 * L3 * L4);
        cos3 = std::clamp(cos3, -1.0, 1.0);
        
        double sin3 = std::sqrt(1.0 - cos3 * cos3);
        dg = std::atan2(sin3, cos3);
        ht = std::atan2(sin3 * L4, L3 + cos3 * L4) + std::atan2(dx, d_yz - L5);
        mct = -dg + ht;
        mcn = -hn;
    }
    
    ///////////////////////////////////////////////////////////////////////////////////
    //// PUBLISH LEGS ////
    ///////////////////////////////////////////////////////////////////////////////////
    void publish_legs(double l_hn, double l_ht, double l_dg, double l_mct, double l_mcn,
                      double r_hn, double r_ht, double r_dg, double r_mct, double r_mcn) {
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
    
    void send_cmd(const std::string& key, double value) {
        if (pubs_.count(key)) {
            auto msg = std_msgs::msg::Float64();
            msg.data = value;
            pubs_[key]->publish(msg);
        }
    }
    
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr create_joint_pub(const std::string& joint_name) {
        return this->create_publisher<std_msgs::msg::Float64>(
            "/model/humanoid_robot/joint/" + joint_name + "/cmd_pos", 10);
    }
    
    // =================== MEMBER VARIABLES ===================
    double gain, rr, fwctEnd, stance_width, fhMax;
    double tilt_threshold, tilt_threshold_deg, landing_phase;
    double min_clearance, scale_base;
    
    double fwct, fwctUp;
    double autoH, fh;
    double dxi, dyi, dxis, dyis;
    double dxib, dyib;
    double dxi_before, dyi_before;
    int support_leg;
    
    double pitch, roll;
    double pitch_filtered, roll_filtered;
    double pitch_prev, roll_prev;
    double pitch_derivative, roll_derivative;
    double pitch_offset, roll_offset;
    double last_tilt_magnitude;
    double ppp, rrr;
    
    int imu_spike_count;
    double last_valid_pitch, last_valid_roll;
    
    int mode;
    int calibration_samples;
    int stable_count;
    int warmup_counter;
    double default_stance_width;
    double default_gain;
    
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
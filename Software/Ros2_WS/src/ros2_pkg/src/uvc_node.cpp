#include "std_msgs/msg/float64_multi_array.hpp"  // THÊM DÒNG NÀY
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>
#include <algorithm>
#include <map>
#include <string>
#include <memory>
#include <chrono>  // Thêm dòng này nếu chưa có
#include <thread>  // Thêm dòng này nếu chưa có
const double L3 = 60.0;
const double L4 = 100.0;
const double L5 = 65.0;
const double HEIGHT_STD = 224.0-20;

class UvcControllerNode : public rclcpp::Node {
public:
    UvcControllerNode() : Node("uvc_node") {
        // =================== CÁC THAM SỐ CÓ THỂ TINH CHỈNH CHO RL ===================
        declare_parameter("gain", 0.18);
        declare_parameter("recovery_rate", 0.1);
        declare_parameter("step_duration", 25.0);
        
        // [THAY ĐỔI QUAN TRỌNG] Giảm từ 35.0 xuống 30.0 để tránh chân bị loe (nghiêng)
        declare_parameter("stance_width", 20.0);
        
        declare_parameter("max_foot_lift", 12.0);
        declare_parameter("scale_base", 0.12);
        declare_parameter("tilt_threshold", 2.0);
        declare_parameter("landing_phase", 6.0);
        declare_parameter("min_clearance", 18.0);
        
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
        
        // Chuyển đổi ngưỡng sang radian
        tilt_threshold = tilt_threshold_deg * M_PI / 180.0;
        
        // =================== KHỞI TẠO BIẾN TRẠNG THÁI ===================
        // Biến UVC
        fwct = 0.0;
        fwctUp = 1.0;
        autoH = HEIGHT_STD;
        fh = 0.0;
        
        // Tích phân vị trí - khởi tạo ở tư thế đứng song song
        dxi = 15.0;      
        dyi = stance_width;  
        dxis = 15.0;     
        dyis = -stance_width; 

        dxi_before=dxi;
        dyi_before=dyi;
        
        dxib = 0.0;
        dyib = 0.0;
        
        // Chân trụ: 0 = phải, 1 = trái
        support_leg = 0;  
        
        // Biến IMU và lọc
        pitch = 0.0;
        roll = 0.0;
        pitch_filtered = 0.0;
        roll_filtered = 0.0;
        pitch_prev = 0.0;
        roll_prev = 0.0;
        last_tilt_magnitude = 0.0;
        
        // Offset hiệu chuẩn
        pitch_offset = 0.0;
        roll_offset = 0.0;
        
        // Trạng thái: Khởi tạo mode = 0
        mode = 0;  
        calibration_samples = 0;
        stable_count = 0;
        
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
        
        RCLCPP_INFO(this->get_logger(), "UVC FAST RESET MODE - READY");
    }

private:
    void imu_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        // Đọc và chuyển đổi dữ liệu IMU
        double pitch_raw = -msg->x * M_PI / 180.0;
        double roll_raw = -msg->y * M_PI / 180.0;
        
        // Lọc low-pass để giảm nhiễu
        double alpha = 0.4;
        pitch_filtered = alpha * pitch_raw + (1 - alpha) * pitch_filtered;
        roll_filtered = alpha * roll_raw + (1 - alpha) * roll_filtered;
        
        // Tính đạo hàm (tốc độ thay đổi) cho D controller
        pitch_derivative = (pitch_filtered - pitch_prev) / 0.01;  // 0.05s = 50ms
        roll_derivative = (roll_filtered - roll_prev) / 0.01;
        
        pitch_prev = pitch_filtered;
        roll_prev = roll_filtered;
        ppp=pitch_filtered;
        rrr=roll_filtered;

    }
     
    ///////////////////////////////////////////////////////////////////////////////////
    //// RL PARAMETER UPDATE - Nhận tham số tối ưu từ RL training node ////
    ///////////////////////////////////////////////////////////////////////////////////
    void rl_param_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        // Nhận 7 tham số từ RL: [gain, scale_base, step_duration, landing_phase, stance_width, max_foot_lift, recovery_rate]
        if (msg->data.size() >= 7) {
            gain = msg->data[0];
            scale_base = msg->data[1];
            fwctEnd = msg->data[2];
            landing_phase = msg->data[3];
            stance_width = msg->data[4];
            fhMax = msg->data[5];
            rr = msg->data[6];
            
            // Log mỗi 100 lần nhận (mỗi ~5 giây ở 20Hz)
            static int rl_callback_cnt = 0;
            if (++rl_callback_cnt % 100 == 0) {
                RCLCPP_INFO(this->get_logger(),
                    "[RL UPDATE] gain=%.3f scale=%.3f step_dur=%.1f land_phase=%.1f stance=%.1f fh_max=%.1f rr=%.3f",
                    gain, scale_base, fwctEnd, landing_phase, stance_width, fhMax, rr);
            }
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////
    //// RESET CALLBACK - Xử lý lệnh reset từ RL training ////
    ///////////////////////////////////////////////////////////////////////////////////
    void reset_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            // [NHẬN TÍN HIỆU TRUE]: CHUẨN BỊ RESET
            RCLCPP_INFO(this->get_logger(), "========================================");
            RCLCPP_INFO(this->get_logger(), "RESET REQUEST: Holding Joints Stiff (Wait for Physics)");
            
            // 1. Chuyển sang chế độ CHỜ (-1) thay vì Calibrate ngay
            mode = -1; 
            
            // 2. Reset các biến bước đi
            fwct = 0.0;
            fh = 0.0;
            support_leg = 0;
            
            // 3. Reset vị trí chân về thế đứng chuẩn
            dxi = 15.0; dyi = stance_width;
            dxis = 15.0; dyis = -stance_width;
            dxib = 0.0; dyib = 0.0;
            autoH = HEIGHT_STD; // <--- THÊM DÒNG NÀY (Khoảng dòng 130)
            // 4. Reset biến IMU
            pitch = 0.0; roll = 0.0;
            pitch_filtered = 0.0; roll_filtered = 0.0;
            pitch_prev = 0.0; roll_prev = 0.0;
            last_tilt_magnitude = 0.0;
            dxi_before=dxi;
            dyi_before=dyi;
            // 5. Reset các bộ đếm
            calibration_samples = 0;
            stable_count = 0;
            
            // [QUAN TRỌNG] Ép robot duỗi thẳng chân ngay lập tức để giữ cứng khớp khi rơi
            publish_parallel_stance();

        } else {
            // [NHẬN TÍN HIỆU FALSE]: BẮT ĐẦU TRAINING
            if (mode == -1) {
                // [THAY ĐỔI QUAN TRỌNG]: Bỏ qua Calibration (Fast Start)
                // Vì Simulation Reset luôn đưa robot về thẳng đứng tuyệt đối
                pitch_offset = 0.0;
                roll_offset = 0.0;
                
                // Nhảy thẳng vào Mode 1 (Đứng chờ)
                mode = 1; 
                RCLCPP_INFO(this->get_logger(), "FAST START: Calibration skipped. Offsets=0.0 -> Ready!");
            }
        }
    }   
    void control_loop() {
        switch(mode) {
            case -1: // [MỚI] CHẾ ĐỘ CHỜ VẬT LÝ ỔN ĐỊNH
                // Liên tục gửi lệnh đứng thẳng để giữ khớp cứng, chống đổ khi vừa thả xuống
                publish_parallel_stance();
                break;
            case 0: // Hiệu chuẩn (Không dùng trong Fast Reset, nhưng giữ để tránh lỗi logic)
                // calibration_mode();
                break;
            case 1: // Đứng chờ nghiêng
                standing_mode();
                break;
            case 2: // UVC hoạt động
                uvc_active_mode();
                break;
        }
    }
    ///////////////////////////////////////////////////////////////////////////////////
    //// CALIBRATION MODE - Đo offset IMU ban đầu ////
    ///////////////////////////////////////////////////////////////////////////////////
    void calibration_mode() {
        // Thu thập mẫu để hiệu chuẩn offset
        if (calibration_samples < 100) {
            pitch_offset += pitch_filtered;
            roll_offset += roll_filtered;
            calibration_samples++;
            
            // Giữ nguyên tư thế đứng trong khi hiệu chuẩn
            publish_parallel_stance();
            
            if (calibration_samples % 20 == 0) {
                RCLCPP_INFO(this->get_logger(), "Calibrating... %d/100", calibration_samples);
            }
        } else {
            // Tính offset trung bình
            pitch_offset /= 100.0;
            roll_offset /= 100.0;
            mode = 1;
            
            RCLCPP_INFO(this->get_logger(), "Calibration complete!");
            RCLCPP_INFO(this->get_logger(), "Pitch offset: %.3f rad (%.1f deg)", 
                       pitch_offset, pitch_offset * 180.0 / M_PI);
            RCLCPP_INFO(this->get_logger(), "Roll offset: %.3f rad (%.1f deg)", 
                       roll_offset, roll_offset * 180.0 / M_PI);
        }
    }
    
    ///////////////////////////////////////////////////////////////////////////////////
    //// STANDING MODE - Chờ và phát hiện nghiêng để kích hoạt UVC ////
    ///////////////////////////////////////////////////////////////////////////////////
    void standing_mode() {
        // Tính góc nghiêng đã hiệu chuẩn
        pitch = pitch_filtered - pitch_offset;
        roll = roll_filtered - roll_offset;
        
        // Kiểm tra nếu đủ ổn định
        if (fabs(pitch) < 0.017 && fabs(roll) < 0.017) { // ~1 độ
            stable_count++;
            if (stable_count > 50) { // 2.5 giây ổn định
                // Reset vị trí tích phân về stance song song
                dxi = 15.0;
                dyi = stance_width;
                dxis = 15.0;
                dyis = -stance_width;
                fwct = 0.0;
                support_leg = 0;
                dxi_before=dxi;
                dyi_before=dyi;
                
                // Publish tư thế đứng song song
                publish_parallel_stance();
                return;
            }
        } else {
            stable_count = 0;
        }
        
        // Kiểm tra nếu bị nghiêng đủ để kích hoạt UVC
        double tilt_magnitude = sqrt(pitch * pitch + roll * roll);
        if (tilt_magnitude > tilt_threshold) {
            mode = 2;
            fwct = 0.0;
            
            // Xác định chân trụ dựa trên hướng nghiêng
            // Nghiêng phải (roll dương) -> chân TRÁI trụ (vì trọng tâm sang phải)
            // Nghiêng trái (roll âm) -> chân PHẢI trụ
            if (roll > 0) {
                support_leg = 1; // Chân trái trụ
            } else {
                support_leg = 0; // Chân phải trụ
            }
            
            // RCLCPP_INFO(this->get_logger(), "UVC Activated! Tilt: %.1f deg, Support leg: %s",
            //            tilt_magnitude * 180.0 / M_PI,
            //            support_leg == 0 ? "RIGHT" : "LEFT");
        }
        
        // Vẫn publish tư thế đứng trong khi chờ
        publish_parallel_stance();
    }
    
    ///////////////////////////////////////////////////////////////////////////////////
    //// UVC ACTIVE MODE - Điều khiển UVC chính để cân bằng ////
    ///////////////////////////////////////////////////////////////////////////////////
    void uvc_active_mode() {
        // 1. Tính góc nghiêng đã hiệu chuẩn
        pitch = pitch_filtered - pitch_offset;
        roll = roll_filtered - roll_offset;
        
        // DEBUG: In ra giá trị IMU mỗi 100ms
        static int dbg_imu_cnt = 0;
        if (++dbg_imu_cnt % 2 == 0) {
            // RCLCPP_INFO(this->get_logger(), 
            //            "[IMU RAW] pitch=%.2f° roll=%.2f° | pitch_offset=%.2f° roll_offset=%.2f°",
            //            pitch * 180.0/M_PI, roll * 180.0/M_PI,
            //            pitch_offset * 180.0/M_PI, roll_offset * 180.0/M_PI);
        }
        
        // 2. Áp dụng offset ngưỡng (giống main.c) - nếu góc < 0.033 rad thì coi như 0
        double threshold_offset = 0.033;
        if (fabs(pitch) < threshold_offset && fabs(roll) < threshold_offset) {
            pitch = 0.0;
            roll = 0.0;
        } else {
            // Rút gọn góc nếu vượt ngưỡng (normalize)
            double k = sqrt(pitch*pitch + roll*roll);
            if (k > threshold_offset) {
                k = (k - threshold_offset) / k;
                pitch *= k;
                roll *= k;
            }
        }
        
        // 3. Lưu tilt magnitude để tính fh adaptive
        double tilt_magnitude = sqrt(pitch * pitch + roll * roll);
        last_tilt_magnitude = tilt_magnitude;
        
        // DEBUG: In ra sau offset
        if (dbg_imu_cnt % 2 == 0) {
            // RCLCPP_INFO(this->get_logger(), 
                    //    "[AFTER THRESHOLD] pitch=%.2f° roll=%.2f° | tilt_mag=%.2f°",
                    //    pitch * 180.0/M_PI, roll * 180.0/M_PI,
                    //    tilt_magnitude * 180.0/M_PI);
        }
        
        // 4. Áp dụng UVC logic (giống main.c)
        dyi_before = dyi;
        dxi_before = dxi;
        apply_uvc_geometry_control();
        
        // DEBUG: In ra sau UVC
        if (dbg_imu_cnt % 2 == 0) {
            // RCLCPP_INFO(this->get_logger(), 
            //            "[AFTER UVC] dyi: %.1f→%.1f | dxi: %.1f→%.1f | autoH=%.1f",
            //            dyi_before, dyi, dxi_before, dxi, autoH);
        }
        
        // 5. Tính độ cao nâng chân theo chu kỳ
        calculate_foot_lift();
        
        // 6. Tính toán và publish vị trí chân
        calculate_and_publish_legs();
        
        // 7. Cập nhật bộ đếm chu kỳ
        update_cycle_counter();
        
        // 8. Gửi feedback cho RL (tùy chọn)
        publish_rl_feedback();
        
        // Debug thông tin chính
        static int debug_counter = 0;
        if (debug_counter++ % 10 == 0) {
            RCLCPP_INFO(this->get_logger(), 
                       "[CYCLE] phase=%.1f/%.1f, leg=%s, fh=%.1f, pitch =%.2f rad, roll=%.2f rad",
                       fwct, fwctEnd,
                       support_leg == 0 ? "RIGHT" : "LEFT",
                       fh, ppp, rrr);
        }
        
        // Kiểm tra nếu đã đủ ổn định để quay lại đứng
        if (fwct >= fwctEnd * 0.8) {
            double tilt_magnitude = sqrt(pitch * pitch + roll * roll);
            if (tilt_magnitude < tilt_threshold * 0.5) {
                mode = 1;
                stable_count = 0;
                RCLCPP_INFO(this->get_logger(), "Returning to standing mode");
            }
        }
    }
    
    ///////////////////////////////////////////////////////////////////////////////////
    //// UVC GEOMETRY CONTROL - Điều chỉnh vị trí chân dùng hình học (giống main.c) ////
    ///////////////////////////////////////////////////////////////////////////////////
    void apply_uvc_geometry_control() {
        // 1. Tính toán UVC như bình thường
        double k = atan((dyi) / autoH);
        double kl = autoH / cos(k);
        double tilt_mag = sqrt(pitch * pitch + roll * roll);
        double scale_factor = this->scale_base * (1.0 + 0.8 * std::min(tilt_mag, 1.0));
        
        // ROLL Control
        double p_term_roll = scale_factor * roll;
        double d_term_roll = this->gain * this->roll_derivative; 
        double ks = k - (p_term_roll + d_term_roll);
        dyi = kl * sin(ks);
        autoH = kl * cos(ks);
        
        // PITCH Control
        k = atan(dxi / autoH);
        kl = autoH / cos(k);
        double p_term_pitch = scale_factor * pitch;
        double d_term_pitch = this->gain * this->pitch_derivative; 
        ks = k - (p_term_pitch + d_term_pitch); 
        dxi = kl * sin(ks);
        autoH = kl * cos(ks);
        
        // 2. LIMITER: Cho phép bước dài (nhưng cẩn thận)
        double max_change = 0.5 + (tilt_mag * 10.0);
        if (dxi > dxi_before + max_change) dxi = dxi_before + max_change;
        if (dxi < dxi_before - max_change) dxi = dxi_before - max_change;
        if (dyi > dyi_before + max_change) dyi = dyi_before + max_change;
        if (dyi < dyi_before - max_change) dyi = dyi_before - max_change;

        // Cho phép bước rất dài (cả âm và dương)
        if (dyi < -30.0) dyi = -30.0;
        if (dyi > 65.0) dyi = 65.0;
        if (dxi < -70.0) dxi = -70.0; 
        if (dxi > 70.0) dxi = 70.0;
        
        dyis = dyi;
        dxis = -dxi;
        
        // 3. [QUAN TRỌNG NHẤT] DYNAMIC HEIGHT ADJUSTMENT (ĐIỀU CHỈNH ĐỘ CAO ĐỘNG)
        // Tính độ dài chân cần thiết với dxi và dyi hiện tại
        double leg_extension_sq = dxi*dxi + dyi*dyi;
        double max_safe_H_sq = (225.0*225.0) - leg_extension_sq; // 225 là max chân
        
        // Nếu căn bậc 2 âm (tức là bước quá dài, chân không với tới), phải giới hạn
        if (max_safe_H_sq < 0) max_safe_H_sq = 100.0; // Giá trị an toàn thấp
        
        double max_possible_H = sqrt(max_safe_H_sq) - 2.0; // Trừ 2mm để an toàn
        
        // Logic hồi phục:
        // Cố gắng hồi phục về HEIGHT_STD (220mm)
        if (HEIGHT_STD > autoH) {
            autoH += (HEIGHT_STD - autoH) * this->rr;
        }
        
        // NHƯNG: Không bao giờ được cao hơn max_possible_H (giới hạn vật lý)
        // Nếu bước dài, max_possible_H sẽ giảm -> autoH bị ép giảm theo -> Robot tự hạ thấp
        if (autoH > max_possible_H) {
            autoH = max_possible_H;
        }
        
        // Giới hạn dưới cùng
        if (autoH < 140) autoH = 140;
    }
    // void apply_uvc_geometry_control() {
    // // ============ ĐIỀU CHỈNH Y (trái-phải) ============
    //     double k = atan((dyi) / autoH);
    //     double kl = autoH / cos(k);
        
    //     double tilt_mag = sqrt(pitch * pitch + roll * roll);
        
    //     // 1. P-TERM (Dựa trên góc nghiêng - Đã sửa lỗi scale_base cục bộ ở bước trước)
    //     double scale_factor = this->scale_base * (1.0 + 0.5 * std::min(tilt_mag, 1.0));
    //     double p_term_roll = scale_factor * roll;
        
    //     // 2. D-TERM (Dựa trên tốc độ nghiêng - SỬ DỤNG BIẾN GAIN TỪ RL)
    //     // gain đóng vai trò là Kd (Damping), giúp robot không bị lắc qua lắc lại
    //     double d_term_roll = this->gain * this->roll_derivative; 
        
    //     // Tổng hợp điều khiển: Góc sửa = P + D
    //     double roll_total_correction = p_term_roll + d_term_roll;

    //     // Áp dụng vào công thức UVC
    //     double ks = k - roll_total_correction;  // Dùng tổng hợp thay vì chỉ roll_scaled
        
    //     dyi = kl * sin(ks);
    //     autoH = kl * cos(ks);
        
    //     // ============ ĐIỀU CHỈNH X (trước-sau) ============
    //     k = atan(dxi / autoH);
    //     kl = autoH / cos(k);
        
    //     // Tương tự cho trục Pitch
    //     double p_term_pitch = scale_factor * pitch;
    //     double d_term_pitch = this->gain * this->pitch_derivative; // Sử dụng gain
        
    //     double pitch_total_correction = p_term_pitch + d_term_pitch;
        
    //     ks = k - pitch_total_correction; // Dùng tổng hợp
        
    //     dxi = kl * sin(ks);
    //     autoH = kl * cos(ks);
        
    //     // // DEBUG: In ra pitch geometry
    //     // if (dbg_geom_cnt % 2 == 0) {
    //     //     // RCLCPP_INFO(this->get_logger(),
    //     //     //            "[GEOM-X] pitch=%.3f rad | pitch_scaled=%.3f | k=%.4f ks=%.4f | kl=%.1f | dxi_new=%.1f autoH=%.1f",
    //     //     //            pitch, pitch_scaled, k, ks, kl, dxi, autoH);
    //     // }
        
    //     // ============ GIỚI HẠN AN TOÀN ============

    //     // double max_change = 0.5; // Chỉ cho phép thay đổi tối đa 2mm mỗi 10ms
    //     double emergency_multiplier = 1.0; 
    //     double max_change = 0.5 + (tilt_mag * emergency_multiplier);
    //     // Kẹp dxi trong khoảng [old - 2.0, old + 2.0]
    //     if (dxi > dxi_before + max_change) dxi = dxi_before + max_change;
    //     if (dxi < dxi_before - max_change) dxi = dxi_before - max_change;
        
    //     // Kẹp dyi tương tự
    //     if (dyi > dyi_before + max_change) dyi = dyi_before + max_change;
    //     if (dyi < dyi_before - max_change) dyi = dyi_before - max_change;

    //     if (dyi < -30) dyi = -30;
    //     if (dyi > 65) dyi = 65;
    //     if (dxi < -65) dxi = -65;
    //     if (dxi > 65) dxi = 65;
        
    //     dyis = dyi;
    //     dxis = -dxi;
        
    //     // ============ PHỤC HỒI CHIỀU CAO ============
    //     if (HEIGHT_STD > autoH) {
    //         autoH += (HEIGHT_STD - autoH) * this->rr;
    //     } else {
    //         autoH = HEIGHT_STD;
    //     }
        
    //     if (autoH < 140) autoH = 140;
    // }
    
    ///////////////////////////////////////////////////////////////////////////////////
    //// FOOT LIFT CALCULATION - Tính độ cao nâng chân theo hình sin ////
    ///////////////////////////////////////////////////////////////////////////////////
    void calculate_foot_lift() {
        // Tính độ cao nâng chân theo hình sin (giống main.c)
        // Chỉ nâng trong phần giữa của chu kỳ (swing phase)
        
        double landF = landing_phase;
        double landB = landing_phase;
        
        if (fwct > landF && fwct <= (fwctEnd - landB)) {
            // Phần swing: tính độ cao theo hình sin
            double swing_progress = (fwct - landF) / (fwctEnd - (landF + landB));
            fh = fhMax * sin(M_PI * swing_progress);
            
            // Đảm bảo chân di chuyển không thấp hơn min_clearance
            if (autoH - fh < min_clearance) {
                fh = std::max(0.0, autoH - min_clearance);
            }
        } else {
            fh = 0.0;
        }
    }

    void calculate_and_publish_legs() {
        double l_hn, l_ht, l_dg, l_mct, l_mcn;
        double r_hn, r_ht, r_dg, r_mct, r_mcn;
        
        // 1. Tính độ cao (Z)
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
            if (landing_duration > 0.0) landing_progress = std::clamp((fwct - swing_end) / landing_duration, 0.0, 1.0);
            else landing_progress = 1.0;

            if (support_leg == 0) swing_dz_left = autoH + (swing_dz_left - autoH) * (1.0 - landing_progress);
            else swing_dz_right = autoH + (swing_dz_right - autoH) * (1.0 - landing_progress);
        }

        double support_dz = autoH;

        // 2. Tính Sway (Y) - Dời trọng tâm
        // 15mm là khoảng cách đủ để trọng tâm rơi vào chân trụ
        double sway_offset = 15.0; 

        if (support_leg == 0) { // Chân PHẢI trụ
            // Dịch hông sang phải -> Tức là giảm dy của chân phải đi
            // dyi đang là 20, trừ 15 còn 5 -> Chân phải rất gần tâm -> Trọng tâm đè lên chân phải
            double support_dy_right = dyi - sway_offset; 
            double swing_dy_left = dyis;

            solve_ik(dxi, support_dy_right, support_dz, r_hn, r_ht, r_dg, r_mct, r_mcn);
            solve_ik(dxis, swing_dy_left, swing_dz_left, l_hn, l_ht, l_dg, l_mct, l_mcn);
            
        } else { // Chân TRÁI trụ
            // Dịch hông sang trái -> Tức là tăng dy của chân trái lên (đang âm cho về gần 0)
            double support_dy_left = dyis + sway_offset;
            double swing_dy_right = dyi;

            solve_ik(dxis, support_dy_left, support_dz, l_hn, l_ht, l_dg, l_mct, l_mcn);
            solve_ik(dxi, swing_dy_right, swing_dz_right, r_hn, r_ht, r_dg, r_mct, r_mcn);
        }
        
        publish_legs(l_hn, l_ht, l_dg, l_mct, l_mcn, r_hn, r_ht, r_dg, r_mct, r_mcn);
    }
    ///////////////////////////////////////////////////////////////////////////////////
    //// CYCLE COUNTER - Quản lý chu kỳ bước chân ////
    ///////////////////////////////////////////////////////////////////////////////////
    void update_cycle_counter() {
        fwct += fwctUp;
        
        if (fwct >= fwctEnd) {
            // Hoán đổi chân trụ
            support_leg ^= 1;
            fwct = 0.0;
            
            // Reset một số biến
            fh = 0.0;
            
            // Hoán đổi vị trí (đơn giản hóa)
            std::swap(dxi, dxis);
            std::swap(dyi, dyis);
            
            // Đảm bảo giữ đúng stance width
            if (support_leg == 0) {
                dyi = stance_width;
                dyis = -stance_width;
            } else {
                dyis = -stance_width;
                dyi = stance_width;
            }
            dxi_before = dxi;
            dyi_before = dyi;
            // RCLCPP_INFO(this->get_logger(), "Step cycle complete. New support leg: %s",
            //            support_leg == 0 ? "RIGHT" : "LEFT");
        }
    }
    
    ///////////////////////////////////////////////////////////////////////////////////
    //// STANCE PUBLISHER - Tư thế đứng song song (hai chân song song) ////
    ///////////////////////////////////////////////////////////////////////////////////
    void publish_parallel_stance() {
        // Tư thế đứng với hai chân SONG SONG
        double l_hn, l_ht, l_dg, l_mct, l_mcn;
        double r_hn, r_ht, r_dg, r_mct, r_mcn;
        
        // Chân trái: giữa, bên trái
        solve_ik(15.0, stance_width, HEIGHT_STD, l_hn, l_ht, l_dg, l_mct, l_mcn);
        // Chân phải: giữa, bên phải
        solve_ik(15.0, -stance_width, HEIGHT_STD, r_hn, r_ht, r_dg, r_mct, r_mcn);
        
        publish_legs(l_hn, l_ht, l_dg, l_mct, l_mcn,
                    r_hn, r_ht, r_dg, r_mct, r_mcn);
    }
    
    ///////////////////////////////////////////////////////////////////////////////////
    //// RL FEEDBACK - Gửi phản hồi cho Reinforcement Learning ////
    ///////////////////////////////////////////////////////////////////////////////////
    void publish_rl_feedback() {
        // Gửi feedback cho RL training
        auto msg = geometry_msgs::msg::Vector3();
        msg.x = pitch * 180.0 / M_PI;  // Góc pitch (độ)
        msg.y = roll * 180.0 / M_PI;   // Góc roll (độ)
        msg.z = fwct / fwctEnd;        // Tiến trình chu kỳ (0-1)
        
        rl_feedback_pub_->publish(msg);
    }
    
    ///////////////////////////////////////////////////////////////////////////////////
    //// INVERSE KINEMATICS - Tính góc khớp từ vị trí chân mong muốn ////
    ///////////////////////////////////////////////////////////////////////////////////
    // Input:  dx, dy, dz = vị trí chân (X: trước-sau, Y: trái-phải, Z: cao-thấp)
    // Output: hn, ht, dg, mct, mcn = góc các khớp
    //   hn:  base_hip (quay sang trái/phải)
    //   ht:  hip_hip (cong xương hông trước/sau)  
    //   dg:  hip_knee (cong đầu gối)
    //   mct: knee_ankle (cong mắt cá)
    //   mcn: ankle_ankle (xoay mắt cá)
    void solve_ik(double dx, double dy, double dz, 
                  double &hn, double &ht, double &dg, double &mct, double &mcn) {
        // Động học nghịch của bạn - GIỮ NGUYÊN
        hn = atan2(dy, dz);
        double d_yz = sqrt(dz*dz + dy*dy);
        double cos3 = (pow(d_yz - L5, 2) + dx*dx - L3*L3 - L4*L4) / (2*L3*L4);
        cos3 = std::clamp(cos3, -1.0, 1.0);
        double sin3 = sqrt(1.0 - cos3*cos3);
        dg = atan2(sin3, cos3);
        ht = atan2(sin3*L4, L3 + cos3*L4) + atan2(dx, d_yz - L5);
        mct = -dg + ht ;
        mcn = -hn;
    }
    
    ///////////////////////////////////////////////////////////////////////////////////
    //// PUBLISH LEGS - Gửi lệnh góc khớp đến các chân (với ánh xạ dấu L/R) ////
    ///////////////////////////////////////////////////////////////////////////////////
    void publish_legs(double l_hn, double l_ht, double l_dg, double l_mct, double l_mcn,
                      double r_hn, double r_ht, double r_dg, double r_mct, double r_mcn) {
        // Left Leg (mapping chosen to match URDF axes)
        // send_cmd("base_hip_left", l_hn);
        // send_cmd("hip_hip_left", -l_ht);
        // send_cmd("hip_knee_left", l_dg);
        // send_cmd("knee_ankle_left", l_mct);
        // send_cmd("ankle_ankle_left", l_mcn);

        send_cmd("base_hip_left", l_hn);
        send_cmd("hip_hip_left", -l_ht);
        send_cmd("hip_knee_left", l_dg);
        send_cmd("knee_ankle_left", l_mct);
        send_cmd("ankle_ankle_left", l_mcn);

        // Right Leg (mirrored mapping)
        send_cmd("base_hip_right", r_hn);
        send_cmd("hip_hip_right", r_ht);
        send_cmd("hip_knee_right", -r_dg);
        send_cmd("knee_ankle_right", -r_mct);
        send_cmd("ankle_ankle_right", -r_mcn);
    }
    
    ///////////////////////////////////////////////////////////////////////////////////
    //// COMMAND SENDER - Gửi giá trị góc đến driver động cơ ////
    ///////////////////////////////////////////////////////////////////////////////////
    void send_cmd(const std::string& key, double value) {
        if (pubs_.count(key)) {
            auto msg = std_msgs::msg::Float64();
            msg.data = value;
            pubs_[key]->publish(msg);
        }
    }
    
    ///////////////////////////////////////////////////////////////////////////////////
    //// PUBLISHER FACTORY - Tạo publisher cho từng khớp ////
    ///////////////////////////////////////////////////////////////////////////////////
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr create_joint_pub(const std::string& joint_name) {
        return this->create_publisher<std_msgs::msg::Float64>(
            "/model/humanoid_robot/joint/" + joint_name + "/cmd_pos", 10);
    }
    
    ///////////////////////////////////////////////////////////////////////////////////
    //// MEMBER VARIABLES - Trạng thái chương trình ////
    ///////////////////////////////////////////////////////////////////////////////////
    
    //// Biến tham số điều chỉnh ////
    double gain, rr, fwctEnd, stance_width, fhMax;
    double tilt_threshold, tilt_threshold_deg, landing_phase;
    double min_clearance;
    double scale_base;  // Thêm scale_base cho RL learning
    
    //// Biến trạng thái UVC - quản lý bước chân ////
    double fwct, fwctUp;
    double autoH, fh;
    double dxi, dyi;
    double dxis, dyis;
    double dxib, dyib;
    int support_leg;
    double dyi_before , dxi_before ;
    //// Biến IMU và điều khiển ////
    double pitch, roll;
    double pitch_filtered, roll_filtered;
    double pitch_prev, roll_prev;
    double pitch_derivative, roll_derivative;
    double pitch_offset, roll_offset;
    double last_tilt_magnitude;
    
    //// Trạng thái hệ thống ////
    int mode;
    int calibration_samples;
    int stable_count;
    double ppp,rrr;
    //// ROS interfaces ////
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> pubs_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr rl_feedback_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr angle_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rl_param_sub_;  // Subscribe RL params
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_sub_; // Subscribe to RL reset requests
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UvcControllerNode>());
    rclcpp::shutdown();
    return 0;
}
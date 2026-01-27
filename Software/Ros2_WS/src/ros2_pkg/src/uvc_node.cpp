#include "std_msgs/msg/float64_multi_array.hpp"  // THÊM DÒNG NÀY
// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/vector3.hpp"
// #include "std_msgs/msg/float64.hpp"
// #include <cmath>
// #include <algorithm>
// #include <map>
// #include <string>

// const double L3 = 60.0;     
// const double L4 = 100.0;
// const double L5 = 65.0;   
// const double HEIGHT_STD = 224.0; 

// class UvcControllerNode : public rclcpp::Node {
// public:
//     UvcControllerNode() : Node("uvc_node") {
//         gain_pitch_p = 0.8; gain_pitch_d = 0.025;  
//         gain_roll_p  = 0.8; gain_roll_d  = 0.015;
//         last_pitch = 0.0; last_roll = 0.0;

//         std::vector<std::string> joints = {
//             "base_hip_left", "hip_hip_left", "hip_knee_left", "knee_ankle_left", "ankle_ankle_left",
//             "base_hip_right", "hip_hip_right", "hip_knee_right", "knee_ankle_right", "ankle_ankle_right"
//         };
//         for (const auto& j : joints) pubs_[j] = create_joint_pub(j + "_joint");

//         angle_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
//             "/robot_orientation", 10, std::bind(&UvcControllerNode::control_callback, this, std::placeholders::_1));

//         RCLCPP_INFO(this->get_logger(), "--> UVC GEOMETRY-CORRECTED CONTROL: Ready");
//     }

// private:
//     // ... (Giữ nguyên các hàm phụ trợ send_cmd, create_joint_pub) ...

//     void control_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
//         double pitch = -msg->x;
//         double roll  = -msg->y;



//         double l_hn, l_ht, l_dg, l_mct, l_mcn;
//         double r_hn, r_ht, r_dg, r_mct, r_mcn;

//         solve_ik(x, y, z, l_hn, l_ht, l_dg, l_mct, l_mcn);
//         solve_ik(x, y, z, r_hn, r_ht, r_dg, r_mct, r_mcn);

//         publish_full_body(l_hn, l_ht, l_dg, l_mct, l_mcn, r_hn, r_ht, r_dg, r_mct, r_mcn);
//     }

//     void solve_ik(double dx, double dy, double dz, double &hn, double &ht, double &dg, double &mct, double &mcn) {
//         hn = atan2(dy, dz);
//         double d_yz = sqrt(dz*dz + dy*dy);
//         double cos3 = (pow(d_yz - L5, 2) + dx*dx - L3*L3 - L4*L4) / (2*L3*L4);
//         cos3 = std::clamp(cos3, -1.0, 1.0);
//         double sin3 = sqrt(1.0 - cos3*cos3);
//         dg = atan2(sin3, cos3);
//         ht = atan2(sin3*L4, L3 + cos3*L4) + atan2(dx, d_yz - L5);
//         mct = -dg + ht;
//         mcn = -hn;
//     }

//     void publish_full_body(double l_hn, double l_ht, double l_dg, double l_mct, double l_mcn,
//                            double r_hn, double r_ht, double r_dg, double r_mct, double r_mcn) {
//         // Chân Trái
//         send_cmd("base_hip_left", l_hn);
//         send_cmd("hip_hip_left", -l_ht);
//         send_cmd("hip_knee_left", l_dg);
//         send_cmd("knee_ankle_left", l_mct);
//         send_cmd("ankle_ankle_left", l_mcn);

//         // Chân Phải
//         send_cmd("base_hip_right", -r_hn);
//         send_cmd("hip_hip_right", r_ht);
//         send_cmd("hip_knee_right", -r_dg);
//         send_cmd("knee_ankle_right", -r_mct);
//         send_cmd("ankle_ankle_right", r_mcn);
//     }
    
//     // Các khớp tay, háng giữa... (tương tự như trước)
//     void send_cmd(std::string key, double value) {
//         if (pubs_.count(key)) {
//             std_msgs::msg::Float64 msg; msg.data = value; pubs_[key]->publish(msg);
//         }
//     }
//     rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr create_joint_pub(std::string joint_name) {
//         return this->create_publisher<std_msgs::msg::Float64>("/model/humanoid_robot/joint/" + joint_name + "/cmd_pos", 10);
//     }

//     std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> pubs_;
//     rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr angle_sub_;
//     double last_pitch, last_roll, gain_pitch_p, gain_pitch_d, gain_roll_p, gain_roll_d;
//     double x=0.0, y=0.0, z=HEIGHT_STD-20;
// };

// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<UvcControllerNode>());
//     rclcpp::shutdown();
//     return 0;
// }




// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/vector3.hpp"
// #include "std_msgs/msg/float64.hpp"
// #include <cmath>
// #include <algorithm>
// #include <map>
// #include <string>

// const double L3 = 60.0;     
// const double L4 = 100.0;
// const double L5 = 65.0;   
// const double HEIGHT_STD = 224.0; 

// class UvcControllerNode : public rclcpp::Node {
// public:
//     UvcControllerNode() : Node("uvc_node") {
//         gain=1;  
//         rr=0.5;

//         std::vector<std::string> joints = {
//             "base_hip_left", "hip_hip_left", "hip_knee_left", "knee_ankle_left", "ankle_ankle_left",
//             "base_hip_right", "hip_hip_right", "hip_knee_right", "knee_ankle_right", "ankle_ankle_right"
//         };
//         for (const auto& j : joints) pubs_[j] = create_joint_pub(j + "_joint");

//         angle_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
//             "/robot_orientation", 10, std::bind(&UvcControllerNode::control_callback, this, std::placeholders::_1));

//         RCLCPP_INFO(this->get_logger(), "--> UVC GEOMETRY-CORRECTED CONTROL: Ready");
//     }

// private:
//     // ... (Giữ nguyên các hàm phụ trợ send_cmd, create_joint_pub) ...

//     void control_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
//         double pitch = -msg->x *M_PI/180;
//         double roll  = -msg->y *M_PI/180;

//         double tm = sqrt(roll*roll+pitch*pitch);
//         double thetay=atan2(y,z);

//         double lyh=z/cos(thetay);
//         y=lyh*sin(thetay+roll*gain);
//         double h_tmp = lyh*cos(thetay+roll*gain);
//         double thetax=atan2(x,h_tmp);
//         double lxh=h_tmp/cos(thetax);
//         x=lxh*sin(thetax+pitch*gain);
//         z=lxh*cos(thetax+pitch*gain);
//         if(z<HEIGHT_STD-5) z+= (HEIGHT_STD-z)*rr;

//         if(tm > 0.01){
//             x-=x*rr;
//             y-=y*rr;
//         }
//         // if(fabs(x)<=3)x=0;
//         // if(fabs(y)<=3)y=0;
//         double l_hn, l_ht, l_dg, l_mct, l_mcn;
//         double r_hn, r_ht, r_dg, r_mct, r_mcn;
//         std::cout<<"x: "<<x<<" y: "<<y<< " z: "<<z<<std::endl;
//         solve_ik(x, y, z, l_hn, l_ht, l_dg, l_mct, l_mcn);
//         solve_ik(x, y, z, r_hn, r_ht, r_dg, r_mct, r_mcn);

//         publish_full_body(l_hn, l_ht, l_dg, l_mct, l_mcn, r_hn, r_ht, r_dg, r_mct, r_mcn);
//     }

//     void solve_ik(double dx, double dy, double dz, double &hn, double &ht, double &dg, double &mct, double &mcn) {
//         hn = atan2(dy, dz);
//         double d_yz = sqrt(dz*dz + dy*dy);
//         double cos3 = (pow(d_yz - L5, 2) + dx*dx - L3*L3 - L4*L4) / (2*L3*L4);
//         cos3 = std::clamp(cos3, -1.0, 1.0);
//         double sin3 = sqrt(1.0 - cos3*cos3);
//         dg = atan2(sin3, cos3);
//         ht = atan2(sin3*L4, L3 + cos3*L4) + atan2(dx, d_yz - L5);
//         mct = -dg + ht;
//         mcn = -hn;
//     }

//     void publish_full_body(double l_hn, double l_ht, double l_dg, double l_mct, double l_mcn,
//                            double r_hn, double r_ht, double r_dg, double r_mct, double r_mcn) {
//         // Chân Trái
//         send_cmd("base_hip_left", l_hn);
//         send_cmd("hip_hip_left", -l_ht);
//         send_cmd("hip_knee_left", l_dg);
//         send_cmd("knee_ankle_left", l_mct);
//         send_cmd("ankle_ankle_left", l_mcn);

//         // Chân Phải
//         send_cmd("base_hip_right", -r_hn);
//         send_cmd("hip_hip_right", r_ht);
//         send_cmd("hip_knee_right", -r_dg);
//         send_cmd("knee_ankle_right", -r_mct);
//         send_cmd("ankle_ankle_right", r_mcn);
//     }
    
//     // Các khớp tay, háng giữa... (tương tự như trước)
//     void send_cmd(std::string key, double value) {
//         if (pubs_.count(key)) {
//             std_msgs::msg::Float64 msg; msg.data = value; pubs_[key]->publish(msg);
//         }
//     }
//     rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr create_joint_pub(std::string joint_name) {
//         return this->create_publisher<std_msgs::msg::Float64>("/model/humanoid_robot/joint/" + joint_name + "/cmd_pos", 10);
//     }

//     std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> pubs_;
//     rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr angle_sub_;
//     double gain, rr;
//     double x=10.0, y=10.0, z=HEIGHT_STD-5;
// };

// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<UvcControllerNode>());
//     rclcpp::shutdown();
//     return 0;
// }
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>
#include <algorithm>
#include <map>
#include <string>
#include <memory>

const double L3 = 60.0;
const double L4 = 100.0;
const double L5 = 65.0;
const double HEIGHT_STD = 224.0;

class UvcControllerNode : public rclcpp::Node {
public:
    UvcControllerNode() : Node("uvc_node") {
        // =================== CÁC THAM SỐ CÓ THỂ TINH CHỈNH CHO RL ===================
        declare_parameter("gain", 0.18);
        declare_parameter("recovery_rate", 0.1);
        declare_parameter("step_duration", 25.0);
        declare_parameter("stance_width", 35.0);
        declare_parameter("max_foot_lift", 12.0);
        declare_parameter("scale_base", 0.12);  // Thêm scale_base
        declare_parameter("tilt_threshold", 2.0);
        declare_parameter("landing_phase", 6.0);
        declare_parameter("min_clearance", 18.0);
        
        // Lấy giá trị tham số
        gain = get_parameter("gain").as_double();
        rr = get_parameter("recovery_rate").as_double();
        fwctEnd = get_parameter("step_duration").as_double();
        stance_width = get_parameter("stance_width").as_double();
        fhMax = get_parameter("max_foot_lift").as_double();
        scale_base = get_parameter("scale_base").as_double();  // Lấy scale_base
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
        // Hai chân song song, không chữ V
        dxi = 0.0;      // Chân trụ X (trước-sau)
        dyi = stance_width;  // Chân trụ Y (phải - dương khi nhìn từ trên)
        dxis = 0.0;     // Chân di chuyển X
        dyis = -stance_width; // Chân di chuyển Y (trái - âm khi nhìn từ trên)
        
        dxib = 0.0;
        dyib = 0.0;
        
        // Chân trụ: 0 = phải, 1 = trái
        support_leg = 0;  // Bắt đầu với chân phải làm trụ
        
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
        
        // Trạng thái
        mode = 0;  // 0 = hiệu chuẩn, 1 = đứng chờ, 2 = UVC hoạt động
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
        
        // Subscribe vào RL parameters từ RL training node
        // [gain, scale_base, step_duration, landing_phase, stance_width, max_foot_lift, recovery_rate]
        rl_param_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/uvc_parameters", 10,
            std::bind(&UvcControllerNode::rl_param_callback, this, std::placeholders::_1));

        // Subscribe to reset requests from RL node
        reset_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/uvc_reset", 10,
            std::bind(&UvcControllerNode::reset_callback, this, std::placeholders::_1));
        
        // Timer 20Hz (50ms) - đủ nhanh cho điều khiển
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&UvcControllerNode::control_loop, this));
        
        // Publisher cho RL feedback
        rl_feedback_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
            "/uvc_rl_feedback", 10);
        
        RCLCPP_INFO(this->get_logger(), "UVC Controller for RL Training - Ready");
        RCLCPP_INFO(this->get_logger(), "Parameters: gain=%.3f, stance_width=%.1fmm", gain, stance_width);
        RCLCPP_INFO(this->get_logger(), "Tilt threshold: %.1f degrees", tilt_threshold_deg);
    }

private:
    void imu_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        // Đọc và chuyển đổi dữ liệu IMU
        double pitch_raw = -msg->x * M_PI / 180.0;
        double roll_raw = -msg->y * M_PI / 180.0;
        
        // Lọc low-pass để giảm nhiễu
        double alpha = 0.6;
        pitch_filtered = alpha * pitch_raw + (1 - alpha) * pitch_filtered;
        roll_filtered = alpha * roll_raw + (1 - alpha) * roll_filtered;
        
        // Tính đạo hàm (tốc độ thay đổi) cho D controller
        pitch_derivative = (pitch_filtered - pitch_prev) / 0.05;  // 0.05s = 50ms
        roll_derivative = (roll_filtered - roll_prev) / 0.05;
        
        pitch_prev = pitch_filtered;
        roll_prev = roll_filtered;
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

    void reset_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            // Immediate safe reset: go to standing pose
            RCLCPP_INFO(this->get_logger(), "[RL RESET] Received reset request, forcing stand.");
            // Reset integrators and heights to default standing
            dxi = 0.0;
            dyi = stance_width;
            dxis = 0.0;
            dyis = -stance_width;
            autoH = HEIGHT_STD;
            fh = 0.0;
            fwct = 0.0;
            support_leg = 0;
            mode = 1; // standing mode
            publish_parallel_stance();
        }
    }
    
    void control_loop() {
        // State machine chính
        switch(mode) {
            case 0: // Hiệu chuẩn
                calibration_mode();
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
                dxi = 0.0;
                dyi = stance_width;
                dxis = 0.0;
                dyis = -stance_width;
                fwct = 0.0;
                support_leg = 0;
                
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
            
            RCLCPP_INFO(this->get_logger(), "UVC Activated! Tilt: %.1f deg, Support leg: %s",
                       tilt_magnitude * 180.0 / M_PI,
                       support_leg == 0 ? "RIGHT" : "LEFT");
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
            RCLCPP_INFO(this->get_logger(), 
                       "[IMU RAW] pitch=%.2f° roll=%.2f° | pitch_offset=%.2f° roll_offset=%.2f°",
                       pitch * 180.0/M_PI, roll * 180.0/M_PI,
                       pitch_offset * 180.0/M_PI, roll_offset * 180.0/M_PI);
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
            RCLCPP_INFO(this->get_logger(), 
                       "[AFTER THRESHOLD] pitch=%.2f° roll=%.2f° | tilt_mag=%.2f°",
                       pitch * 180.0/M_PI, roll * 180.0/M_PI,
                       tilt_magnitude * 180.0/M_PI);
        }
        
        // 4. Áp dụng UVC logic (giống main.c)
        double dyi_before = dyi;
        double dxi_before = dxi;
        apply_uvc_geometry_control();
        
        // DEBUG: In ra sau UVC
        if (dbg_imu_cnt % 2 == 0) {
            RCLCPP_INFO(this->get_logger(), 
                       "[AFTER UVC] dyi: %.1f→%.1f | dxi: %.1f→%.1f | autoH=%.1f",
                       dyi_before, dyi, dxi_before, dxi, autoH);
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
                       "[CYCLE] phase=%.1f/%.1f, leg=%s, fh=%.1f",
                       fwct, fwctEnd,
                       support_leg == 0 ? "RIGHT" : "LEFT",
                       fh);
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
        // ============ ĐIỀU CHỈNH Y (trái-phải) ============
        double k = atan((dyi) / autoH);
        double kl = autoH / cos(k);
        
        // Khi roll > 0 (nghiêng phải), cần TRỪ khỏi k để dyi giảm (chân sang trái)
        // Scale factor động: tăng nếu tilt lớn, giảm nếu tilt nhỏ
        double tilt_mag = sqrt(pitch * pitch + roll * roll);
        double scale_base = 0.12;  // Tăng từ 0.1 lên 0.12
        double scale_factor = scale_base * (1.0 + 0.5 * std::min(tilt_mag, 1.0));  // Max +50% ở tilt ~90°
        double roll_scaled = scale_factor * roll;
        double ks = k - roll_scaled;  // ĐỔI từ + thành -
        
        dyi = kl * sin(ks);
        autoH = kl * cos(ks);
        
        // DEBUG: In ra roll geometry
        static int dbg_geom_cnt = 0;
        if (++dbg_geom_cnt % 2 == 0) {
            RCLCPP_INFO(this->get_logger(),
                       "[GEOM-Y] roll=%.3f rad | roll_scaled=%.3f | k=%.4f ks=%.4f | kl=%.1f | dyi_new=%.1f autoH=%.1f",
                       roll, roll_scaled, k, ks, kl, dyi, autoH);
        }
        
        // ============ ĐIỀU CHỈNH X (trước-sau) ============
        k = atan(dxi / autoH);
        kl = autoH / cos(k);
        
        // Khi pitch > 0 (nghiêng trước), cần TRỪ khỏi k để dxi giảm (chân sang sau)
        // Sử dụng same scale factor như Y
        double pitch_scaled = scale_factor * pitch;
        ks = k - pitch_scaled;  // ĐỔI từ + thành -
        
        dxi = kl * sin(ks);
        autoH = kl * cos(ks);
        
        // DEBUG: In ra pitch geometry
        if (dbg_geom_cnt % 2 == 0) {
            RCLCPP_INFO(this->get_logger(),
                       "[GEOM-X] pitch=%.3f rad | pitch_scaled=%.3f | k=%.4f ks=%.4f | kl=%.1f | dxi_new=%.1f autoH=%.1f",
                       pitch, pitch_scaled, k, ks, kl, dxi, autoH);
        }
        
        // ============ GIỚI HẠN AN TOÀN ============
        if (dyi < 0) dyi = 0;
        if (dyi > 45) dyi = 45;
        if (dxi < -45) dxi = -45;
        if (dxi > 45) dxi = 45;
        
        dyis = dyi;
        dxis = -dxi;
        
        // ============ PHỤC HỒI CHIỀU CAO ============
        if (HEIGHT_STD > autoH) {
            autoH += (HEIGHT_STD - autoH) * 0.07;
        } else {
            autoH = HEIGHT_STD;
        }
        
        if (autoH < 140) autoH = 140;
    }
    
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
    
    ///////////////////////////////////////////////////////////////////////////////////
    //// LEG TARGETS & LANDING PHASE - Tính vị trí chân cuối cùng + landing ////
    ///////////////////////////////////////////////////////////////////////////////////
    void calculate_and_publish_legs() {
        double l_hn, l_ht, l_dg, l_mct, l_mcn;
        double r_hn, r_ht, r_dg, r_mct, r_mcn;
        
        // Ensure swing foot does not go below min_clearance
        double swing_dz_right = autoH;
        double swing_dz_left = autoH;
        if (fh > 0.0) {
            swing_dz_right = std::max(autoH - fh, min_clearance);
            swing_dz_left = std::max(autoH - fh, min_clearance);
        }

        // During landing phase, bring swing foot back down to ground
        double swing_end = fwctEnd - landing_phase;

        if (fwct >= swing_end) {
            // Landing phase: bring swing foot back to ground (autoH)
            double landing_progress = 0.0;
            double landing_duration = fwctEnd - swing_end;
            if (landing_duration > 0.0) {
                landing_progress = std::clamp((fwct - swing_end) / landing_duration, 0.0, 1.0);
            } else {
                landing_progress = 1.0;
            }

            // Linearly bring swing foot from current height down to autoH
            if (support_leg == 0) {
                // left is swing -> lower to ground
                swing_dz_left = autoH + (swing_dz_left - autoH) * (1.0 - landing_progress);
            } else {
                // right is swing -> lower to ground
                swing_dz_right = autoH + (swing_dz_right - autoH) * (1.0 - landing_progress);
            }
        }

        // Support feet always at full height for stability
        double support_dz_right = autoH;
        double support_dz_left = autoH;

        if (support_leg == 0) { // Chân PHẢI trụ
            // Chân phải (trụ) - độ cao có thể được hạ xuống trong landing
            solve_ik(dxi, dyi, support_dz_right, r_hn, r_ht, r_dg, r_mct, r_mcn);

            // Chân trái (di chuyển) - nâng lên (z limited)
            solve_ik(dxis, dyis, swing_dz_left, l_hn, l_ht, l_dg, l_mct, l_mcn);
        } else { // Chân TRÁI trụ
            // Chân trái (trụ) - có thể hạ xuống
            solve_ik(dxis, dyis, support_dz_left, l_hn, l_ht, l_dg, l_mct, l_mcn);

            // Chân phải (di chuyển) - nâng lên (z limited)
            solve_ik(dxi, dyi, swing_dz_right, r_hn, r_ht, r_dg, r_mct, r_mcn);
        }

        // Periodic debug logging for leg targets
        static int leg_dbg = 0;
        if (++leg_dbg % 6 == 0) { // ~every 300ms at 50ms timer
            RCLCPP_INFO(this->get_logger(), "LEG TARGETS: fwct=%.1f/%.1f support=%s fh=%.2f swing_z(L:%.1f R:%.1f) support_z(L:%.1f R:%.1f) dxi=%.1f dxis=%.1f dyi=%.1f dyis=%.1f",
                        fwct, fwctEnd, support_leg==0?"RIGHT":"LEFT", fh,
                        swing_dz_left, swing_dz_right, support_dz_left, support_dz_right,
                        dxi, dxis, dyi, dyis);
        }
        
        // Publish
        publish_legs(l_hn, l_ht, l_dg, l_mct, l_mcn,
                    r_hn, r_ht, r_dg, r_mct, r_mcn);
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
            
            RCLCPP_INFO(this->get_logger(), "Step cycle complete. New support leg: %s",
                       support_leg == 0 ? "RIGHT" : "LEFT");
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
        solve_ik(0.0, stance_width, HEIGHT_STD, l_hn, l_ht, l_dg, l_mct, l_mcn);
        // Chân phải: giữa, bên phải
        solve_ik(0.0, -stance_width, HEIGHT_STD, r_hn, r_ht, r_dg, r_mct, r_mcn);
        
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
        mct = -dg + ht;
        mcn = -hn;
    }
    
    ///////////////////////////////////////////////////////////////////////////////////
    //// PUBLISH LEGS - Gửi lệnh góc khớp đến các chân (với ánh xạ dấu L/R) ////
    ///////////////////////////////////////////////////////////////////////////////////
    void publish_legs(double l_hn, double l_ht, double l_dg, double l_mct, double l_mcn,
                      double r_hn, double r_ht, double r_dg, double r_mct, double r_mcn) {
        // Left Leg (mapping chosen to match URDF axes)
        send_cmd("base_hip_left", l_hn);
        send_cmd("hip_hip_left", -l_ht);
        send_cmd("hip_knee_left", l_dg);
        send_cmd("knee_ankle_left", l_mct);
        send_cmd("ankle_ankle_left", l_mcn);

        // Right Leg (mirrored mapping)
        send_cmd("base_hip_right", -r_hn);
        send_cmd("hip_hip_right", r_ht);
        send_cmd("hip_knee_right", -r_dg);
        send_cmd("knee_ankle_right", -r_mct);
        send_cmd("ankle_ankle_right", r_mcn);
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
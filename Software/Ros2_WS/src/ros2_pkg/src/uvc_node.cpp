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
        // Các tham số này có thể được RL tối ưu hóa
        declare_parameter("gain", 0.15);                 // Hệ số UVC chính
        declare_parameter("recovery_rate", 0.1);         // Tốc độ phục hồi
        declare_parameter("step_duration", 30.0);        // Thời gian mỗi bước (chu kỳ)
        declare_parameter("stance_width", 35.0);         // Độ rộng đứng (mm) - KHÔNG CHỮ V
        declare_parameter("max_foot_lift", 12.0);        // Độ cao nâng chân tối đa
        declare_parameter("max_swing", 10.0);            // Dao động ngang tối đa
        declare_parameter("tilt_threshold", 2.0);        // Ngưỡng bắt đầu UVC (độ)
        declare_parameter("landing_phase", 5.0);         // Pha tiếp đất trong chu kỳ
        declare_parameter("k_p", 2.0);                   // Hệ số P cho điều chỉnh
        declare_parameter("k_d", 0.5);                   // Hệ số D cho điều chỉnh
        
        // Lấy giá trị tham số
        gain = get_parameter("gain").as_double();
        rr = get_parameter("recovery_rate").as_double();
        fwctEnd = get_parameter("step_duration").as_double();
        stance_width = get_parameter("stance_width").as_double();
        fhMax = get_parameter("max_foot_lift").as_double();
        swMax = get_parameter("max_swing").as_double();
        tilt_threshold_deg = get_parameter("tilt_threshold").as_double();
        landing_phase = get_parameter("landing_phase").as_double();
        k_p = get_parameter("k_p").as_double();
        k_d = get_parameter("k_d").as_double();
        
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
        
        // Timer 20Hz (50ms) - đủ nhanh cho điều khiển
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&UvcControllerNode::control_loop, this));
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rl_param_sub_;

        rl_param_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/uvc_parameters", 10,
            std::bind(&UvcControllerNode::rl_param_callback, this, std::placeholders::_1));
        // Publisher cho RL feedback (tùy chọn)
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
    // Callback function
    void rl_param_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 7) {
            gain = msg->data[0];
            stance_width = msg->data[1];
            fhMax = msg->data[2];
            swMax = msg->data[3];
            tilt_threshold_deg = msg->data[4];
            k_p = msg->data[5];
            k_d = msg->data[6];
            
            tilt_threshold = tilt_threshold_deg * M_PI / 180.0;
            
            RCLCPP_INFO(this->get_logger(), "RL params updated");
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
    
    void uvc_active_mode() {
        // 1. Tính góc nghiêng đã hiệu chuẩn
        pitch = pitch_filtered - pitch_offset;
        roll = roll_filtered - roll_offset;
        
        // 2. Áp dụng PD controller cho UVC
        apply_uvc_pd_control();
        
        // 3. Tính độ cao nâng chân theo chu kỳ
        calculate_foot_lift();
        
        // 4. Tính toán và publish vị trí chân
        calculate_and_publish_legs();
        
        // 5. Cập nhật bộ đếm chu kỳ
        update_cycle_counter();
        
        // 6. Gửi feedback cho RL (tùy chọn)
        publish_rl_feedback();
        
        // Debug thông tin
        static int debug_counter = 0;
        if (debug_counter++ % 10 == 0) { // Mỗi 0.5 giây
            RCLCPP_INFO(this->get_logger(), 
                       "UVC: phase=%.1f/%.1f, leg=%s, tilt=(%.1f,%.1f)deg",
                       fwct, fwctEnd,
                       support_leg == 0 ? "RIGHT" : "LEFT",
                       pitch * 180.0 / M_PI, roll * 180.0 / M_PI);
        }
        
        // Kiểm tra nếu đã đủ ổn định để quay lại đứng
        if (fwct >= fwctEnd * 0.8) { // 80% chu kỳ
            double tilt_magnitude = sqrt(pitch * pitch + roll * roll);
            if (tilt_magnitude < tilt_threshold * 0.5) {
                mode = 1;
                stable_count = 0;
                RCLCPP_INFO(this->get_logger(), "Returning to standing mode");
            }
        }
    }
    
    void apply_uvc_pd_control() {
        // UVC đơn giản hóa với PD control
        // Mục tiêu: điều chỉnh vị trí chân trụ để chống lại độ nghiêng
        
        // Tính toán điều chỉnh mong muốn dựa trên PD
        double adjust_x = 0.0;
        double adjust_y = 0.0;
        
        // PD control cho trục X (pitch - nghiêng trước/sau)
        adjust_x = pitch * k_p + pitch_derivative * k_d;
        
        // PD control cho trục Y (roll - nghiêng trái/phải)
        // Dấu âm vì nếu nghiêng phải (roll dương) cần di chuyển chân trụ sang trái
        adjust_y = -roll * k_p - roll_derivative * k_d;
        
        // Scale bằng gain
        adjust_x *= gain * 10.0;  // Nhân 10 để chuyển từ rad sang mm (ước lượng)
        adjust_y *= gain * 10.0;
        
        // Giới hạn điều chỉnh để tránh quá kích
        adjust_x = std::clamp(adjust_x, -15.0, 15.0);
        adjust_y = std::clamp(adjust_y, -15.0, 15.0);
        
        // Chỉ áp dụng trong phần swing của chu kỳ
        if (fwct > landing_phase && fwct < (fwctEnd - landing_phase)) {
            // Áp dụng cho chân trụ
            if (support_leg == 0) { // Chân phải trụ
                dxi += adjust_x;
                dyi = stance_width + adjust_y;  // Giữ khoảng cách nhưng điều chỉnh
                
                // Chân di chuyển (trái) đối xứng và nâng lên
                dxis = -dxi * 0.3;  // Di chuyển ít hơn
                dyis = -stance_width - adjust_y * 0.5;
            } else { // Chân trái trụ
                dxis += adjust_x;
                dyis = -stance_width + adjust_y;
                
                // Chân di chuyển (phải) đối xứng và nâng lên
                dxi = -dxis * 0.3;
                dyi = stance_width - adjust_y * 0.5;
            }
        }
        
        // Phục hồi chiều cao nếu cần
        if (HEIGHT_STD > autoH) {
            autoH += (HEIGHT_STD - autoH) * rr;
        } else {
            autoH = HEIGHT_STD;
        }
        
        // Giới hạn an toàn
        autoH = std::clamp(autoH, HEIGHT_STD - 20.0, HEIGHT_STD + 10.0);
    }
    
    void calculate_foot_lift() {
        // Tính độ cao nâng chân theo hình sin
        // Chỉ nâng trong phần giữa của chu kỳ
        double swing_start = landing_phase;
        double swing_end = fwctEnd - landing_phase;
        
        if (fwct > swing_start && fwct < swing_end) {
            double swing_duration = swing_end - swing_start;
            double swing_progress = (fwct - swing_start) / swing_duration;
            fh = fhMax * sin(M_PI * swing_progress);
        } else {
            fh = 0.0;
        }
    }
    
    void calculate_and_publish_legs() {
        double l_hn, l_ht, l_dg, l_mct, l_mcn;
        double r_hn, r_ht, r_dg, r_mct, r_mcn;
        
        if (support_leg == 0) { // Chân PHẢI trụ
            // Chân phải (trụ) - đầy đủ chiều cao
            solve_ik(dxi, dyi, autoH, r_hn, r_ht, r_dg, r_mct, r_mcn);
            
            // Chân trái (di chuyển) - nâng lên
            solve_ik(dxis, dyis, autoH - fh, l_hn, l_ht, l_dg, l_mct, l_mcn);
        } else { // Chân TRÁI trụ
            // Chân trái (trụ) - đầy đủ chiều cao
            solve_ik(dxis, dyis, autoH, l_hn, l_ht, l_dg, l_mct, l_mcn);
            
            // Chân phải (di chuyển) - nâng lên
            solve_ik(dxi, dyi, autoH - fh, r_hn, r_ht, r_dg, r_mct, r_mcn);
        }
        
        // Áp dụng giới hạn khớp
        apply_joint_limits(l_hn, l_ht, l_dg, l_mct, l_mcn, false); // Chân trái
        apply_joint_limits(r_hn, r_ht, r_dg, r_mct, r_mcn, true);  // Chân phải
        
        // Publish
        publish_legs(l_hn, l_ht, l_dg, l_mct, l_mcn,
                    r_hn, r_ht, r_dg, r_mct, r_mcn);
    }
    
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
    
    void publish_parallel_stance() {
        // Tư thế đứng với hai chân SONG SONG
        double l_hn, l_ht, l_dg, l_mct, l_mcn;
        double r_hn, r_ht, r_dg, r_mct, r_mcn;
        
        // Chân trái: giữa, bên trái
        solve_ik(0.0, stance_width, HEIGHT_STD, l_hn, l_ht, l_dg, l_mct, l_mcn);
        // Chân phải: giữa, bên phải
        solve_ik(0.0, -stance_width, HEIGHT_STD, r_hn, r_ht, r_dg, r_mct, r_mcn);
        
        apply_joint_limits(l_hn, l_ht, l_dg, l_mct, l_mcn, false);
        apply_joint_limits(r_hn, r_ht, r_dg, r_mct, r_mcn, true);
        
        publish_legs(l_hn, l_ht, l_dg, l_mct, l_mcn,
                    r_hn, r_ht, r_dg, r_mct, r_mcn);
    }
    
    void publish_rl_feedback() {
        // Gửi feedback cho RL training
        auto msg = geometry_msgs::msg::Vector3();
        msg.x = pitch * 180.0 / M_PI;  // Góc pitch (độ)
        msg.y = roll * 180.0 / M_PI;   // Góc roll (độ)
        msg.z = fwct / fwctEnd;        // Tiến trình chu kỳ (0-1)
        
        rl_feedback_pub_->publish(msg);
    }
    
    // =================== HÀM ĐỘNG HỌC VÀ ĐIỀU KHIỂN ===================
    
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
    
    void apply_joint_limits(double &hn, double &ht, double &dg, double &mct, double &mcn, bool is_right) {
        // Áp dụng giới hạn từ URDF của bạn
        // hn = std::clamp(hn, -0.15, 0.65);     // base_hip
        // ht = std::clamp(ht, -0.7, 0.65);      // hip_hip
        // dg = std::clamp(dg, -1.3, 1.3);       // hip_knee
        // mct = std::clamp(mct, -0.75, 0.75);   // knee_ankle
        // mcn = std::clamp(mcn, -1.2, 1.2);     // ankle_ankle
        
        // Đảo dấu cho chân phải
        if (is_right) {
            hn = -hn;
            ht = -ht;
            dg = -dg;
            mct = -mct;
        }
    }
    
    void publish_legs(double l_hn, double l_ht, double l_dg, double l_mct, double l_mcn,
                      double r_hn, double r_ht, double r_dg, double r_mct, double r_mcn) {
        // Left Leg
        send_cmd("base_hip_left", -l_hn);
        send_cmd("hip_hip_left", -l_ht);
        send_cmd("hip_knee_left", l_dg);
        send_cmd("knee_ankle_left", l_mct);
        send_cmd("ankle_ankle_left", -l_mcn);
        
        // Right Leg
        send_cmd("base_hip_right", r_hn);
        send_cmd("hip_hip_right", r_ht);
        send_cmd("hip_knee_right", -r_dg);
        send_cmd("knee_ankle_right", -r_mct);
        send_cmd("ankle_ankle_right", r_mcn);
    }
    
    void send_cmd(const std::string& key, double value) {
        if (pubs_.count(key)) {
            auto msg = std_msgs::msg::Float64();
            msg.data = value;
            pubs_[key]->publish(msg);
        }
    }
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rl_param_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr create_joint_pub(const std::string& joint_name) {
        return this->create_publisher<std_msgs::msg::Float64>(
            "/model/humanoid_robot/joint/" + joint_name + "/cmd_pos", 10);
    }
    
    // =================== BIẾN THÀNH VIÊN ===================
    
    // Tham số có thể điều chỉnh (cho RL)
    double gain, rr, fwctEnd, stance_width, fhMax, swMax;
    double tilt_threshold, tilt_threshold_deg, landing_phase;
    double k_p, k_d;
    
    // Biến trạng thái UVC
    double fwct, fwctUp;
    double autoH, fh;
    double dxi, dyi, dxis, dyis, dxib, dyib;
    int support_leg;
    
    // Biến IMU và điều khiển
    double pitch, roll;
    double pitch_filtered, roll_filtered;
    double pitch_prev, roll_prev;
    double pitch_derivative, roll_derivative;
    double pitch_offset, roll_offset;
    
    // Trạng thái hệ thống
    int mode;
    int calibration_samples;
    int stable_count;
    
    // ROS interfaces
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> pubs_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr rl_feedback_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr angle_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UvcControllerNode>());
    rclcpp::shutdown();
    return 0;
}
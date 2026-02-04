#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/vector3.hpp> 
#include <cmath>
#include <map>
#include <vector>
#include <string>
#include <algorithm>

class HumanoidIKController : public rclcpp::Node {
public:
    // Thông số vật lý trích xuất từ URDF [cite: 21, 25, 29, 41, 44, 48]
    const double L3 = 0.06;    
    const double L4 = 0.1034;  
    const double L5 = 0.057;   

    HumanoidIKController() : Node("humanoid_ik_controller") {
        std::vector<std::string> joints = {
            "base_hip_left", "hip_hip_left", "hip_knee_left", "knee_ankle_left", "ankle_ankle_left",
            "base_hip_right", "hip_hip_right", "hip_knee_right", "knee_ankle_right", "ankle_ankle_right"
        };

        for (const auto & name : joints) {
            std::string topic = "/model/humanoid_robot/joint/" + name + "_joint/cmd_pos";
            pubs_[name] = this->create_publisher<std_msgs::msg::Float64>(topic, 10);
        }

        // Lắng nghe từ node imu_process_node của bạn
        orientation_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/robot_orientation", 10, std::bind(&HumanoidIKController::orientation_callback, this, std::placeholders::_1));

        // Tăng tần số lên 100Hz (10ms) để điều khiển mượt hơn
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&HumanoidIKController::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "--> IK CONTROLLER READY: Using Raw check + Clamp safety");
    }

    // Hàm IK kết hợp cả 2: Trả về false nếu NaN, nhưng vẫn dùng clamp để bảo vệ motor
    bool solve_ik_safe(double dx, double dy, double dz, double &hn, double &ht, double &dg, double &mct, double &mcn) {
        hn = std::atan2(dy, dz);
        double d_yz = std::sqrt(dz * dz + dy * dy);
        
        // 1. Tính toán giá trị thô
        double cos3_raw = (std::pow(d_yz - L5, 2) + dx * dx - L3 * L3 - L4 * L4) / (2.0 * L3 * L4);
        
        // 2. Kiểm tra NaN cho mục đích Logic/Training (Nếu ngoài dải [-1, 1] thì return false)
        if (cos3_raw < -1.0 || cos3_raw > 1.0) {
            return false; 
        }

        // 3. Dùng clamp để đảm bảo an toàn tuyệt đối cho hàm toán học sqrt/atan2
        double cos3 = std::clamp(cos3_raw, -1.0, 1.0);
        double sin3 = std::sqrt(1.0 - cos3 * cos3);

        dg = std::atan2(sin3, cos3);
        ht = std::atan2(sin3 * L4, L3 + cos3 * L4) + std::atan2(dx, d_yz - L5);
        mct = -dg + ht ; // Offset bù sai số    
        mcn = -hn;

        return true;
    }

private:
    double current_pitch_deg_ = 0.0;
    double current_roll_deg_ = 0.0;
    double L_left=0.0;
    double L_right=0.0;
    double bmcn=0.00;
    double bmct=0.00;
    void orientation_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        // Quy ước của bạn: 
        // x: Roll (Âm là nghiêng trái)
        // y: Pitch (Âm là ngả sau)
        current_roll_deg_ = msg->y;
        current_pitch_deg_ = msg->x;
        bmcn=-current_roll_deg_ *M_PI/180.0;
        bmct=current_pitch_deg_*M_PI/180.0 ;
    }

    void send_cmd(std::string name, double pos) {
        auto msg = std_msgs::msg::Float64();
        msg.data = pos;
        pubs_[name]->publish(msg);
    }

    void control_loop() {
        double l_hn, l_ht, l_dg, l_mct, l_mcn;
        double r_hn, r_ht, r_dg, r_mct, r_mcn;

        // Tọa độ đứng thẳng chuẩn bạn đã cung cấp [cite: 17, 21, 32, 37, 41]
        double tx_l = 0.0;
        double tx_r = -0.0;
        double ty_l = 0.02;
        double ty_r = -0.02;
        double tz_l = 0.20;
        double tz_r = 0.20;
        L_left = std::sqrt(tx_l*tx_l + tz_l*tz_l);
        L_right = std::sqrt(tx_r*tx_r + tz_r*tz_r);
        printf("L_left: %.4f | L_right: %.4f \n", L_left, L_right);
        // Chỉ thực thi nếu điểm mục tiêu nằm trong Workspace (trả về true)
        if (solve_ik_safe(tx_l, ty_l, tz_l, l_hn, l_ht, l_dg, l_mct, l_mcn) &&
            solve_ik_safe(tx_r, ty_r, tz_r, r_hn, r_ht, r_dg, r_mct, r_mcn)) {
            
            // Chân trái: Sử dụng logic dấu bạn đã test đứng thẳng thành công [cite: 17, 21, 25, 29, 33]
            send_cmd("base_hip_left", l_hn); 
            send_cmd("hip_hip_left", -l_ht); 
            send_cmd("hip_knee_left", l_dg); 
            send_cmd("knee_ankle_left", l_mct+bmct); 
            send_cmd("ankle_ankle_left", l_mcn+bmcn);

            // Chân phải: Bù trừ dấu cho các trục ngược hướng trong URDF [cite: 37, 41, 44, 48, 52]
            send_cmd("base_hip_right", r_hn); 
            send_cmd("hip_hip_right", r_ht); 
            send_cmd("hip_knee_right", -r_dg); 
            send_cmd("knee_ankle_right", -r_mct-bmct); 
            send_cmd("ankle_ankle_right", -r_mcn-bmcn);
            printf("mctt: %.4f | mctp: %.4f | bmct: %.4f | bmcn: %.4f\n", l_mct+bmct, -r_mcn-bmcn, bmct,bmcn);
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Target out of reach!");
        }
    }

    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> pubs_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr orientation_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HumanoidIKController>());
    rclcpp::shutdown();
    return 0;
}
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>
#include <algorithm>
#include <map>
#include <string>

const double L1 = 60.0;     
const double L2 = 103.4;    
const double HEIGHT_STD = 148.0; 

class UvcControllerNode : public rclcpp::Node {
public:
    UvcControllerNode() : Node("uvc_node") {
        dxi = 0.0; dyi = 0.0; 
        autoH = HEIGHT_STD;
        
        // Gain cho MG996R (như đã tune)
        gain_pitch = 2.5;   
        gain_roll =  2.5;    
        recovery = 0.1; 

        // --- 1. KHỞI TẠO PUBLISHER CHO CHÂN (Đã có) ---
        pubs_["base_hip_left"]   = create_joint_pub("base_hip_left_joint");
        pubs_["hip_hip_left"]    = create_joint_pub("hip_hip_left_joint");
        pubs_["hip_knee_left"]   = create_joint_pub("hip_knee_left_joint");
        pubs_["knee_ankle_left"] = create_joint_pub("knee_ankle_left_joint");
        pubs_["ankle_ankle_left"]= create_joint_pub("ankle_ankle_left_joint");

        pubs_["base_hip_right"]   = create_joint_pub("base_hip_right_joint");
        pubs_["hip_hip_right"]    = create_joint_pub("hip_hip_right_joint");
        pubs_["hip_knee_right"]   = create_joint_pub("hip_knee_right_joint");
        pubs_["knee_ankle_right"] = create_joint_pub("knee_ankle_right_joint");
        pubs_["ankle_ankle_right"]= create_joint_pub("ankle_ankle_right_joint");

        pubs_["base_hip_middle"]  = create_joint_pub("base_hip_middle_joint");

        // --- 2. THÊM PUBLISHER CHO TAY (MỚI) ---
        // Tay Trái
        pubs_["hip_shoulder_left"]      = create_joint_pub("hip_shoulder_left_joint");
        pubs_["shoulder_shoulder_left"] = create_joint_pub("shoulder_shoulder_left_joint");
        pubs_["shoulder_elbow_left"]    = create_joint_pub("shoulder_elbow_left_joint");

        // Tay Phải
        pubs_["hip_shoulder_right"]      = create_joint_pub("hip_shoulder_right_joint");
        pubs_["shoulder_shoulder_right"] = create_joint_pub("shoulder_shoulder_right_joint");
        pubs_["shoulder_elbow_right"]    = create_joint_pub("shoulder_elbow_right_joint");


        angle_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/robot_orientation", 10, std::bind(&UvcControllerNode::control_callback, this, std::placeholders::_1));

        this->set_parameter(rclcpp::Parameter("use_sim_time", true));
        RCLCPP_INFO(this->get_logger(), "--> UVC FULL BODY CONTROL: Ready!");
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr create_joint_pub(std::string joint_name) {
        std::string topic = "/model/humanoid_robot/joint/" + joint_name + "/cmd_pos";
        return this->create_publisher<std_msgs::msg::Float64>(topic, 10);
    }

    void send_cmd(std::string key, double value) {
        if (pubs_.count(key)) {
            std_msgs::msg::Float64 msg;
            msg.data = value;
            pubs_[key]->publish(msg);
        }
    }

    void control_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        // ... (Giữ nguyên logic tính toán của bạn) ...
        double pitch_rad = msg->x * M_PI / 180.0;
        double roll_rad  = msg->y * M_PI / 180.0;
        
        tm = sqrt(pitch_rad*pitch_rad + roll_rad*roll_rad);
        
        theta_y = atan2(dyi, autoH);
        Lyh = autoH / cos(theta_y);
        
        dyi = Lyh * sin(theta_y + roll_rad * gain_roll);
        H_tmp = Lyh * cos(theta_y + roll_rad * gain_roll);

        theta_x = atan2(dxi, H_tmp);
        Lxh = H_tmp / cos(theta_x);
        
        dxi = Lxh * sin(theta_x + pitch_rad * gain_pitch);
        autoH = Lxh * cos(theta_x + pitch_rad * gain_pitch);

        dyi = std::clamp(dyi, -60.0, 60.0);
        dxi = std::clamp(dxi, -60.0, 60.0); 
        autoH = std::clamp(autoH, 130.0, 155.0); 

        if(tm < 0.1) {
             dxi -= dxi * recovery;
             dyi -= dyi * recovery;
        }
        if(autoH < HEIGHT_STD) autoH += (HEIGHT_STD - autoH) * recovery;
        
        if(std::fabs(dxi) < 0.1) dxi = 0;
        if(std::fabs(dyi) < 0.1) dyi = 0;

        double hn, ht, dg, mct, mcn;
        solve_ik(dxi, dyi, autoH, hn, ht, dg, mct, mcn);

        if(!std::isnan(ht) && !std::isnan(dg)) {
            publish_command(hn, ht, dg, mct, mcn);
        }
    }

    void solve_ik(double dx, double dy, double h, 
                  double &hn, double &ht, double &dg, double &mct, double &mcn) {
        // ... (Giữ nguyên logic IK của bạn) ...
        double L_sq = dx*dx + dy*dy + h*h;
        double L = std::sqrt(L_sq);

        if (L > (L1 + L2 - 0.1)) { L = L1 + L2 - 0.1; L_sq = L*L; }
        if (L < abs(L1 - L2) + 0.1) { L = abs(L1 - L2) + 0.1; L_sq = L*L; }

        hn = atan2(dy, h);

        double cos_dg = (L1*L1 + L2*L2 - L_sq) / (2*L1*L2);
        dg = M_PI - acos(std::clamp(cos_dg, -1.0, 1.0)); 

        double phi = atan2(dx, sqrt(dy*dy + h*h)); 
        double cos_ht = (L1*L1 + L_sq - L2*L2) / (2*L1*L);
        ht = phi + acos(std::clamp(cos_ht, -1.0, 1.0));

        mct = ht - dg; 
        mcn = -hn;    
    }

    void publish_command(double hn, double ht, double dg, double mct, double mcn) {
        // --- CHÂN TRÁI ---
        send_cmd("base_hip_left",   ht);
        send_cmd("hip_hip_left",    -hn);
        send_cmd("hip_knee_left",    -dg); 
        send_cmd("knee_ankle_left",  mct);
        send_cmd("ankle_ankle_left",-mcn);

        // --- CHÂN PHẢI (Mirror) ---
        send_cmd("base_hip_right",    -ht);
        send_cmd("hip_hip_right",    hn);
        send_cmd("hip_knee_right",    dg); 
        send_cmd("knee_ankle_right",  -mct);
        send_cmd("ankle_ankle_right", mcn);
        
        // Giữ hông giữa thẳng
        send_cmd("base_hip_middle", 0.0);

        // --- 3. GỬI LỆNH KHÓA TAY (GÓC 0) ---
        // Giữ tay ép sát người hoặc duỗi thẳng để ổn định trọng tâm
        // Tùy vào trục của bạn, 0.0 thường là duỗi thẳng hoặc dang ngang (T-pose)
        
        // Tay Trái
        send_cmd("hip_shoulder_left",      0.0);
        send_cmd("shoulder_shoulder_left", 0.0);
        send_cmd("shoulder_elbow_left",    0.0);

        // Tay Phải
        send_cmd("hip_shoulder_right",      0.0);
        send_cmd("shoulder_shoulder_right", 0.0);
        send_cmd("shoulder_elbow_right",    0.0);
    }

    double dxi, dyi, autoH, tm, theta_x, theta_y, Lyh, Lxh, H_tmp;
    double gain_pitch, gain_roll, recovery;
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> pubs_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr angle_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UvcControllerNode>());
    rclcpp::shutdown();
    return 0;
}
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>
#include <algorithm>
#include <map>
#include <string>

const double L1 = 20.0;     
const double L2 = 0.1;  
const double L3 = 60.0;     
const double L4 = 100.0;
const double L5 = 65.0;   //65  
 
const double HEIGHT_STD = 244.0; 

class UvcControllerNode : public rclcpp::Node {
public:
    UvcControllerNode() : Node("uvc_node") {
        dxi = 0.0; dyi = 0.0; 
        autoH = HEIGHT_STD;
        
        // Gain cho MG996R (như đã tune)
        gain_pitch = 0.2;   
        gain_roll =  0.2;    
        recovery = 0.01; 

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
        double pitch_rad = msg->x * M_PI / 180.0;
        double roll_rad  = msg->y * M_PI / 180.0;
        double autoH_tmp;
        double theta_x = atan2(dxi, autoH)-pitch_rad * gain_pitch;
        double Lx = autoH / cos(theta_x);

        // theta_x -= pitch_rad * gain_pitch;

        dxi   = Lx * sin(theta_x);
        autoH_tmp = Lx * cos(theta_x);

        /* ---------- ROLL (Y) ----------- */
        double theta_y = atan2(dyi - L2, autoH_tmp)-roll_rad * gain_roll;;
        double Ly = autoH_tmp / cos(theta_y);

        // theta_y -= roll_rad * gain_roll;

        dyi   = Ly * sin(theta_y);
        autoH = Ly * cos(theta_y);

        /* ---------- DAMPING ---------- */
        if (fabs(dxi) < 1) dxi = 0;
        else dxi -= dxi * recovery;

        if (fabs(dyi) < 1) dyi = 0;
        else dyi -= dyi * recovery;

        autoH += (HEIGHT_STD - autoH) * recovery;
        dxi=std::clamp(dxi, -50.0, 50.0);
        dyi=std::clamp(dyi, -50.0, 50.0);
        autoH=std::clamp(autoH, 220.0, 260.0);
        solve_ik(dxi, dyi, autoH, hn, ht, dg, mct, mcn);
        std::cout<<dxi<<"   "<<dyi<<"   "<<autoH<<"   "<<hn<<"   "<<ht<<"   "<<dg<<"   "<<mct<<"   "<<mcn<<std::endl;
        if(!std::isnan(ht) && !std::isnan(dg)) {
            publish_command(hn, ht, dg, mct, mcn);
        }
    }

    void solve_ik(double dx, double dy, double h, 
                  double &hn, double &ht, double &dg, double &mct, double &mcn) {
        // double L_sq = dx*dx + dy*dy + h*h;
        // double L = std::sqrt(L_sq);

        // if (L > (L1 + L2 - 0.1)) { L = L1 + L2 - 0.1; L_sq = L*L; }
        // if (L < abs(L1 - L2) + 0.1) { L = abs(L1 - L2) + 0.1; L_sq = L*L; }

        // hn = atan2(dy, h);

        // double cos_dg = (L1*L1 + L2*L2 - L_sq) / (2*L1*L2);
        // dg = acos(std::clamp(cos_dg, -1.0, 1.0)); 

        // double phi = atan2(dx, sqrt(dy*dy + h*h)); 
        // double cos_ht = (L1*L1 + L_sq - L2*L2) / (2*L1*L);
        // ht = phi + acos(std::clamp(cos_ht, -1.0, 1.0));

        // mct = (-ht + dg); 
        // mcn = -hn;    
        L=sqrt(dx*dx+dy*dy+h*h);
        a = sqrt((dy-L2)*(dy-L2)+(Lxh-L5)*(Lxh-L5))+ 1e-6;
        // double a_max = L3 + L4 - 1e-6;
        // double a_min = fabs(L3 - L4) + 1e-6;
        // a = std::clamp(a, a_min, a_max);
        hn = asin(std::clamp(dx/Lxh, -1.0, 1.0));
        theta_2a = atan2(dy-L2, Lxh-L5);
        
        theta_2b = acos(std::clamp((L3*L3+a*a-L4*L4)/(2*a*L3), -1.0, 1.0));
        ht=theta_2a+theta_2b;
        dg = acos(std::clamp((L3*L3+L4*L4-a*a)/(2*L3*L4), -1.0, 1.0));
                    
        mct = M_PI - ht+dg;
        mcn = M_PI/2- hn;



        

    }

    void publish_command(double hn, double ht, double dg, double mct, double mcn) {
        // --- CHÂN TRÁI ---
        send_cmd("base_hip_left",   -hn);
        send_cmd("hip_hip_left",    ht);
        send_cmd("hip_knee_left",    -dg); 
        send_cmd("knee_ankle_left", mct);
        send_cmd("ankle_ankle_left",-mcn);

        // --- CHÂN PHẢI (Mirror) ---
        send_cmd("base_hip_right",    hn);
        send_cmd("hip_hip_right",    -ht);
        send_cmd("hip_knee_right",    dg); 
        send_cmd("knee_ankle_right",  -mct);
        send_cmd("ankle_ankle_right", -mcn);
        
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

    double a,b;
    double theta_2a;
    double theta_2b;
    double theta_2c;
    double theta_tmp;
    double hn, ht, dg, mct, mcn;
    double dxi, dyi, autoH, tm, theta_x, theta_y, Lyh, Lxh,L, H_tmp;
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
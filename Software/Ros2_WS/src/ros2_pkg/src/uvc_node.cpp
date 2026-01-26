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
 
const double HEIGHT_STD = 224.0; 

class UvcControllerNode : public rclcpp::Node {
    public:
        UvcControllerNode() : Node("uvc_node") {
            x = 0.0; y = 0.0; 
            z = HEIGHT_STD-5;
            
            // Gain cho MG996R (như đã tune)
            gain_pitch = 0.01;   
            gain_roll =  0.01;    
            recovery = 0.04; 

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
            double roll_rad = msg->y * M_PI / 180.0;
            double x=0.0, y=0.0, z=224.0;
            solve_ik(x, y, z, hn, ht, dg, mct, mcn);
            publish_command(hn, ht, dg, mct, mcn);
            std::cout << "x:" << x << " y:" << y << " z:" << z << " hn:" << hn << " ht:" << ht << " dg:" << dg << " mct:" << mct << " mcn:" << mcn << "\n";
        }


        void solve_ik(double dx, double dy, double dz,
            double &hn, double &ht, double &dg, double &mct, double &mcn) {
            hn=atan2(dy,dz);
            double cos_theta3 =(pow((sqrt(dz*dz+dy*dy)-L5),2)+dx*dx-L3*L3-L4*L4)/(2*L3*L4);
            double sin_theta3=sqrt(1-cos_theta3*cos_theta3);
            dg=atan2(sin_theta3,cos_theta3);
            ht=atan2(sin_theta3*L4, L3+cos_theta3*L4)+atan2(dx, sqrt(dz*dz+dy*dy)-L5);
            mct = dg-ht;
            mcn=-hn;

        } 
        void publish_command(double hn, double ht, double dg, double mct, double mcn) {
            // --- CHÂN TRÁI ---
            send_cmd("base_hip_left", -hn);
            send_cmd("hip_hip_left", ht);
            send_cmd("hip_knee_left", -dg);
            send_cmd("knee_ankle_left", mct);
            send_cmd("ankle_ankle_left",-mcn);
            // --- CHÂN PHẢI (Mirror) ---
            send_cmd("base_hip_right", hn);
            send_cmd("hip_hip_right", -ht);
            send_cmd("hip_knee_right", dg);
            send_cmd("knee_ankle_right", -mct);
            send_cmd("ankle_ankle_right", -mcn);
            send_cmd("base_hip_middle", 0.0);

            send_cmd("hip_shoulder_left", 0.0);
            send_cmd("shoulder_shoulder_left", 0.0);
            send_cmd("shoulder_elbow_left", 0.0);
            send_cmd("hip_shoulder_right", 0.0);

            send_cmd("shoulder_shoulder_right", 0.0);

            send_cmd("shoulder_elbow_right", 0.0);

        }
        double a,b;
        double theta_2a;
        double theta_2b;
        double theta_2c;
        double theta_tmp;
        double hn, ht, dg, mct, mcn;
        double x, y, z, tm, theta_x, theta_y, Lyh, Lxh,L, H_tmp;
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
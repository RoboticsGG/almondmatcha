#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "msgs_ifaces/msg/gnss_data.hpp"
#include "msgs_ifaces/msg/main_rocon.hpp"
#include "msgs_ifaces/msg/sub_rocon.hpp"

class NodeMonitoring : public rclcpp::Node {
public:
    NodeMonitoring() : Node("topic_subscriber") {
        sub_cc_rcon_ = this->create_subscription<std_msgs::msg::Bool>(
            "/cc_rcon", 10, 
            std::bind(&NodeMonitoring::cc_rcon_callback, this, std::placeholders::_1)
        );
        
        sub_dis_remain_ = this->create_subscription<std_msgs::msg::Float64>(
            "/dis_remain", 10, 
            std::bind(&NodeMonitoring::dis_remain_callback, this, std::placeholders::_1)
        );
        
        sub_gnss_data_ = this->create_subscription<msgs_ifaces::msg::GnssData>(
            "/gnss_data", 10, 
            std::bind(&NodeMonitoring::gnss_data_callback, this, std::placeholders::_1)
        );
        
        sub_pub_despose_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/pub_despose", 10, 
            std::bind(&NodeMonitoring::pub_despose_callback, this, std::placeholders::_1)
        );
        
        sub_pub_rovercontrol_d2_ = this->create_subscription<msgs_ifaces::msg::MainRocon>(
            "/pub_rovercontrol_d2", 10, 
            std::bind(&NodeMonitoring::pub_rovercontrol_callback, this, std::placeholders::_1)
        );

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000), 
            std::bind(&NodeMonitoring::timer_callback, this)
        );

        RCLCPP_INFO(this->get_logger(), "Node Monitoring Initialized.");
    }

private:
    bool cc_rcon_;
    float dis_remain_;
    float current_lat_;
    float current_long_;
    float des_lat_;
    float des_long_;

    uint8_t fdr_msg_;
    float ro_ctrl_msg_;
    uint8_t spd_msg_;
    uint8_t bdr_msg_;

    std::string f_dir_;
    std::string b_dir_;

    msgs_ifaces::msg::GnssData cur_pose_msg_;

    void cc_rcon_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        cc_rcon_ = msg->data;
    }
    
    void dis_remain_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        dis_remain_ = msg->data;
    }
    
    void gnss_data_callback(const msgs_ifaces::msg::GnssData::SharedPtr msg) {
        current_lat_ = msg->latitude;
        current_long_ = msg->longitude;
    }
    
    void pub_despose_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        des_lat_ = msg->data[0];
        des_long_ = msg->data[1];
    }
    
    void pub_rovercontrol_callback(const msgs_ifaces::msg::MainRocon::SharedPtr msg) {
        fdr_msg_ = msg->mainrocon_msg.fdr_msg;
        ro_ctrl_msg_ = msg->mainrocon_msg.ro_ctrl_msg;
        spd_msg_ = msg->mainrocon_msg.spd_msg;
        bdr_msg_ = msg->mainrocon_msg.bdr_msg;

        if (fdr_msg_ == 1) {
            f_dir_ = "Turn Left: " + std::to_string(ro_ctrl_msg_) + " Degree";
        } else if (fdr_msg_ == 3) {
            f_dir_ = "Turn Right: " + std::to_string(ro_ctrl_msg_) + " Degree";
        } else {
            f_dir_ = "Stay Middle";
        }

        if (bdr_msg_ == 1 && cc_rcon_==false) {
            b_dir_ = "Forward at speed: " + std::to_string(spd_msg_) + "%";
        } else if (bdr_msg_ == 2 && cc_rcon_==false) {
            b_dir_ = "Backward at speed: " + std::to_string(spd_msg_) + "%";
        } else {
            b_dir_ = "Stop";
        }
    }
    
    void timer_callback() {
        RCLCPP_INFO(this->get_logger(), "########## Status Update ##########");
        RCLCPP_INFO(this->get_logger(), "Target coordinates: %f, %f", des_lat_, des_long_);
        RCLCPP_INFO(this->get_logger(), "Current coordinates: %f, %f", current_lat_, current_long_);
        RCLCPP_INFO(this->get_logger(), "Remaining distance: %f KM", dis_remain_);
        RCLCPP_INFO(this->get_logger(), "Rover Front Direction: %s", f_dir_.c_str());
        RCLCPP_INFO(this->get_logger(), "Rover Back Direction: %s", b_dir_.c_str());
        RCLCPP_INFO(this->get_logger(), "##################################");
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_cc_rcon_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_dis_remain_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_pub_despose_;
    rclcpp::Subscription<msgs_ifaces::msg::GnssData>::SharedPtr sub_gnss_data_;
    rclcpp::Subscription<msgs_ifaces::msg::MainRocon>::SharedPtr sub_pub_rovercontrol_d2_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NodeMonitoring>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
#include "rclcpp/rclcpp.hpp"
#include "msgs_ifaces/msg/rover_status.hpp"

class NodeBaseMonitoring : public rclcpp::Node {
public:
    NodeBaseMonitoring() : Node("node_base_monitoring") {
        
        sub_rover_status_ = this->create_subscription<msgs_ifaces::msg::RoverStatus>(
            "/tpc_rover_status", 10,
            std::bind(&NodeBaseMonitoring::rover_status_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Base Station Monitoring Node Initialized");
    }

private:
    void rover_status_callback(const msgs_ifaces::msg::RoverStatus::SharedPtr msg) {
        std::string f_dir, b_dir;
        
        if (msg->fdr_msg == 1) {
            f_dir = "Turn Left: " + std::to_string(msg->ro_ctrl_msg) + " Degree";
        } else if (msg->fdr_msg == 3) {
            f_dir = "Turn Right: " + std::to_string(msg->ro_ctrl_msg) + " Degree";
        } else {
            f_dir = "Stay Middle";
        }

        if (msg->bdr_msg == 1 && !msg->mission_active) {
            b_dir = "Forward at speed: " + std::to_string(msg->spd_msg) + "%";
        } else if (msg->bdr_msg == 2 && !msg->mission_active) {
            b_dir = "Backward at speed: " + std::to_string(msg->spd_msg) + "%";
        } else {
            b_dir = "Stop";
        }
        
        RCLCPP_INFO(this->get_logger(), "########## Rover Status Update ##########");
        RCLCPP_INFO(this->get_logger(), "Target coordinates: %.6f, %.6f", 
            msg->destination_latitude, msg->destination_longitude);
        RCLCPP_INFO(this->get_logger(), "Current coordinates: %.6f, %.6f", 
            msg->current_latitude, msg->current_longitude);
        RCLCPP_INFO(this->get_logger(), "RTK Position: %.6f, %.6f | Alt: %.2fm | Fix: %s | Err: %.1fcm | Sats: %d",
            msg->rtk_latitude, msg->rtk_longitude, msg->rtk_altitude, 
            msg->rtk_fix_quality.c_str(), msg->rtk_centimeter_error, msg->rtk_satellites);
        RCLCPP_INFO(this->get_logger(), "Remaining distance: %.3f KM", msg->distance_remaining);
        RCLCPP_INFO(this->get_logger(), "Mission Active: %s", msg->mission_active ? "YES" : "NO");
        RCLCPP_INFO(this->get_logger(), "Rover Front Direction: %s", f_dir.c_str());
        RCLCPP_INFO(this->get_logger(), "Rover Back Direction: %s", b_dir.c_str());
        RCLCPP_INFO(this->get_logger(), "########################################");
    }

    rclcpp::Subscription<msgs_ifaces::msg::RoverStatus>::SharedPtr sub_rover_status_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NodeBaseMonitoring>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

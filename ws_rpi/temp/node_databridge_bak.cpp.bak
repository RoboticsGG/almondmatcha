// ROS2 Core
#include <rclcpp/rclcpp.hpp>

// Standard Library
#include <chrono>
#include <memory>
#include <mutex>

// Custom Interfaces
#include <msgs_ifaces/msg/main_rocon.hpp>
#include <msgs_ifaces/msg/sub_rocon.hpp>

/**
 * @brief Data Bridge Node for Domain Communication
 * 
 * This node bridges communication between two ROS2 domains:
 * - Subscribes from Domain ID 5: Topic "pub_rovercontrol_d5"
 * - Publishes to Domain ID 2: Topic "pub_rovercontrol"
 * 
 * Acts as a relay to forward control messages between domains at 50Hz (20ms period).
 */
class DataBridge : public rclcpp::Node {
public:
    DataBridge() : Node("data_bridge") {
        // Publisher to Domain ID 2
        publisher_domain2_ = this->create_publisher<msgs_ifaces::msg::MainRocon>(
            "pub_rovercontrol", 10
        );

        // Subscriber from Domain ID 5
        subscriber_domain5_ = this->create_subscription<msgs_ifaces::msg::MainRocon>(
            "pub_rovercontrol_d5", 10, 
            std::bind(&DataBridge::messageCallback, this, std::placeholders::_1)
        );

        // Timer for periodic republishing at 50Hz (20ms)
        republish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(REPUBLISH_PERIOD_MS), 
            std::bind(&DataBridge::republishCallback, this)
        );

        RCLCPP_INFO(this->get_logger(), 
                   "Data Bridge initialized: Domain 5 -> Domain 2 (50Hz)");
    }

private:
    // === Configuration ===
    static constexpr int REPUBLISH_PERIOD_MS = 20;  // 50Hz republish rate
    
    // === ROS2 Communication ===
    rclcpp::Publisher<msgs_ifaces::msg::MainRocon>::SharedPtr publisher_domain2_;
    rclcpp::Subscription<msgs_ifaces::msg::MainRocon>::SharedPtr subscriber_domain5_;
    rclcpp::TimerBase::SharedPtr republish_timer_;
    
    // === State Variables ===
    msgs_ifaces::msg::MainRocon latest_message_;
    bool message_received_ = false;
    bool warning_enabled_ = true;  // Flag to show warning only once


    bool warning_enabled_ = true;  // Flag to show warning only once
    
    // === Callback Methods ===
    
    /**
     * @brief Callback for receiving messages from Domain 5
     * @param msg MainRocon message from Domain 5
     * 
     * Stores the latest message for periodic republishing to Domain 2
     */
    void messageCallback(const msgs_ifaces::msg::MainRocon::SharedPtr msg) {
        latest_message_ = *msg;
        message_received_ = true;

        RCLCPP_INFO(this->get_logger(), 
                   "Received from Domain 5 [Dir:%d, Steer:%.2f, Speed:%d, Back:%d]",
                   msg->mainrocon_msg.fdr_msg, 
                   msg->mainrocon_msg.ro_ctrl_msg, 
                   msg->mainrocon_msg.spd_msg, 
                   msg->mainrocon_msg.bdr_msg);
    }

    /**
     * @brief Timer callback for periodic republishing (50Hz)
     * 
     * Republishes the latest message from Domain 5 to Domain 2.
     * Shows warning only once if no messages have been received yet.
     */
    void republishCallback() {
        if (!message_received_) {
            if (warning_enabled_) {
                RCLCPP_WARN(this->get_logger(), 
                           "Waiting for messages from Domain 5 'pub_rovercontrol_d5'...");
                warning_enabled_ = false;  // Show warning only once
            }
            return;
        }

        // Republish latest message to Domain 2
        publisher_domain2_->publish(latest_message_);

        RCLCPP_INFO(this->get_logger(), 
                   "Republished to Domain 2 [Dir:%d, Steer:%.2f, Speed:%d, Back:%d]",
                   latest_message_.mainrocon_msg.fdr_msg, 
                   latest_message_.mainrocon_msg.ro_ctrl_msg, 
                   latest_message_.mainrocon_msg.spd_msg, 
                   latest_message_.mainrocon_msg.bdr_msg);
    }
};

/**
 * @brief Main entry point for the data bridge node
 */
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<DataBridge>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("data_bridge"), 
                    "Exception in data bridge: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
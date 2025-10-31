// ROS2 Core
#include <rclcpp/rclcpp.hpp>

// Standard Library
#include <chrono>
#include <memory>
#include <mutex>

// Custom Interfaces
#include <msgs_ifaces/msg/chassis_ctrl.hpp>

/**
 * @brief Domain Communication Bridge Node
 * 
 * This node bridges ROS2 communication between two domains:
 * - Subscribes from Domain ID 5: Topic "pub_rovercontrol_d5"
 * - Publishes to Domain ID 2: Topic "pub_rovercontrol"
 * 
 * Acts as a relay to forward control messages between domains at 50Hz (20ms period).
 */
class DomainBridge : public rclcpp::Node {
public:
    DomainBridge() : Node("domain_bridge") {
        // Publisher to Domain ID 2
        domain2_publisher_ = this->create_publisher<msgs_ifaces::msg::ChassisCtrl>(
            "pub_rovercontrol", 10
        );

        // Subscriber from Domain ID 5
        domain5_subscriber_ = this->create_subscription<msgs_ifaces::msg::ChassisCtrl>(
            "pub_rovercontrol_d5", 10, 
            std::bind(&DomainBridge::onDomain5MessageReceived, this, std::placeholders::_1)
        );

        // Timer for periodic republishing at 50Hz (20ms)
        bridge_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(BRIDGE_PERIOD_MS), 
            std::bind(&DomainBridge::bridgeTimerCallback, this)
        );

        RCLCPP_INFO(this->get_logger(), 
                   "Domain Bridge initialized: Domain 5 -> Domain 2 (50Hz)");
    }

private:
    // === Configuration ===
    static constexpr int BRIDGE_PERIOD_MS = 20;  // 50Hz bridge rate
    
    // === ROS2 Communication ===
    rclcpp::Publisher<msgs_ifaces::msg::ChassisCtrl>::SharedPtr domain2_publisher_;
    rclcpp::Subscription<msgs_ifaces::msg::ChassisCtrl>::SharedPtr domain5_subscriber_;
    rclcpp::TimerBase::SharedPtr bridge_timer_;
    
    // === State Variables ===
    msgs_ifaces::msg::ChassisCtrl latest_control_message_;
    bool has_received_message_ = false;
    bool show_waiting_warning_ = true;  // Flag to show warning only once
    
    // === Callback Methods ===
    
    /**
     * @brief Callback for receiving messages from Domain 5
     * @param msg ChassisCtrl message from Domain 5
     * 
     * Stores the latest control message for periodic bridging to Domain 2
     */
    void onDomain5MessageReceived(const msgs_ifaces::msg::ChassisCtrl::SharedPtr msg) {
        latest_control_message_ = *msg;
        has_received_message_ = true;

        RCLCPP_INFO(this->get_logger(), 
                   "Received from Domain 5 [Dir:%d, Steer:%.2f, Speed:%d, Back:%d]",
                   msg->fdr_msg, 
                   msg->ro_ctrl_msg, 
                   msg->spd_msg, 
                   msg->bdr_msg);
    }

    /**
     * @brief Timer callback for periodic bridging (50Hz)
     * 
     * Forwards the latest control message from Domain 5 to Domain 2.
     * Shows warning only once if no messages have been received yet.
     */
    void bridgeTimerCallback() {
        if (!has_received_message_) {
            if (show_waiting_warning_) {
                RCLCPP_WARN(this->get_logger(), 
                           "Waiting for messages from Domain 5 on 'pub_rovercontrol_d5'...");
                show_waiting_warning_ = false;  // Show warning only once
            }
            return;
        }

        // Bridge latest control message to Domain 2
        domain2_publisher_->publish(latest_control_message_);

        RCLCPP_INFO(this->get_logger(), 
                   "Bridged to Domain 2 [Dir:%d, Steer:%.2f, Speed:%d, Back:%d]",
                   latest_control_message_.fdr_msg, 
                   latest_control_message_.ro_ctrl_msg, 
                   latest_control_message_.spd_msg, 
                   latest_control_message_.bdr_msg);
    }
};

/**
 * @brief Main entry point for the domain bridge node
 */
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<DomainBridge>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("domain_bridge"), 
                    "Exception in domain bridge: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
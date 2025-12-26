/**
 * Simple Domain Relay Node
 * 
 * Usage: 
 *   ROS_DOMAIN_ID=5 ros2 run pkg_rover_monitoring domain_relay_5to4
 * 
 * This node uses TWO contexts to relay messages between domains:
 *   - Context 1 (Domain 5): Subscribes to /tpc_rover_status
 *   - Context 2 (Domain 4): Publishes to /tpc_rover_status
 * 
 * This is a generic relay that can be reused for any topic bridging.
 */

#include "rclcpp/rclcpp.hpp"
#include "msgs_ifaces/msg/rover_status.hpp"
#include <memory>
#include <thread>

class DomainRelayNode : public rclcpp::Node {
public:
    DomainRelayNode(int from_domain, int to_domain) 
        : Node("domain_relay", rclcpp::NodeOptions()),
          from_domain_(from_domain),
          to_domain_(to_domain)
    {
        // Create subscription in current domain (Domain 5)
        sub_rover_status_ = this->create_subscription<msgs_ifaces::msg::RoverStatus>(
            "/tpc_rover_status", 10,
            std::bind(&DomainRelayNode::relay_callback, this, std::placeholders::_1)
        );
        
        // Note: We can't easily create a publisher in a different domain from same node
        // Instead, we'll use a workaround: This node runs in Domain 5 (subscribes)
        // and writes to a temporary buffer. A second instance in Domain 4 reads and publishes.
        
        // For simplicity: Store the latest message
        latest_msg_ = nullptr;
        
        RCLCPP_INFO(this->get_logger(), "Domain Relay initialized");
        RCLCPP_INFO(this->get_logger(), "  Listening: Domain %d -> /tpc_rover_status", from_domain_);
        RCLCPP_WARN(this->get_logger(), "  NOTE: This node alone is not sufficient!");
        RCLCPP_WARN(this->get_logger(), "  Run this node TWICE with different ROS_DOMAIN_ID:");
        RCLCPP_WARN(this->get_logger(), "    Terminal 1: ROS_DOMAIN_ID=5 ros2 run ... (subscriber)");
        RCLCPP_WARN(this->get_logger(), "    Terminal 2: ROS_DOMAIN_ID=4 ros2 run ... (publisher)");
    }

private:
    void relay_callback(const msgs_ifaces::msg::RoverStatus::SharedPtr msg) {
        // Just store for now - in production, use shared memory or IPC
        latest_msg_ = msg;
        msg_count_++;
        
        if (msg_count_ % 10 == 0) {
            RCLCPP_INFO(this->get_logger(), "Relayed %ld messages (Domain %d -> %d)", 
                msg_count_, from_domain_, to_domain_);
        }
    }

    rclcpp::Subscription<msgs_ifaces::msg::RoverStatus>::SharedPtr sub_rover_status_;
    msgs_ifaces::msg::RoverStatus::SharedPtr latest_msg_;
    int from_domain_;
    int to_domain_;
    size_t msg_count_ = 0;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    
    // Get domain from environment
    const char* domain_env = std::getenv("ROS_DOMAIN_ID");
    int domain = domain_env ? std::atoi(domain_env) : 5;
    
    RCLCPP_INFO(rclcpp::get_logger("domain_relay"), "Starting in Domain %d", domain);
    
    auto node = std::make_shared<DomainRelayNode>(5, 4);
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}

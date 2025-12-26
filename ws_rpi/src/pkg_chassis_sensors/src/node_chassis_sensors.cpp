#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "msgs_ifaces/msg/chassis_sensors.hpp"

using namespace std::chrono_literals;

class ChassisSensorsNode : public rclcpp::Node {
public:
    ChassisSensorsNode() : Node("chassis_sensors_node") {
        setupSubscriber();
        
        // Heartbeat timer to show node is alive
        heartbeat_timer_ = this->create_wall_timer(5s, std::bind(&ChassisSensorsNode::heartbeat, this));
        
        RCLCPP_INFO(this->get_logger(), "Chassis Sensors Node initialized. Waiting for STM32 data on /tpc_chassis_sensors...");
        RCLCPP_INFO(this->get_logger(), "CSV logging handled by node_rover_monitoring");
    }

private:
    // ROS2 Components
    rclcpp::Subscription<msgs_ifaces::msg::ChassisSensors>::SharedPtr sub_chassis_sensors_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    
    // Data Management
    bool has_data_ = false;
    uint32_t message_count_ = 0;

    void setupSubscriber() {
        // QoS must match STM32 mbed publishers: best_effort + volatile
        rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        qos_profile.best_effort();

        sub_chassis_sensors_ = this->create_subscription<msgs_ifaces::msg::ChassisSensors>(
            "/tpc_chassis_sensors", qos_profile,
            std::bind(&ChassisSensorsNode::sensorsCallback, this, std::placeholders::_1)
        );
        
        RCLCPP_INFO(this->get_logger(), "Subscribed to /tpc_chassis_sensors");
    }

    void sensorsCallback(const msgs_ifaces::msg::ChassisSensors::SharedPtr msg) {
        has_data_ = true;
        message_count_++;

        RCLCPP_INFO(this->get_logger(),
                    "Chassis Sensors [#%u] - Encoders: [L:%d, R:%d] | Power: [I:%.2fA, V:%.2fV]",
                    message_count_,
                    msg->mt_lf_encode_msg, msg->mt_rt_encode_msg,
                    msg->sys_current_msg, msg->sys_volt_msg);
    }
    
    void heartbeat() {
        if (!has_data_) {
            RCLCPP_WARN(this->get_logger(), "Still waiting for STM32 sensor data... (no messages received yet)");
        } else {
            RCLCPP_INFO(this->get_logger(), "Chassis Sensors alive - received %u messages", message_count_);
        }
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ChassisSensorsNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
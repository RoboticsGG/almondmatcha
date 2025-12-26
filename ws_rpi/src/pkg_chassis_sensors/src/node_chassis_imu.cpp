#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "msgs_ifaces/msg/chassis_imu.hpp"

using namespace std::chrono_literals;

class ChassisIMUNode : public rclcpp::Node {
public:
    ChassisIMUNode() : Node("chassis_imu_node") {
        setupSubscriber();
        
        // Heartbeat timer to show node is alive
        heartbeat_timer_ = this->create_wall_timer(5s, std::bind(&ChassisIMUNode::heartbeat, this));
        
        RCLCPP_INFO(this->get_logger(), "Chassis IMU Node initialized. Waiting for STM32 data on /tpc_chassis_imu...");
        RCLCPP_INFO(this->get_logger(), "CSV logging handled by node_rover_monitoring");
    }

private:
    // ROS2 Components
    rclcpp::Subscription<msgs_ifaces::msg::ChassisIMU>::SharedPtr sub_chassis_imu_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    
    // Data Management
    bool has_data_ = false;
    uint32_t message_count_ = 0;

    void setupSubscriber() {
        // QoS must match STM32 mbed publishers: best_effort + volatile
        rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        qos_profile.best_effort();

        sub_chassis_imu_ = this->create_subscription<msgs_ifaces::msg::ChassisIMU>(
            "/tpc_chassis_imu", qos_profile,
            std::bind(&ChassisIMUNode::imuCallback, this, std::placeholders::_1)
        );
        
        RCLCPP_INFO(this->get_logger(), "Subscribed to /tpc_chassis_imu");
    }

    void imuCallback(const msgs_ifaces::msg::ChassisIMU::SharedPtr msg) {
        has_data_ = true;
        message_count_++;

        RCLCPP_INFO(this->get_logger(),
                    "Chassis IMU [#%u] - Accel: [%d, %d, %d] | Gyro: [%d, %d, %d]",
                    message_count_,
                    msg->accel_x, msg->accel_y, msg->accel_z,
                    msg->gyro_x, msg->gyro_y, msg->gyro_z);
    }
    
    void heartbeat() {
        if (!has_data_) {
            RCLCPP_WARN(this->get_logger(), "Still waiting for STM32 IMU data... (no messages received yet)");
        } else {
            RCLCPP_INFO(this->get_logger(), "Chassis IMU alive - received %u messages", message_count_);
        }
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ChassisIMUNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

/*
 * Base Station Bridge Node
 * 
 * Purpose:
 *   Multi-domain bridge relay between Domain 5 (rover internal) and Domain 2 (base station).
 *   Enables telemetry monitoring and command relay for ground station communication.
 * 
 * Architecture:
 *   - Domain 5 (PRIMARY): Subscribes to rover sensor/state topics
 *   - Domain 2 (SECONDARY): Publishes aggregated telemetry, subscribes to commands
 * 
 * Domain 5 Subscriptions (Rover Internal):
 *   - tpc_chassis_imu: Raw IMU data from STM32
 *   - tpc_chassis_sensors: Raw encoder/power data from STM32
 *   - tpc_gnss_spresense: GNSS position data
 *   - tpc_chassis_cmd: Motor control commands
 *   - tpc_gnss_mission_active: Mission status
 *   - tpc_gnss_mission_remain_dist: Distance to waypoint
 * 
 * Domain 2 Publications (Base Station):
 *   - tpc_chassis_imu: Relayed IMU data
 *   - tpc_chassis_sensors: Relayed encoder/power data
 *   - tpc_gnss_spresense: Relayed GNSS data
 *   - tpc_chassis_cmd: Relayed motor commands
 *   - tpc_gnss_mission_active: Relayed mission status
 *   - tpc_gnss_mission_remain_dist: Relayed distance data
 * 
 * Domain 2 Subscriptions (Base Station Commands):
 *   - tpc_rover_dest_coordinate: Target waypoint from base
 * 
 * Domain 5 Publications (Rover Commands):
 *   - tpc_rover_dest_coordinate: Relayed target waypoint
 * 
 * Author: Almondmatcha Development Team
 * Date: November 6, 2025
 */

#include <rclcpp/rclcpp.hpp>
#include <msgs_ifaces/msg/chassis_imu.hpp>
#include <msgs_ifaces/msg/chassis_sensors.hpp>
#include <msgs_ifaces/msg/spresense_gnss.hpp>
#include <msgs_ifaces/msg/chassis_ctrl.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <chrono>
#include <memory>

class BaseBridgeNode : public rclcpp::Node {
public:
    BaseBridgeNode() : Node("base_bridge_node", 
                            rclcpp::NodeOptions()
                                .allow_undeclared_parameters(true)
                                .automatically_declare_parameters_from_overrides(true)) {
        
        RCLCPP_INFO(this->get_logger(), "=== Base Bridge Node Initializing ===");
        RCLCPP_INFO(this->get_logger(), "Multi-domain bridge: Domain 5 (rover) <-> Domain 2 (base)");
        
        init_domain5_subscribers();
        init_domain2_publishers();
        init_domain2_subscribers();
        init_domain5_publishers();
        
        RCLCPP_INFO(this->get_logger(), "=== Base Bridge Node Ready ===");
    }

private:
    // ===================== Domain 5 Subscribers (Rover -> Bridge) =====================
    rclcpp::Subscription<msgs_ifaces::msg::ChassisIMU>::SharedPtr sub_d5_imu_;
    rclcpp::Subscription<msgs_ifaces::msg::ChassisSensors>::SharedPtr sub_d5_sensors_;
    rclcpp::Subscription<msgs_ifaces::msg::SpresenseGNSS>::SharedPtr sub_d5_gnss_;
    rclcpp::Subscription<msgs_ifaces::msg::ChassisCtrl>::SharedPtr sub_d5_chassis_cmd_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_d5_mission_active_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_d5_distance_;

    // ===================== Domain 2 Publishers (Bridge -> Base) =====================
    rclcpp::Publisher<msgs_ifaces::msg::ChassisIMU>::SharedPtr pub_d2_imu_;
    rclcpp::Publisher<msgs_ifaces::msg::ChassisSensors>::SharedPtr pub_d2_sensors_;
    rclcpp::Publisher<msgs_ifaces::msg::SpresenseGNSS>::SharedPtr pub_d2_gnss_;
    rclcpp::Publisher<msgs_ifaces::msg::ChassisCtrl>::SharedPtr pub_d2_chassis_cmd_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_d2_mission_active_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_d2_distance_;

    // ===================== Domain 2 Subscribers (Base -> Bridge) =====================
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_d2_dest_coord_;

    // ===================== Domain 5 Publishers (Bridge -> Rover) =====================
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_d5_dest_coord_;

    /**
     * Initialize Domain 5 subscribers (rover internal topics)
     * These topics originate from rover nodes and are relayed to base station
     */
    void init_domain5_subscribers() {
        RCLCPP_INFO(this->get_logger(), "Initializing Domain 5 subscribers...");
        
        // IMU data from STM32
        sub_d5_imu_ = this->create_subscription<msgs_ifaces::msg::ChassisIMU>(
            "tpc_chassis_imu", 10,
            std::bind(&BaseBridgeNode::on_d5_imu, this, std::placeholders::_1)
        );
        
        // Encoder/power data from STM32
        sub_d5_sensors_ = this->create_subscription<msgs_ifaces::msg::ChassisSensors>(
            "tpc_chassis_sensors", 10,
            std::bind(&BaseBridgeNode::on_d5_sensors, this, std::placeholders::_1)
        );
        
        // GNSS position data
        sub_d5_gnss_ = this->create_subscription<msgs_ifaces::msg::SpresenseGNSS>(
            "tpc_gnss_spresense", 10,
            std::bind(&BaseBridgeNode::on_d5_gnss, this, std::placeholders::_1)
        );
        
        // Chassis motor commands
        sub_d5_chassis_cmd_ = this->create_subscription<msgs_ifaces::msg::ChassisCtrl>(
            "tpc_chassis_cmd", 10,
            std::bind(&BaseBridgeNode::on_d5_chassis_cmd, this, std::placeholders::_1)
        );
        
        // Mission active status
            sub_d5_mission_active_ = this->create_subscription<std_msgs::msg::Bool>(
                "tpc_gnss_mission_active", 10,
                [this](const std::shared_ptr<const std_msgs::msg::Bool> msg) {
                    pub_d2_mission_active_->publish(*msg);
                }
            );

            // Distance remaining
            sub_d5_distance_ = this->create_subscription<std_msgs::msg::Float64>(
                "tpc_gnss_mission_remain_dist", 10,
                [this](const std::shared_ptr<const std_msgs::msg::Float64> msg) {
                    pub_d2_distance_->publish(*msg);
                }
            );
        
        RCLCPP_INFO(this->get_logger(), "Domain 5 subscribers initialized (6 topics)");
    }

    /**
     * Initialize Domain 2 publishers (relay to base station)
     * 
     * NOTE: This node should be run TWICE with different ROS_DOMAIN_ID:
     *   Process 1: ROS_DOMAIN_ID=5 (subscribes from rover, publishes relay flag)
     *   Process 2: ROS_DOMAIN_ID=2 (subscribes relay flag, publishes to base)
     * 
     * For now, we create a simpler single-domain relay that depends on
     * DDS domain routing configuration or can be run with domain parameter.
     */
    void init_domain2_publishers() {
        RCLCPP_INFO(this->get_logger(), "Initializing relay publishers...");
        
        pub_d2_imu_ = this->create_publisher<msgs_ifaces::msg::ChassisIMU>(
            "tpc_chassis_imu", 10
        );
        
        pub_d2_sensors_ = this->create_publisher<msgs_ifaces::msg::ChassisSensors>(
            "tpc_chassis_sensors", 10
        );
        
        pub_d2_gnss_ = this->create_publisher<msgs_ifaces::msg::SpresenseGNSS>(
            "tpc_gnss_spresense", 10
        );
        
        pub_d2_chassis_cmd_ = this->create_publisher<msgs_ifaces::msg::ChassisCtrl>(
            "tpc_chassis_cmd", 10
        );
        
        pub_d2_mission_active_ = this->create_publisher<std_msgs::msg::Bool>(
            "tpc_gnss_mission_active", 10
        );
        
        pub_d2_distance_ = this->create_publisher<std_msgs::msg::Float64>(
            "tpc_gnss_mission_remain_dist", 10
        );
        
        RCLCPP_INFO(this->get_logger(), "Relay publishers initialized (6 topics)");
    }

    /**
     * Initialize Domain 2 subscribers (commands from base station)
     * These subscribers receive commands from ground station
     */
    void init_domain2_subscribers() {
        RCLCPP_INFO(this->get_logger(), "Initializing Domain 2 subscribers...");
        
        // Destination coordinate commands from base station
        sub_d2_dest_coord_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "tpc_rover_dest_coordinate", 10,
            std::bind(&BaseBridgeNode::on_d2_dest_coord, this, std::placeholders::_1)
        );
        
        RCLCPP_INFO(this->get_logger(), "Domain 2 subscribers initialized (1 topic)");
    }

    /**
     * Initialize Domain 5 publishers (relay commands to rover)
     * These publishers send base station commands to rover nodes
     */
    void init_domain5_publishers() {
        RCLCPP_INFO(this->get_logger(), "Initializing Domain 5 publishers...");
        
        // Relay destination coordinates to rover
        pub_d5_dest_coord_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "tpc_rover_dest_coordinate", 10
        );
        
        RCLCPP_INFO(this->get_logger(), "Domain 5 publishers initialized (1 topic)");
    }

    // ===================== Domain 5 -> Domain 2 Relay Callbacks =====================
    
    void on_d5_imu(const msgs_ifaces::msg::ChassisIMU::SharedPtr msg) {
        pub_d2_imu_->publish(*msg);
    }

    void on_d5_sensors(const msgs_ifaces::msg::ChassisSensors::SharedPtr msg) {
        pub_d2_sensors_->publish(*msg);
    }

    void on_d5_gnss(const msgs_ifaces::msg::SpresenseGNSS::SharedPtr msg) {
        pub_d2_gnss_->publish(*msg);
    }

    void on_d5_chassis_cmd(const msgs_ifaces::msg::ChassisCtrl::SharedPtr msg) {
        pub_d2_chassis_cmd_->publish(*msg);
    }

    // ===================== Domain 2 -> Domain 5 Relay Callbacks =====================
    
    void on_d2_dest_coord(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        pub_d5_dest_coord_->publish(*msg);
        RCLCPP_INFO(this->get_logger(), "Relayed destination coordinate from base to rover");
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<BaseBridgeNode>();
    
    RCLCPP_INFO(node->get_logger(), "Base Bridge Node spinning...");
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}

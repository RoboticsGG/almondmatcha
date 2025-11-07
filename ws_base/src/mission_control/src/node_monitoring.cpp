/*
 * Mission Control Monitoring Node
 * 
 * Purpose:
 *   Real-time monitoring of rover telemetry and mission progress.
 *   Subscribes to rover sensor data and displays comprehensive status updates.
 * 
 * Key Responsibilities:
 *   - Monitor GNSS position (latitude, longitude)
 *   - Track mission progress (remaining distance)
 *   - Display rover control state (direction, speed)
 *   - Provide periodic status summaries (1 second intervals)
 * 
 * Topics Subscribed:
 *   /tpc_gnss_mission_active (Bool): Mission active flag
 *   /tpc_gnss_mission_remain_dist (Float64): Remaining distance to target (km)
 *   /tpc_gnss_spresense (SpresenseGNSS): Current GPS position
 *   /tpc_rover_dest_coordinate (Float64MultiArray): Target coordinates [lat, long]
 *   /tpc_chassis_cmd (ChassisCtrl): Rover control commands (steering, speed, direction)
 * 
 * Author: Mission Control System
 * Date: November 4, 2025
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <msgs_ifaces/msg/spresense_gnss.hpp>
#include <msgs_ifaces/msg/chassis_ctrl.hpp>
#include <iomanip>
#include <sstream>

class MissionMonitoringNode : public rclcpp::Node {
public:
    MissionMonitoringNode() : Node("mission_monitoring_node") {
        RCLCPP_INFO(this->get_logger(), "Initializing Mission Control Monitoring Node...");
        
        init_subscriptions();
        init_timer();
        
        RCLCPP_INFO(this->get_logger(), "Mission Control Monitoring Node ready. Status updates every 1 second.");
    }

private:
    // ===================== Member Variables =====================
    
    // Mission State
    bool mission_active_ = false;
    float distance_remaining_ = 0.0f;
    
    // Target Coordinates
    double target_latitude_ = 0.0;
    double target_longitude_ = 0.0;
    
    // Current Position (from GNSS)
    double current_latitude_ = 0.0;
    double current_longitude_ = 0.0;
    
    // Rover Control State
    uint8_t steering_direction_ = 0;      // 1=Left, 3=Right, else=Straight
    float steering_angle_ = 0.0f;
    uint8_t speed_command_ = 0;           // 0-100%
    uint8_t movement_direction_ = 0;      // 1=Forward, 2=Backward, else=Stop
    
    // ROS2 Subscriptions
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_mission_active_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_distance_remaining_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_target_coords_;
    rclcpp::Subscription<msgs_ifaces::msg::SpresenseGNSS>::SharedPtr sub_current_position_;
    rclcpp::Subscription<msgs_ifaces::msg::ChassisCtrl>::SharedPtr sub_rover_control_;
    
    // Timer for periodic status updates
    rclcpp::TimerBase::SharedPtr status_timer_;

    // ===================== Initialization Methods =====================
    
    /**
     * Initialize all topic subscriptions
     * Connects to rover telemetry topics with queue depth of 10
     */
    void init_subscriptions() {
        RCLCPP_DEBUG(this->get_logger(), "Creating topic subscriptions...");
        
        // Mission status subscription
        sub_mission_active_ = this->create_subscription<std_msgs::msg::Bool>(
            "tpc_gnss_mission_active", 10,
            std::bind(&MissionMonitoringNode::on_mission_active, this, std::placeholders::_1)
        );
        
        // Remaining distance subscription
        sub_distance_remaining_ = this->create_subscription<std_msgs::msg::Float64>(
            "tpc_gnss_mission_remain_dist", 10,
            std::bind(&MissionMonitoringNode::on_distance_remaining, this, std::placeholders::_1)
        );
        
        // Target coordinates subscription
        sub_target_coords_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "tpc_rover_dest_coordinate", 10,
            std::bind(&MissionMonitoringNode::on_target_coordinates, this, std::placeholders::_1)
        );
        
        // Current GNSS position subscription
        sub_current_position_ = this->create_subscription<msgs_ifaces::msg::SpresenseGNSS>(
            "tpc_gnss_spresense", 10,
            std::bind(&MissionMonitoringNode::on_current_position, this, std::placeholders::_1)
        );
        
        // Rover control state subscription (via base bridge)
        sub_rover_control_ = this->create_subscription<msgs_ifaces::msg::ChassisCtrl>(
            "tpc_chassis_cmd", 10,
            std::bind(&MissionMonitoringNode::on_rover_control, this, std::placeholders::_1)
        );
        
        RCLCPP_DEBUG(this->get_logger(), "All subscriptions created successfully.");
    }

    /**
     * Initialize timer for periodic status updates (1 second interval)
     */
    void init_timer() {
        status_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&MissionMonitoringNode::publish_status_update, this)
        );
    }

    // ===================== Subscription Callbacks =====================
    
    /**
     * Callback: Mission active status
     * Updates mission state flag
     */
    void on_mission_active(const std_msgs::msg::Bool::SharedPtr msg) {
        mission_active_ = msg->data;
        RCLCPP_DEBUG(this->get_logger(),
            "Mission status updated: %s", mission_active_ ? "ACTIVE" : "INACTIVE");
    }

    /**
     * Callback: Remaining distance to target
     * Updates mission progress tracking
     */
    void on_distance_remaining(const std_msgs::msg::Float64::SharedPtr msg) {
        distance_remaining_ = static_cast<float>(msg->data);
    }

    /**
     * Callback: Target coordinates
     * Updates navigation goal
     */
    void on_target_coordinates(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 2) {
            target_latitude_ = msg->data[0];
            target_longitude_ = msg->data[1];
        }
    }

    /**
     * Callback: Current GNSS position
     * Updates rover's current location
     */
    void on_current_position(const msgs_ifaces::msg::SpresenseGNSS::SharedPtr msg) {
        current_latitude_ = msg->latitude;
        current_longitude_ = msg->longitude;
    }

    /**
     * Callback: Rover control commands
     * Parses steering and movement commands to display user-friendly status
     */
    void on_rover_control(const msgs_ifaces::msg::ChassisCtrl::SharedPtr msg) {
        steering_direction_ = msg->fdr_msg;
        steering_angle_ = msg->ro_ctrl_msg;
        speed_command_ = msg->spd_msg;
        movement_direction_ = msg->bdr_msg;
    }

    // ===================== Status Display Methods =====================
    
    /**
     * Format steering command for display
     * Returns: Descriptive string of steering command
     */
    std::string format_steering_command() {
        std::ostringstream oss;
        
        if (steering_direction_ == 1) {
            oss << "Turn Left: " << static_cast<int>(steering_angle_) << " degrees";
        } else if (steering_direction_ == 3) {
            oss << "Turn Right: " << static_cast<int>(steering_angle_) << " degrees";
        } else {
            oss << "Maintain Course";
        }
        
        return oss.str();
    }

    /**
     * Format movement command for display
     * Returns: Descriptive string of movement (speed + direction)
     */
    std::string format_movement_command() {
        std::ostringstream oss;
        
        // Check if mission is active for mission-stop logic
        if (movement_direction_ == 1 && !mission_active_) {
            oss << "Forward at " << static_cast<int>(speed_command_) << "%";
        } else if (movement_direction_ == 2 && !mission_active_) {
            oss << "Backward at " << static_cast<int>(speed_command_) << "%";
        } else {
            oss << "Stop";
        }
        
        return oss.str();
    }

    /**
     * Format coordinates with precision
     * Returns: Formatted coordinate string
     */
    std::string format_coordinate(double latitude, double longitude) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(6) 
            << "Lat: " << latitude << ", Long: " << longitude;
        return oss.str();
    }

    /**
     * Publish periodic status update (called every 1 second)
     * Displays comprehensive rover and mission status
     */
    void publish_status_update() {
        RCLCPP_INFO(this->get_logger(), "============ MISSION STATUS ============");
        RCLCPP_INFO(this->get_logger(), "Status: %s", 
            mission_active_ ? "ACTIVE" : "INACTIVE");
        RCLCPP_INFO(this->get_logger(), " ");
        
        RCLCPP_INFO(this->get_logger(), "--- Navigation ---");
        RCLCPP_INFO(this->get_logger(), "Target: %s", 
            format_coordinate(target_latitude_, target_longitude_).c_str());
        RCLCPP_INFO(this->get_logger(), "Current: %s",
            format_coordinate(current_latitude_, current_longitude_).c_str());
        RCLCPP_INFO(this->get_logger(), "Distance Remaining: %.2f km",
            distance_remaining_);
        RCLCPP_INFO(this->get_logger(), " ");
        
        RCLCPP_INFO(this->get_logger(), "--- Rover Control ---");
        RCLCPP_INFO(this->get_logger(), "Steering: %s",
            format_steering_command().c_str());
        RCLCPP_INFO(this->get_logger(), "Movement: %s",
            format_movement_command().c_str());
        RCLCPP_INFO(this->get_logger(), "========================================");
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MissionMonitoringNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
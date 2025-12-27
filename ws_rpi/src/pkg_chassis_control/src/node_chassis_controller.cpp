// ROS2 Core
#include "rclcpp/rclcpp.hpp"

// Standard Messages
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int16.hpp>

// Custom Interfaces
#include <msgs_ifaces/msg/chassis_ctrl.hpp>
#include <services_ifaces/srv/spd_limit.hpp>

// Standard Library
#include <mutex>
#include <string>
#include <thread>

/**
 * @brief Rover Chassis Low-Level Controller Node
 * 
 * This node manages low-level chassis control in Domain ID 5 (rover internal):
 * - Receives control commands from vision navigation
 * - Provides speed limit service
 * - Publishes chassis control messages to STM32 boards
 */
class ChassisController : public rclcpp::Node {
public:
    ChassisController() : Node("chassis_controller") {
        RCLCPP_INFO(this->get_logger(), "Initializing Chassis Controller (Domain 5 - Rover Internal)");
        
        // Create speed limit service
        srv_spd_limit_ = this->create_service<services_ifaces::srv::SpdLimit>(
            "srv_spd_limit",
            std::bind(&ChassisController::handleSpeedLimitRequest, 
                     this, std::placeholders::_1, std::placeholders::_2)
        );
        
        // Create mission active subscription (from ws_rpi node)
        rclcpp::QoS qos_reliable(10);
        qos_reliable.reliable().transient_local();
        
        sub_cc_rcon_ = this->create_subscription<std_msgs::msg::Bool>(
            "tpc_gnss_mission_active", qos_reliable,
            std::bind(&ChassisController::cruiseControlCallback, 
                     this, std::placeholders::_1)
        );
        
        // Create flight mode control subscription (from Jetson node)
        // Jetson Python publisher uses BEST_EFFORT QoS - match it exactly
        rclcpp::QoS qos_jetson(10);
        qos_jetson.best_effort();  // Match Jetson's BEST_EFFORT policy
        
        sub_fmctl_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "tpc_rover_fmctl", qos_jetson,
            std::bind(&ChassisController::flightModeControlCallback, 
                     this, std::placeholders::_1)
        );
        
        // Create publisher to STM32 chassis (Domain 5)
        pub_chassis_cmd_ = this->create_publisher<msgs_ifaces::msg::ChassisCtrl>(
            "tpc_chassis_cmd", qos_reliable
        );
        
        // Status timer to show node is alive
        status_timer_ = this->create_wall_timer(
            std::chrono::seconds(10),
            [this]() {
                RCLCPP_INFO(this->get_logger(), 
                    "Chassis Controller alive - waiting for vision commands on tpc_rover_fmctl");
            }
        );
        
        RCLCPP_INFO(this->get_logger(), "Chassis Controller initialized");
    }

    ~ChassisController() = default;

private:
    // === Services ===
    rclcpp::Service<services_ifaces::srv::SpdLimit>::SharedPtr srv_spd_limit_;
    
    // === Subscribers ===
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_cc_rcon_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_fmctl_;
    
    // === Publishers ===
    rclcpp::Publisher<msgs_ifaces::msg::ChassisCtrl>::SharedPtr pub_chassis_cmd_;
    
    // === Timers ===
    rclcpp::TimerBase::SharedPtr status_timer_;
    
    // === State Variables ===
    float steer_msg_;                    // Steering command (-1.0 to 1.0)
    float detect_msg_;                   // Detection status
    uint8_t spd_msg_;                    // Speed limit
    bool cc_rcon_msg_ = true;            // Cruise control/remote control flag
    
    // === Timer Variables ===
    bool detect_zero_active_ = false;    // Detection zero state tracking
    rclcpp::Time detect_zero_start_time_;
    
    // === Thread Safety ===
    std::mutex data_lock_;
    
    // === Callback Methods ===
    
    /**
     * @brief Callback for cruise control/remote control flag
     * @param msg Boolean message indicating CC/RC state
     */
    void cruiseControlCallback(const std::shared_ptr<std_msgs::msg::Bool> msg) {
        std::lock_guard<std::mutex> lock(data_lock_);
        if (msg->data != cc_rcon_msg_) { 
            cc_rcon_msg_ = msg->data;
            RCLCPP_INFO(this->get_logger(), 
                       "Cruise control state changed: %s", 
                       msg->data ? "ACTIVE (OVERRIDE)" : "INACTIVE (NORMAL)");
        }   
    }
    
    /**
     * @brief Service handler for speed limit requests
     * @param request Speed limit request containing rover_spd
     * @param response Response message confirming speed limit change
     */
    void handleSpeedLimitRequest(
        const std::shared_ptr<services_ifaces::srv::SpdLimit::Request> request,
        std::shared_ptr<services_ifaces::srv::SpdLimit::Response> response) {
        std::lock_guard<std::mutex> lock(data_lock_);
        spd_msg_ = request->rover_spd;
        response->spd_result = "Speed Limit set to " + std::to_string(request->rover_spd);
        RCLCPP_INFO(this->get_logger(), "Speed limit updated to: %d", request->rover_spd);
    }
    
    /**
     * @brief Callback for flight mode control messages
     * @param msg Float32MultiArray containing [steering, detection]
     */
    void flightModeControlCallback(const std::shared_ptr<std_msgs::msg::Float32MultiArray> msg) {
        if (msg->data.size() >= 2) {
            std::lock_guard<std::mutex> lock(data_lock_);
            steer_msg_ = msg->data[0];   // Steering command
            detect_msg_ = msg->data[1];  // Detection status
        } else {
            RCLCPP_WARN(this->get_logger(), 
                       "Insufficient data received on tpc_rover_fmctl (expected 2, got %zu)", 
                       msg->data.size());
            return;
        }
        
        // Process and publish control commands
        processAndPublishControl();
    }
    
    // === Control Logic Methods ===
    
    /**
     * @brief Process control inputs and publish rover control messages
     * 
     * This method implements the main control logic:
     * - Handles cruise control override
     * - Processes steering commands
     * - Manages detection-based speed control
     * - Publishes directly to STM32 chassis (Domain 5)
     */
    void processAndPublishControl() {
        auto chassis_ctrl = msgs_ifaces::msg::ChassisCtrl();
        
        {
            std::lock_guard<std::mutex> lock(data_lock_);

            if (cc_rcon_msg_) {
                // Cruise control is ACTIVE - override all controls (emergency stop)
                chassis_ctrl.fdr_msg = 2;        // Front direction: Stop
                chassis_ctrl.ro_ctrl_msg = 0.0;  // Steering: Zero
                chassis_ctrl.spd_msg = 0;        // Speed: Zero
                chassis_ctrl.bdr_msg = 0;        // Back direction: Stop
            } else {
                // Normal operation - process steering and detection
                
                // Process steering commands
                processSteeringCommand(chassis_ctrl);
                
                // Process detection and speed control
                processDetectionAndSpeed(chassis_ctrl);
                
                // Set forward direction when active
                chassis_ctrl.bdr_msg = 1;  // Move forward
            }

            // Publish directly to STM32 chassis on Domain 5
            pub_chassis_cmd_->publish(chassis_ctrl);
        }

        // Log published control values
        RCLCPP_INFO(this->get_logger(), 
                   "Control: [Dir:%d, Steer:%.2f, Speed:%d, Back:%d]", 
                   chassis_ctrl.fdr_msg, 
                   chassis_ctrl.ro_ctrl_msg, 
                   chassis_ctrl.spd_msg, 
                   chassis_ctrl.bdr_msg);
    }
    
    /**
     * @brief Process steering command and set direction
     * @param chassis_ctrl Output message to populate with steering data
     */
    void processSteeringCommand(msgs_ifaces::msg::ChassisCtrl& chassis_ctrl) {
        if (steer_msg_ > 0.0) {
            chassis_ctrl.fdr_msg = 1;                        // Turn right
            chassis_ctrl.ro_ctrl_msg = std::fabs(steer_msg_);
        } else if (steer_msg_ < 0.0) {
            chassis_ctrl.fdr_msg = 3;                        // Turn left
            chassis_ctrl.ro_ctrl_msg = std::fabs(steer_msg_);
        } else {
            chassis_ctrl.fdr_msg = 2;                        // Straight
            chassis_ctrl.ro_ctrl_msg = 0.0;
        }
    }
    
    /**
     * @brief Process detection signal and adjust speed accordingly
     * @param chassis_ctrl Output message to populate with speed data
     * 
     * Speed Logic:
     * - If detection active: Use full speed limit
     * - If no detection for <10s: Use half speed (caution mode)
     * - If no detection for >=10s: Stop completely (safety mode)
     */
    void processDetectionAndSpeed(msgs_ifaces::msg::ChassisCtrl& chassis_ctrl) {
        const double DETECTION_TIMEOUT_SEC = 10.0;
        
        if (detect_msg_ == 0.0) {
            // No detection signal
            if (!detect_zero_active_) {
                // First time detecting zero - start timer
                detect_zero_start_time_ = this->now();
                detect_zero_active_ = true;
            }

            auto elapsed = (this->now() - detect_zero_start_time_).seconds();

            if (elapsed >= DETECTION_TIMEOUT_SEC) {
                // No detection for too long - stop for safety
                chassis_ctrl.spd_msg = 0;
            } else {
                // Caution mode - reduce speed to half
                chassis_ctrl.spd_msg = spd_msg_ / 2;
            }
        } else {
            // Detection signal present - normal operation
            detect_zero_active_ = false;
            chassis_ctrl.spd_msg = spd_msg_;
        }
    }

};

/**
 * @brief Main entry point for the chassis controller node
 */
int main(int argc, char* argv[]) {
    // Initialize rclcpp only if not already initialized (for launch file compatibility)
    if (!rclcpp::ok()) {
        rclcpp::init(argc, argv);
    }
    
    try {
        auto node = std::make_shared<ChassisController>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("chassis_controller"), "Exception in node: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}

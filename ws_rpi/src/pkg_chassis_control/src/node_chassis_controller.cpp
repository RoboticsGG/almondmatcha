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
 * This node manages low-level chassis control, bridging communication between two ROS2 domains:
 * - Domain ID 2: Receives control commands and provides services
 * - Domain ID 5: Publishes chassis control messages
 */
class ChassisController : public rclcpp::Node {
public:
    ChassisController() : Node("chassis_controller") {
        // Initialize Domain ID 2 (Subscriber Domain)
        initializeSubscriberDomain();
        
        // Initialize Domain ID 5 (Publisher Domain)
        initializePublisherDomain();
        
        // Start executor thread for multi-domain communication
        executor_.add_node(sub_node_);
        executor_.add_node(pub_node_);
        executor_thread_ = std::thread([this]() { executor_.spin(); });
        
        RCLCPP_INFO(this->get_logger(), 
                    "Chassis Controller initialized (Subscriber: Domain 2, Publisher: Domain 5)");
    }

    ~ChassisController() {
        executor_.cancel();
        if (executor_thread_.joinable()) {
            executor_thread_.join();
        }
    }

private:
    // === Domain Nodes ===
    rclcpp::Node::SharedPtr sub_node_;  // Domain ID 2
    rclcpp::Node::SharedPtr pub_node_;  // Domain ID 5
    
    // === Services ===
    rclcpp::Service<services_ifaces::srv::SpdLimit>::SharedPtr srv_spd_limit_;
    
    // === Subscribers ===
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_cc_rcon_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_fmctl_;
    
    // === Publishers ===
    rclcpp::Publisher<msgs_ifaces::msg::ChassisCtrl>::SharedPtr pub_rocon_d5_;
    rclcpp::Publisher<msgs_ifaces::msg::ChassisCtrl>::SharedPtr pub_rocon_d2_;
    
    // === Executor ===
    rclcpp::executors::MultiThreadedExecutor executor_;
    std::thread executor_thread_;
    
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
    
    // === Initialization Methods ===
    
    /**
     * @brief Initialize subscriber domain (Domain ID 2)
     * Creates node, service, and subscriptions in Domain ID 2
     */
    void initializeSubscriberDomain() {
        rclcpp::InitOptions init_options;
        init_options.set_domain_id(2);
        
        auto context = std::make_shared<rclcpp::Context>();
        context->init(0, nullptr, init_options);
        
        rclcpp::NodeOptions node_options;
        node_options.context(context);
        sub_node_ = std::make_shared<rclcpp::Node>("sub_node", node_options);
        
        // Create speed limit service
        srv_spd_limit_ = sub_node_->create_service<services_ifaces::srv::SpdLimit>(
            "srv_spd_limit",
            std::bind(&ChassisController::handleSpeedLimitRequest, 
                     this, std::placeholders::_1, std::placeholders::_2)
        );
        
        // Create mission active subscription
        sub_cc_rcon_ = sub_node_->create_subscription<std_msgs::msg::Bool>(
            "tpc_gnss_mission_active", 10,
            std::bind(&ChassisController::cruiseControlCallback, 
                     this, std::placeholders::_1)
        );
        
        // Create flight mode control subscription
        sub_fmctl_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "tpc_rover_fmctl", 10,
            std::bind(&ChassisController::flightModeControlCallback, 
                     this, std::placeholders::_1)
        );
        
        // Create publisher in Domain ID 2
        pub_rocon_d2_ = sub_node_->create_publisher<msgs_ifaces::msg::ChassisCtrl>(
            "tpc_chassis_ctrl_d2", 10
        );
    }
    
    /**
     * @brief Initialize publisher domain (Domain ID 5)
     * Creates node and publishers in Domain ID 5
     */
    void initializePublisherDomain() {
        rclcpp::InitOptions init_options;
        init_options.set_domain_id(5);
        
        auto context = std::make_shared<rclcpp::Context>();
        context->init(0, nullptr, init_options);
        
        rclcpp::NodeOptions node_options;
        node_options.context(context);
        pub_node_ = std::make_shared<rclcpp::Node>("pub_node", node_options);
        
        // Create publisher in Domain ID 5
        pub_rocon_d5_ = pub_node_->create_publisher<msgs_ifaces::msg::ChassisCtrl>(
            "tpc_chassis_ctrl_d5", 10
        );
    }
    
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
     * - Publishes to both domains
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

            // Publish to both domains
            pub_rocon_d5_->publish(chassis_ctrl);
            pub_rocon_d2_->publish(chassis_ctrl);
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

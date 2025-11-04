/*
 * Mission Control Command Node
 * 
 * Purpose:
 *   Manages rover mission commands including destination navigation and speed limits.
 *   Communicates with the rover via ROS2 actions and services.
 * 
 * Key Responsibilities:
 *   - Load mission parameters (destination coordinates, speed limit)
 *   - Send navigation goals to the rover
 *   - Monitor goal progress via feedback
 *   - Handle mission cancellation on shutdown
 * 
 * Topics/Actions/Services:
 *   Action Client: /des_data (DesData action)
 *   Service Client: /spd_limit (SpdLimit service)
 *   Parameters: rover_spd, des_lat, des_long
 * 
 * Author: Mission Control System
 * Date: November 4, 2025
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <action_ifaces/action/des_data.hpp>
#include <services_ifaces/srv/spd_limit.hpp>
#include <chrono>
#include <memory>
#include <cmath>

class MissionCommandNode : public rclcpp::Node {
public:
    // Type aliases for cleaner code
    using DesDataAction = action_ifaces::action::DesData;
    using DesDataGoalHandle = rclcpp_action::ClientGoalHandle<DesDataAction>;
    using SpdLimitService = services_ifaces::srv::SpdLimit;

    MissionCommandNode() : Node("mission_command_node") {
        RCLCPP_INFO(this->get_logger(), "Initializing Mission Control Command Node...");
        
        init_parameters();
        init_clients();
        send_commands();
        
        RCLCPP_INFO(this->get_logger(), "Mission Control Command Node ready.");
    }

    /**
     * Cancel the active rover mission goal
     * Called during shutdown or on user request
     */
    void cancel_mission() {
        RCLCPP_WARN(this->get_logger(), "Cancelling active mission...");

        if (!goal_handle_) {
            RCLCPP_WARN(this->get_logger(), "No active goal to cancel.");
            return;
        }

        auto cancel_future = des_action_client_->async_cancel_goal(goal_handle_);
        
        if (cancel_future.valid()) {
            RCLCPP_INFO(this->get_logger(), "Mission cancellation sent to rover.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to send mission cancellation.");
        }
    }

private:
    // ===================== Member Variables =====================
    
    // ROS2 Clients
    rclcpp::Client<SpdLimitService>::SharedPtr spd_limit_client_;
    rclcpp_action::Client<DesDataAction>::SharedPtr des_action_client_;
    
    // Mission Parameters
    int rover_speed_percent_;
    double destination_latitude_;
    double destination_longitude_;
    
    // Mission State
    DesDataGoalHandle::SharedPtr goal_handle_;

    // ===================== Initialization Methods =====================
    
    /**
     * Load mission parameters from ROS2 parameter server
     * Parameters:
     *   rover_spd: Speed limit as percentage (0-100)
     *   des_lat: Target latitude coordinate
     *   des_long: Target longitude coordinate
     */
    void init_parameters() {
        RCLCPP_INFO(this->get_logger(), "Loading mission parameters...");
        
        this->declare_parameter("rover_spd", 50);
        this->declare_parameter("des_lat", 0.0);
        this->declare_parameter("des_long", 0.0);

        rover_speed_percent_ = this->get_parameter("rover_spd").as_int();
        destination_latitude_ = this->get_parameter("des_lat").as_double();
        destination_longitude_ = this->get_parameter("des_long").as_double();

        RCLCPP_INFO(this->get_logger(), 
            "Mission Parameters Loaded: speed=%d%%, target=(%f, %f)",
            rover_speed_percent_, destination_latitude_, destination_longitude_);
    }

    /**
     * Initialize ROS2 action and service clients
     */
    void init_clients() {
        RCLCPP_INFO(this->get_logger(), "Initializing ROS2 clients...");
        
        // Initialize speed limit service client
        spd_limit_client_ = this->create_client<SpdLimitService>("spd_limit");
        RCLCPP_DEBUG(this->get_logger(), "Speed limit service client created.");
        
        // Initialize destination action client
        des_action_client_ = rclcpp_action::create_client<DesDataAction>(
            this, "des_data"
        );
        RCLCPP_DEBUG(this->get_logger(), "Destination action client created.");
    }

    // ===================== Command Execution Methods =====================
    
    /**
     * Send mission commands to rover
     * Sequence:
     *   1. Send speed limit command via service
     *   2. Send navigation goal via action
     */
    void send_commands() {
        RCLCPP_INFO(this->get_logger(), "Sending mission commands to rover...");
        
        send_speed_limit();
        send_destination_goal();
    }

    /**
     * Send speed limit command to rover via service
     * Blocks until service is available (max 2 seconds)
     */
    void send_speed_limit() {
        RCLCPP_INFO(this->get_logger(), "Sending speed limit: %d%%", rover_speed_percent_);
        
        if (!spd_limit_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_WARN(this->get_logger(), 
                "Speed limit service not available. Continuing anyway...");
            return;
        }

        auto request = std::make_shared<SpdLimitService::Request>();
        request->rover_spd = rover_speed_percent_;
        
        spd_limit_client_->async_send_request(request,
            [this](rclcpp::Client<SpdLimitService>::SharedFuture future) {
                try {
                    auto response = future.get();
                    RCLCPP_INFO(this->get_logger(), "Speed limit command acknowledged by rover.");
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "Speed limit service call failed: %s", e.what());
                }
            }
        );
    }

    /**
     * Send navigation goal to rover via action
     * Includes callbacks for:
     *   - Goal acceptance/rejection
     *   - Progress feedback (remaining distance)
     *   - Final result (success/failure)
     */
    void send_destination_goal() {
        RCLCPP_INFO(this->get_logger(), "Sending destination goal: (%f, %f)",
            destination_latitude_, destination_longitude_);
        
        if (!des_action_client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "Destination action server not available.");
            return;
        }

        auto goal = DesDataAction::Goal();
        goal.des_lat = destination_latitude_;
        goal.des_long = destination_longitude_;

        auto send_goal_options = rclcpp_action::Client<DesDataAction>::SendGoalOptions();
        
        // Callback: Called when rover accepts/rejects the goal
        send_goal_options.goal_response_callback =
            [this](DesDataGoalHandle::SharedPtr handle) {
                if (!handle) {
                    RCLCPP_ERROR(this->get_logger(), 
                        "Navigation goal rejected by rover.");
                } else {
                    RCLCPP_INFO(this->get_logger(), 
                        "Navigation goal accepted by rover.");
                    goal_handle_ = handle;
                }
            };
        
        // Callback: Called periodically with progress feedback
        send_goal_options.feedback_callback =
            [this](DesDataGoalHandle::SharedPtr,
                   const std::shared_ptr<const DesDataAction::Feedback> feedback) {
                RCLCPP_INFO(this->get_logger(), 
                    "Distance Remaining: %.2f km", feedback->dis_remain);
            };
        
        // Callback: Called when goal is complete (success or failure)
        send_goal_options.result_callback =
            [this](const DesDataGoalHandle::WrappedResult &result) {
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(this->get_logger(), 
                            "Mission Complete: %s", 
                            result.result->result_fser.c_str());
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_WARN(this->get_logger(), 
                            "Mission aborted by rover.");
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_WARN(this->get_logger(), 
                            "Mission cancelled.");
                        break;
                    default:
                        RCLCPP_ERROR(this->get_logger(), 
                            "Mission failed with unknown result code.");
                }
            };

        des_action_client_->async_send_goal(goal, send_goal_options);
    }
};

/**
 * Main entry point
 * Sets up shutdown handler to gracefully cancel mission on exit
 */
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MissionCommandNode>();

    // Register shutdown handler
    rclcpp::on_shutdown([node]() {
        RCLCPP_INFO(node->get_logger(), "Shutdown signal received. Cancelling rover mission...");
        node->cancel_mission();
        rclcpp::sleep_for(std::chrono::seconds(1));
        RCLCPP_INFO(node->get_logger(), "Shutdown complete.");
    });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <action_ifaces/action/des_data.hpp>
#include <services_ifaces/srv/spd_limit.hpp>
#include <cstdlib>
#include <ctime>
#include <memory>
#include <future>

class NodeCommand : public rclcpp::Node {
public:
    using DesData = action_ifaces::action::DesData; 
    using GoalHandleDesData = rclcpp_action::ClientGoalHandle<DesData>;

    NodeCommand() : Node("node_commands") {

        this->declare_parameter("rover_spd", 0);
        this->declare_parameter("des_lat", 0.0);
        this->declare_parameter("des_long", 0.0);

        rover_spd_ = this->get_parameter("rover_spd").as_int();
        des_lat_ = this->get_parameter("des_lat").as_double();
        des_long_ = this->get_parameter("des_long").as_double();

        RCLCPP_INFO(this->get_logger(), "Loaded Parameters: rover_spd=%.2f, des_lat=%.6f, des_long=%.6f", 
                    rover_spd_, des_lat_, des_long_);

        spd_client_ = this->create_client<services_ifaces::srv::SpdLimit>("spd_limit");
        des_client_ = rclcpp_action::create_client<DesData>(this, "des_data");

        //RCLCPP_INFO(this->get_logger(), "Command Node is running...");
        send_service_requests();
    }

    void cancel_goal() {
        RCLCPP_INFO(this->get_logger(), "cancel_goal() called.");

        if (!goal_handle_) {
            RCLCPP_WARN(this->get_logger(), "No active goal to cancel.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Cancelling goal and stopping the rover...");
        auto cancel_future = des_client_->async_cancel_goal(goal_handle_);

        if (cancel_future.valid()) {
            RCLCPP_INFO(this->get_logger(), "Cancel request sent.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Cancel request failed.");
        }
    }

private:
    rclcpp::Client<services_ifaces::srv::SpdLimit>::SharedPtr spd_client_;
    rclcpp_action::Client<DesData>::SharedPtr des_client_;
    GoalHandleDesData::SharedPtr goal_handle_;
    float rover_spd_;
    float des_lat_;
    float des_long_;

    void send_service_requests() {
        auto speed_request = std::make_shared<services_ifaces::srv::SpdLimit::Request>();
        speed_request->rover_spd = rover_spd_;

        if (spd_client_->wait_for_service(std::chrono::seconds(2))) {
            spd_client_->async_send_request(speed_request);
        }

        if (!des_client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_WARN(this->get_logger(), "Destination Action server is not available.");
            return;
        }

        auto goal_msg = DesData::Goal();
        goal_msg.des_lat = des_lat_;
        goal_msg.des_long = des_long_;

        RCLCPP_INFO(this->get_logger(), "Sending destination goal...");

        auto send_goal_options = rclcpp_action::Client<DesData>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            [this](GoalHandleDesData::SharedPtr goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "Destination Action goal was rejected.");
                } else {
                    RCLCPP_INFO(this->get_logger(), "Destination Action goal accepted.");
                    goal_handle_ = goal_handle;
                }
            };
        send_goal_options.feedback_callback =
            [this](GoalHandleDesData::SharedPtr, const std::shared_ptr<const DesData::Feedback> feedback) {
                RCLCPP_INFO(this->get_logger(), "Distance Remaining: %.2f km", feedback->dis_remain);
            };
        send_goal_options.result_callback =
            [this](const GoalHandleDesData::WrappedResult &result) {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_INFO(this->get_logger(), "Destination Reached: %s", result.result->result_fser.c_str());
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Destination Action failed.");
                }
            };

        des_client_->async_send_goal(goal_msg, send_goal_options);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NodeCommand>();

    rclcpp::on_shutdown([node]() {
        RCLCPP_WARN(node->get_logger(), "Shutdown requested. Stopping the rover...");

        node->cancel_goal();

        rclcpp::sleep_for(std::chrono::seconds(2)); 

        RCLCPP_WARN(node->get_logger(), "Shutdown complete.");
    });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
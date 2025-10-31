#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <msgs_ifaces/msg/spresense_gnss.hpp>
#include <action_ifaces/action/des_data.hpp>
#include <cmath>

class GNSSMissionMonitor : public rclcpp::Node {
public:
    using DesData = action_ifaces::action::DesData;
    using GoalHandleDesData = rclcpp_action::ServerGoalHandle<DesData>;

    GNSSMissionMonitor() : Node("gnss_mission_monitor"), des_lat_(0.0), des_long_(0.0) {
        action_server_ = rclcpp_action::create_server<DesData>(
            this, "des_data",
            std::bind(&GNSSMissionMonitor::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&GNSSMissionMonitor::handle_cancel, this, std::placeholders::_1),
            std::bind(&GNSSMissionMonitor::handle_accepted, this, std::placeholders::_1)
        );

        sub_current_position_ = this->create_subscription<msgs_ifaces::msg::SpresenseGNSS>(
            "tpc_gnss_spresense", 10,
            std::bind(&GNSSMissionMonitor::topic_cur_callback, this, std::placeholders::_1)
        );

        pub_mission_active_ = this->create_publisher<std_msgs::msg::Bool>("tpc_gnss_mission_active", 10);

        pub_distance_remaining_ = this->create_publisher<std_msgs::msg::Float64>("tpc_gnss_mission_remain_dist", 10);

        pub_destination_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("tpc_rover_dest_coordinate", 10);

        RCLCPP_INFO(this->get_logger(), "GNSS Mission Monitor Action Server Initialized.");
        RCLCPP_INFO(this->get_logger(), "Waiting for goals...");
    }

private:
    rclcpp_action::Server<DesData>::SharedPtr action_server_;
    rclcpp::Subscription<msgs_ifaces::msg::SpresenseGNSS>::SharedPtr sub_current_position_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_mission_active_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_distance_remaining_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_destination_;
    
    msgs_ifaces::msg::SpresenseGNSS current_position_;

    float des_lat_;
    float des_long_;
    bool goal_reached_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const DesData::Goal> goal) {
        //RCLCPP_INFO(this->get_logger(), "Received new goal: Lat=%.6f, Lon=%.6f", goal->des_lat, goal->des_long);
        des_lat_ = goal->des_lat;
        des_long_ = goal->des_long;

        goal_reached_ = false;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleDesData>) {
        RCLCPP_WARN(this->get_logger(), "Goal Cancelled!");
        des_lat_ = 0.0; 
        des_long_ = 0.0;

        std_msgs::msg::Bool mission_active_msg;
        mission_active_msg.data = true;
        pub_mission_active_->publish(mission_active_msg);
        RCLCPP_INFO(this->get_logger(), "mission_active published: true");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleDesData> goal_handle) {
        std::thread{std::bind(&GNSSMissionMonitor::execute, this, goal_handle)}.detach();
    }

    void topic_cur_callback(const msgs_ifaces::msg::SpresenseGNSS::SharedPtr msg) {
        auto destination_msg = std_msgs::msg::Float64MultiArray();
        destination_msg.data = {des_lat_, des_long_};
        pub_destination_->publish(destination_msg);
        
        current_position_.date = msg->date;
        current_position_.time = msg->time;
        current_position_.num_satellites = msg->num_satellites;
        current_position_.fix = msg->fix;
        current_position_.latitude = msg->latitude;
        current_position_.longitude = msg->longitude;
    }

    double haversine_distance(double lat1, double lon1, double lat2, double lon2) {
        double R = 6371.0; // Earth radius in km
        double phi1 = lat1 * M_PI / 180; // φ, λ in radians
        double phi2 = lat2 * M_PI / 180;
        double delta_phi = (lat2 - lat1) * M_PI / 180;
        double delta_lambda = (lon2 - lon1) * M_PI / 180;

        double a = sin(delta_phi/2) * sin(delta_phi/2) +
                cos(phi1) * cos(phi2) * sin(delta_lambda/2) * sin(delta_lambda/2);
        double c = 2 * atan2(sqrt(a), sqrt(1-a));

        return R * c; // in km
    }

    void execute(const std::shared_ptr<GoalHandleDesData> goal_handle) {
        auto feedback = std::make_shared<DesData::Feedback>();
        std_msgs::msg::Bool mission_active_msg;

        while (rclcpp::ok()) {
            if (current_position_.latitude == 0.0 && current_position_.longitude == 0.0) {
                mission_active_msg.data = true;
                continue;
            } else if (des_lat_ == 0.0 && des_long_ == 0.0) {
                RCLCPP_WARN(this->get_logger(), "Waiting for Destination Data...");
                mission_active_msg.data = true;
                continue;
            }
            else{

                    double distance = haversine_distance(current_position_.latitude, current_position_.longitude, des_lat_, des_long_);
                    feedback->dis_remain = distance;

                    std_msgs::msg::Float64 distance_remaining_msg;
                    distance_remaining_msg.data = distance;
                    pub_distance_remaining_->publish(distance_remaining_msg);

                    goal_handle->publish_feedback(feedback);
                    RCLCPP_INFO(this->get_logger(), "Distance Remaining: %.2f km", feedback->dis_remain);

                    if (distance < 0.02) {
                        mission_active_msg.data = true;

                        if (!goal_reached_){
                            auto result = std::make_shared<DesData::Result>();
                            result->result_fser = "Arrived at Destination";
                            goal_handle->succeed(result);
                            RCLCPP_INFO(this->get_logger(), "Destination Reached!");
                            goal_reached_ = true;
                        }
                    } else {
                        mission_active_msg.data = false;
                    }
            }
            pub_mission_active_->publish(mission_active_msg);
            RCLCPP_INFO(this->get_logger(), "mission_active published: %s", mission_active_msg.data ? "true" : "false");
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GNSSMissionMonitor>());
    rclcpp::shutdown();
    return 0;
}
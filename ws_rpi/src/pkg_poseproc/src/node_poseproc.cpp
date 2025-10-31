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

class PoseProcessor : public rclcpp::Node {
public:
    using DesData = action_ifaces::action::DesData;
    using GoalHandleDesData = rclcpp_action::ServerGoalHandle<DesData>;

    PoseProcessor() : Node("pose_processor"), des_lat_(0.0), des_long_(0.0) {
        action_server_ = rclcpp_action::create_server<DesData>(
            this, "des_data",
            std::bind(&PoseProcessor::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&PoseProcessor::handle_cancel, this, std::placeholders::_1),
            std::bind(&PoseProcessor::handle_accepted, this, std::placeholders::_1)
        );

        cur_pose_sub_ = this->create_subscription<msgs_ifaces::msg::SpresenseGNSS>(
            "gnss_data", 10,
            std::bind(&PoseProcessor::topic_cur_callback, this, std::placeholders::_1)
        );

        cc_rcon_pub_ = this->create_publisher<std_msgs::msg::Bool>("cc_rcon", 10);

        dis_remain_ = this->create_publisher<std_msgs::msg::Float64>("dis_remain", 10);

        topic_despose_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("pub_despose", 10);

        RCLCPP_INFO(this->get_logger(), "PoseProcessor Action Server Initialized.");
        RCLCPP_INFO(this->get_logger(), "Waiting for goals...");
    }

private:
    rclcpp_action::Server<DesData>::SharedPtr action_server_;
    rclcpp::Subscription<msgs_ifaces::msg::SpresenseGNSS>::SharedPtr cur_pose_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr cc_rcon_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dis_remain_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr topic_despose_pub;
    
    msgs_ifaces::msg::SpresenseGNSS cur_pose_msg_;

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

        std_msgs::msg::Bool cc_rcon_msg;
        cc_rcon_msg.data = true;
        cc_rcon_pub_->publish(cc_rcon_msg);
        RCLCPP_INFO(this->get_logger(), "cc_rcon published: true");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleDesData> goal_handle) {
        std::thread{std::bind(&PoseProcessor::execute, this, goal_handle)}.detach();
    }

    void topic_cur_callback(const msgs_ifaces::msg::SpresenseGNSS::SharedPtr msg) {
        auto despose_msgs = std_msgs::msg::Float64MultiArray();
        despose_msgs.data = {des_lat_, des_long_};
        topic_despose_pub->publish(despose_msgs);
        
        cur_pose_msg_.date = msg->date;
        cur_pose_msg_.time = msg->time;
        cur_pose_msg_.num_satellites = msg->num_satellites;
        cur_pose_msg_.fix = msg->fix;
        cur_pose_msg_.latitude = msg->latitude;
        cur_pose_msg_.longitude = msg->longitude;

        //RCLCPP_INFO(this->get_logger(), "Received GNSS Data: Date=%s, Time=%s, Sat=%d, Fix=%d, Lat=%f, Lon=%f",
                    //cur_pose_msg_.date.c_str(), cur_pose_msg_.time.c_str(), cur_pose_msg_.num_satellites, cur_pose_msg_.fix, cur_pose_msg_.latitude, cur_pose_msg_.longitude);

        
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
        std_msgs::msg::Bool cc_rcon_msg;

        while (rclcpp::ok()) {
            if (cur_pose_msg_.latitude == 0.0 && cur_pose_msg_.longitude == 0.0) {
                // RCLCPP_WARN(this->get_logger(), "Waiting for GNSS Data..."); // rattachai
                cc_rcon_msg.data = true;
                //std::this_thread::sleep_for(std::chrono::seconds(2));
                continue;
            } else if (des_lat_ == 0.0 && des_long_ == 0.0) {
                RCLCPP_WARN(this->get_logger(), "Waiting for Destination Data...");
                cc_rcon_msg.data = true;
                //std::this_thread::sleep_for(std::chrono::seconds(2));
                continue;
            }
            else{

                    double distance = haversine_distance(cur_pose_msg_.latitude, cur_pose_msg_.longitude, des_lat_, des_long_);
                    feedback->dis_remain = distance;

                    std_msgs::msg::Float64 disremain_msg_;
                    disremain_msg_.data = distance;
                    dis_remain_->publish(disremain_msg_);

                    goal_handle->publish_feedback(feedback);
                    RCLCPP_INFO(this->get_logger(), "Distance Remaining: %.2f km", feedback->dis_remain);

                    if (distance < 0.02) {
                        cc_rcon_msg.data = true;

                        if (!goal_reached_){
                            auto result = std::make_shared<DesData::Result>();
                            result->result_fser = "Arrived at Destination";
                            goal_handle->succeed(result);
                            RCLCPP_INFO(this->get_logger(), "Destination Reached!");
                            goal_reached_ = true;
                        }
                    } else {
                        cc_rcon_msg.data = false;
                    }
                    // cc_rcon_pub_->publish(cc_rcon_msg);
                    // RCLCPP_INFO(this->get_logger(), "cc_rcon published: %s", cc_rcon_msg.data ? "true" : "false");
                    //std::this_thread::sleep_for(std::chrono::seconds(2));
            }
            cc_rcon_pub_->publish(cc_rcon_msg);
            RCLCPP_INFO(this->get_logger(), "cc_rcon published: %s", cc_rcon_msg.data ? "true" : "false");
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseProcessor>());
    rclcpp::shutdown();
    return 0;
}
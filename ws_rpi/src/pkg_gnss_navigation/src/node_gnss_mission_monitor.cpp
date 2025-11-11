#include <rclcpp/rclcpp.hpp> // Core ROS2 C++ API
#include "rclcpp_action/rclcpp_action.hpp" // ROS2 Action API
#include <std_msgs/msg/string.hpp> // Standard ROS2 message types
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <msgs_ifaces/msg/spresense_gnss.hpp> // Custom GNSS message type
#include <action_ifaces/action/des_data.hpp>
#include <cmath>

// GNSSMissionMonitor: Monitors GNSS position and manages navigation goals via ROS2 actions.
// Publishes mission status, remaining distance, and destination coordinates.
class GNSSMissionMonitor : public rclcpp::Node {
public:
    using DesData = action_ifaces::action::DesData;
    using GoalHandleDesData = rclcpp_action::ServerGoalHandle<DesData>;

    // Constructor: Initializes node, QoS, action server, subscriptions, and publishers.
    GNSSMissionMonitor() : Node("gnss_mission_monitor"), des_lat_(0.0), des_long_(0.0) {
        // Reliable and transient_local QoS for all topics to ensure delivery and history.
        rclcpp::QoS qos_reliable(10);
        qos_reliable.reliable().transient_local();
        
        // Action server for navigation goals (destination latitude/longitude)
        action_server_ = rclcpp_action::create_server<DesData>(
            this, "des_data",
            std::bind(&GNSSMissionMonitor::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&GNSSMissionMonitor::handle_cancel, this, std::placeholders::_1),
            std::bind(&GNSSMissionMonitor::handle_accepted, this, std::placeholders::_1)
        );

        // Subscription to GNSS position updates
        sub_current_position_ = this->create_subscription<msgs_ifaces::msg::SpresenseGNSS>(
            "tpc_gnss_spresense", qos_reliable,
            std::bind(&GNSSMissionMonitor::topic_cur_callback, this, std::placeholders::_1)
        );

        // Publishers for mission status, remaining distance, and destination coordinates
        pub_mission_active_ = this->create_publisher<std_msgs::msg::Bool>("tpc_gnss_mission_active", qos_reliable);
        pub_distance_remaining_ = this->create_publisher<std_msgs::msg::Float64>("tpc_gnss_mission_remain_dist", qos_reliable);
        pub_destination_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("tpc_rover_dest_coordinate", qos_reliable);

        RCLCPP_INFO(this->get_logger(), "GNSS Mission Monitor Action Server Initialized.");
        RCLCPP_INFO(this->get_logger(), "Waiting for goals...");
    }

private:
    // ROS2 communication components
    rclcpp_action::Server<DesData>::SharedPtr action_server_;
    rclcpp::Subscription<msgs_ifaces::msg::SpresenseGNSS>::SharedPtr sub_current_position_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_mission_active_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_distance_remaining_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_destination_;
    
    // Current GNSS position data
    msgs_ifaces::msg::SpresenseGNSS current_position_;

    // Mission parameters
    float des_lat_;
    float des_long_;
    bool goal_reached_;

    // Constants
    static constexpr double DESTINATION_THRESHOLD_KM = 0.02;  // 20 meters arrival threshold
    static constexpr int EXECUTION_LOOP_RATE_SEC = 2;         // Check mission status every 2 seconds
    static constexpr int POSITION_WARNING_THROTTLE_MS = 5000; // Throttle position warning to every 5 seconds

    // Called when a new navigation goal is received
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const DesData::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received new goal: Lat=%.6f, Lon=%.6f", goal->des_lat, goal->des_long);
        des_lat_ = goal->des_lat;
        des_long_ = goal->des_long;

        goal_reached_ = false;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // Called when a navigation goal is cancelled
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleDesData>) {
        RCLCPP_WARN(this->get_logger(), "Goal Cancelled!");
        des_lat_ = 0.0; 
        des_long_ = 0.0;
        goal_reached_ = false;

        publish_mission_status(true);
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // Called when a goal is accepted; starts execution in a new thread
    void handle_accepted(const std::shared_ptr<GoalHandleDesData> goal_handle) {
        std::thread{std::bind(&GNSSMissionMonitor::execute, this, goal_handle)}.detach();
    }

    // Callback for GNSS position updates; publishes destination coordinates
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

    // Calculates great-circle distance between two coordinates (in km)
    double haversine_distance(double lat1, double lon1, double lat2, double lon2) const {
        const double EARTH_RADIUS_KM = 6371.0;
        
        double phi1 = lat1 * M_PI / 180.0;
        double phi2 = lat2 * M_PI / 180.0;
        double delta_phi = (lat2 - lat1) * M_PI / 180.0;
        double delta_lambda = (lon2 - lon1) * M_PI / 180.0;

        double a = sin(delta_phi / 2.0) * sin(delta_phi / 2.0) +
                   cos(phi1) * cos(phi2) * sin(delta_lambda / 2.0) * sin(delta_lambda / 2.0);
        double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

        return EARTH_RADIUS_KM * c;
    }

    // Helper: Check if position data is valid
    bool is_position_valid() const {
        return !(current_position_.latitude == 0.0 && current_position_.longitude == 0.0);
    }

    // Helper: Check if destination is valid
    bool is_destination_valid() const {
        return !(des_lat_ == 0.0 && des_long_ == 0.0);
    }

    // Helper: Publish mission status
    void publish_mission_status(bool is_active) {
        std_msgs::msg::Bool mission_active_msg;
        mission_active_msg.data = is_active;
        pub_mission_active_->publish(mission_active_msg);
    }

    // Helper: Publish remaining distance
    void publish_remaining_distance(double distance_km) {
        std_msgs::msg::Float64 distance_msg;
        distance_msg.data = distance_km;
        pub_distance_remaining_->publish(distance_msg);
    }

    // Main mission execution loop: monitors position, computes distance, and manages goal state
    void execute(const std::shared_ptr<GoalHandleDesData> goal_handle) {
        auto feedback = std::make_shared<DesData::Feedback>();

        RCLCPP_INFO(this->get_logger(), "Executing mission to destination: Lat=%.6f, Lon=%.6f", des_lat_, des_long_);

        while (rclcpp::ok()) {
            // Check if position data is available
            if (!is_position_valid()) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), POSITION_WARNING_THROTTLE_MS,
                    "Waiting for GNSS position data...");
                publish_mission_status(true);
                std::this_thread::sleep_for(std::chrono::seconds(EXECUTION_LOOP_RATE_SEC));
                continue;
            }

            // Check if destination is set
            if (!is_destination_valid()) {
                RCLCPP_WARN(this->get_logger(), "Waiting for Destination Data...");
                publish_mission_status(true);
                std::this_thread::sleep_for(std::chrono::seconds(EXECUTION_LOOP_RATE_SEC));
                continue;
            }

            // Calculate distance to destination
            double distance_km = haversine_distance(
                current_position_.latitude, current_position_.longitude,
                des_lat_, des_long_
            );
            feedback->dis_remain = distance_km;

            // Publish distance and feedback
            publish_remaining_distance(distance_km);
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Distance Remaining: %.2f km", distance_km);

            // Check if destination reached
            if (distance_km < DESTINATION_THRESHOLD_KM) {
                publish_mission_status(true);

                if (!goal_reached_) {
                    auto result = std::make_shared<DesData::Result>();
                    result->result_fser = "Arrived at Destination";
                    goal_handle->succeed(result);
                    RCLCPP_INFO(this->get_logger(), "Destination Reached!");
                    goal_reached_ = true;
                }
            } else {
                publish_mission_status(false);
            }

            std::this_thread::sleep_for(std::chrono::seconds(EXECUTION_LOOP_RATE_SEC));
        }
    }
};

// Main entry point: initializes ROS2 and spins the GNSSMissionMonitor node
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GNSSMissionMonitor>());
    rclcpp::shutdown();
    return 0;
}
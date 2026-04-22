/**
 * mission_monitoring_node_rpi.cpp
 * Workspace:  ws_rpi  |  Package: pkg_rover_monitoring  |  Domain: 5 (sub) → 4 (pub)
 *
 * Purpose:
 *   Dual-role monitoring node running on the Raspberry Pi.
 *   1. Cross-domain relay: aggregates Domain 5 telemetry and re-publishes
 *      a unified TelemetryRelay message on Domain 4 at 5 Hz.
 *   2. CSV logging: writes per-topic CSV files to local storage at native rates.
 *
 * Subscribed Topics (Domain 5):
 *   /tpc_gnss_spresense, /tpc_gnss_ublox, /tpc_chassis_cmd,
 *   /tpc_chassis_imu, /tpc_chassis_sensors, /tpc_rover_ctrl_cmd
 *
 * Published Topics:
 *   /tpc_telemetry_relay (msgs_ifaces/TelemetryRelay) - Domain 4, 5 Hz
 *
 * Author: AlmondMatcha Rover Team
 * Date:   February 27, 2026
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "msgs_ifaces/msg/spresense_gnss.hpp"
#include "msgs_ifaces/msg/ublox_gnss.hpp"
#include "msgs_ifaces/msg/chassis_ctrl.hpp"
#include "msgs_ifaces/msg/chassis_sensors.hpp"
#include "msgs_ifaces/msg/chassis_imu.hpp"
#include "msgs_ifaces/msg/telemetry_relay.hpp"
#include <fstream>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <filesystem>
#include <glob.h>

/**
 * Mission Monitoring Node (RPi)
 * 
 * Multi-Domain Architecture:
 * - Subscribes to all Domain 5 telemetry topics (STM32, Jetson, Spresense)
 * - Publishes aggregated telemetry to Domain 4 via /tpc_telemetry_relay
 * - Logs all telemetry data to CSV files (high-rate, per-topic event-driven)
 * 
 * Dual Functionality:
 * 1. Cross-domain relay: D5 → D4 (telemetry_relay at 5 Hz)
 * 2. CSV data logging: D5 → RPi local storage (full-rate per topic)
 * 
 * Rationale: Consolidates monitoring and logging, reduces D5 participant count
 */
class MissionMonitoringNodeRpi : public rclcpp::Node {
public:
    // Public access to Domain 4 node for executor
    rclcpp::Node::SharedPtr domain4_node_;

    MissionMonitoringNodeRpi(rclcpp::Context::SharedPtr domain5_context, 
                              rclcpp::Context::SharedPtr domain4_context) 
        : Node("mission_monitoring_node_rpi", rclcpp::NodeOptions().context(domain5_context)) {
        // QoS profiles to match publishers
        rclcpp::QoS qos_stm32(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        qos_stm32.best_effort();
        
        rclcpp::QoS qos_reliable(10);
        qos_reliable.reliable();

        // ===== Subscribe to all Domain 5 topics =====
        
        // Chassis sensors (from STM32)
        sub_chassis_sensors_ = this->create_subscription<msgs_ifaces::msg::ChassisSensors>(
            "tpc_chassis_sensors", qos_stm32,
            std::bind(&MissionMonitoringNodeRpi::chassis_sensors_callback, this, std::placeholders::_1)
        );
        
        // Chassis IMU (from STM32)
        sub_chassis_imu_ = this->create_subscription<msgs_ifaces::msg::ChassisIMU>(
            "tpc_chassis_imu", qos_stm32,
            std::bind(&MissionMonitoringNodeRpi::chassis_imu_callback, this, std::placeholders::_1)
        );
        
        // Chassis commands (from RPi chassis controller)
        sub_chassis_cmd_ = this->create_subscription<msgs_ifaces::msg::ChassisCtrl>(
            "tpc_chassis_cmd", qos_reliable,
            std::bind(&MissionMonitoringNodeRpi::chassis_cmd_callback, this, std::placeholders::_1)
        );
        
        // Mission control
        sub_mission_active_ = this->create_subscription<std_msgs::msg::Bool>(
            "tpc_gnss_mission_active", qos_reliable,
            std::bind(&MissionMonitoringNodeRpi::mission_active_callback, this, std::placeholders::_1)
        );
        
        sub_distance_remaining_ = this->create_subscription<std_msgs::msg::Float64>(
            "tpc_gnss_mission_remain_dist", qos_reliable,
            std::bind(&MissionMonitoringNodeRpi::distance_remaining_callback, this, std::placeholders::_1)
        );
        
        // GNSS data
        sub_spresense_gnss_ = this->create_subscription<msgs_ifaces::msg::SpresenseGNSS>(
            "tpc_gnss_spresense", qos_reliable,
            std::bind(&MissionMonitoringNodeRpi::spresense_gnss_callback, this, std::placeholders::_1)
        );
        
        sub_ublox_gnss_ = this->create_subscription<msgs_ifaces::msg::UbloxGNSS>(
            "tpc_gnss_ublox", qos_reliable,
            std::bind(&MissionMonitoringNodeRpi::ublox_gnss_callback, this, std::placeholders::_1)
        );
        
        // Destination coordinate
        sub_destination_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "tpc_rover_dest_coordinate", qos_reliable,
            std::bind(&MissionMonitoringNodeRpi::destination_callback, this, std::placeholders::_1)
        );
        
        // Kinematic control command (from Jetson → tpc_rover_ctrl_cmd)
        // Format: Float32MultiArray [steer_angle, speed_cmd, detected]
        // BEST_EFFORT to match rover_kinematic_control_node.py publisher QoS
        sub_steering_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "tpc_rover_ctrl_cmd", qos_stm32,
            std::bind(&MissionMonitoringNodeRpi::steering_callback, this, std::placeholders::_1)
        );
        
        // Lane detection (from Jetson - D6->D5 bridge)
        sub_lane_detection_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "tpc_rover_nav_lane", qos_reliable,
            std::bind(&MissionMonitoringNodeRpi::lane_detection_callback, this, std::placeholders::_1)
        );

        // ===== Create Domain 4 publisher node for cross-domain relay =====
        
        // Create a separate node in Domain 4 for publishing telemetry
        domain4_node_ = std::make_shared<rclcpp::Node>(
            "mission_monitoring_domain4_pub",
            rclcpp::NodeOptions().context(domain4_context)
        );
        
        // Publisher for aggregated telemetry relay (on Domain 4)
        pub_telemetry_relay_ = domain4_node_->create_publisher<msgs_ifaces::msg::TelemetryRelay>(
            "tpc_telemetry_relay", 10
        );

        // Timer for publishing aggregated telemetry (5 Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&MissionMonitoringNodeRpi::publish_telemetry, this)
        );

        // Initialize CSV logging
        init_csv_logging();

        RCLCPP_INFO(this->get_logger(), "=== Mission Monitoring Node (RPi) Initialized ===");
        RCLCPP_INFO(this->get_logger(), "Subscribing to Domain 5 topics (all telemetry sources)");
        RCLCPP_INFO(this->get_logger(), "Publishing to Domain 4: /tpc_telemetry_relay (5 Hz)");
        RCLCPP_INFO(this->get_logger(), "CSV logging to: %s", log_dir_.c_str());
        RCLCPP_INFO(this->get_logger(), "Architecture: Cross-domain relay (D5→D4) + CSV logging");
        RCLCPP_INFO(this->get_logger(), "=========================================================");
    }

    ~MissionMonitoringNodeRpi() {
        if (csv_rtk_gnss_.is_open()) csv_rtk_gnss_.close();
        if (csv_spresense_gnss_.is_open()) csv_spresense_gnss_.close();
        if (csv_chassis_imu_.is_open()) csv_chassis_imu_.close();
        if (csv_chassis_sensors_.is_open()) csv_chassis_sensors_.close();
        if (csv_chassis_cmd_.is_open()) csv_chassis_cmd_.close();
        if (csv_mission_state_.is_open()) csv_mission_state_.close();
    }

private:
    // ===== Data storage for all Domain 5 topics =====
    
    // Mission status
    bool mission_active_ = false;
    bool mission_active_valid_ = false;
    double distance_remaining_km_ = 0.0;
    bool distance_remaining_valid_ = false;
    
    // Spresense GNSS
    bool spresense_valid_ = false;
    float spresense_latitude_ = 0.0f;
    float spresense_longitude_ = 0.0f;
    float spresense_altitude_ = 0.0f;
    int32_t spresense_satellites_ = 0;
    
    // uBlox RTK GNSS
    bool ublox_valid_ = false;
    double ublox_latitude_ = 0.0;
    double ublox_longitude_ = 0.0;
    double ublox_altitude_ = 0.0;
    std::string ublox_fix_quality_ = "No Fix";
    float ublox_centimeter_error_ = 999.9f;
    int32_t ublox_satellites_ = 0;
    
    // Chassis command
    bool chassis_cmd_valid_ = false;
    float chassis_cmd_left_speed_ = 0.0f;
    float chassis_cmd_right_speed_ = 0.0f;
    float chassis_cmd_left_direction_ = 0.0f;
    float chassis_cmd_right_direction_ = 0.0f;
    
    // Chassis sensors
    bool chassis_sensors_valid_ = false;
    float encoder_left_ = 0.0f;
    float encoder_right_ = 0.0f;
    float voltage_ = 0.0f;
    float current_ = 0.0f;
    float power_watts_ = 0.0f;
    
    // Chassis IMU
    bool chassis_imu_valid_ = false;
    float accel_x_ = 0.0f;
    float accel_y_ = 0.0f;
    float accel_z_ = 0.0f;
    float gyro_x_ = 0.0f;
    float gyro_y_ = 0.0f;
    float gyro_z_ = 0.0f;
    
    // Steering control
    bool steering_valid_ = false;
    float steering_command_ = 0.0f;
    
    // Lane detection
    bool lane_valid_ = false;
    float lane_theta_ = 0.0f;
    float lane_b_ = 0.0f;
    bool lane_detected_ = false;
    
    // Destination
    bool destination_valid_ = false;
    float destination_latitude_ = 0.0f;
    float destination_longitude_ = 0.0f;
    
    // ROS2 objects
    rclcpp::Publisher<msgs_ifaces::msg::TelemetryRelay>::SharedPtr pub_telemetry_relay_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Subscriptions
    rclcpp::Subscription<msgs_ifaces::msg::ChassisSensors>::SharedPtr sub_chassis_sensors_;
    rclcpp::Subscription<msgs_ifaces::msg::ChassisIMU>::SharedPtr sub_chassis_imu_;
    rclcpp::Subscription<msgs_ifaces::msg::ChassisCtrl>::SharedPtr sub_chassis_cmd_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_mission_active_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_distance_remaining_;
    rclcpp::Subscription<msgs_ifaces::msg::SpresenseGNSS>::SharedPtr sub_spresense_gnss_;
    rclcpp::Subscription<msgs_ifaces::msg::UbloxGNSS>::SharedPtr sub_ublox_gnss_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_destination_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_steering_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_lane_detection_;
    
    // CSV logging
    std::ofstream csv_rtk_gnss_;
    std::ofstream csv_spresense_gnss_;
    std::ofstream csv_chassis_imu_;
    std::ofstream csv_chassis_sensors_;
    std::ofstream csv_chassis_cmd_;
    std::ofstream csv_mission_state_;
    std::string log_dir_;
    
    // ===== CSV Logging Initialization =====
    
    int get_next_run_number(const std::string& runs_dir) {
        glob_t glob_result;
        std::string pattern = runs_dir + "/run_*";
        int max_run = 0;
        
        if (glob(pattern.c_str(), GLOB_TILDE | GLOB_ONLYDIR, NULL, &glob_result) == 0) {
            for (size_t i = 0; i < glob_result.gl_pathc; ++i) {
                std::string dirname = glob_result.gl_pathv[i];
                size_t pos = dirname.find("run_");
                if (pos != std::string::npos) {
                    std::string num_str = dirname.substr(pos + 4, 3);
                    try {
                        int num = std::stoi(num_str);
                        if (num > max_run) max_run = num;
                    } catch (...) {}
                }
            }
        }
        globfree(&glob_result);
        
        return max_run + 1;
    }

    void init_csv_logging() {
        // Get ws_rpi path
        std::string ws_rpi_path = std::filesystem::current_path().string();
        
        while (!ws_rpi_path.empty() && 
               ws_rpi_path.find("ws_rpi") != std::string::npos &&
               !std::filesystem::exists(ws_rpi_path + "/src")) {
            ws_rpi_path = std::filesystem::path(ws_rpi_path).parent_path().string();
        }
        
        if (ws_rpi_path.find("ws_rpi") == std::string::npos) {
            auto path = std::filesystem::current_path();
            while (path.has_parent_path()) {
                if (path.filename() == "ws_rpi") {
                    ws_rpi_path = path.string();
                    break;
                }
                path = path.parent_path();
            }
        }
        
        // Create runs directory
        std::string runs_dir = ws_rpi_path + "/runs";
        
        if (!std::filesystem::exists(runs_dir)) {
            try {
                std::filesystem::create_directories(runs_dir);
                RCLCPP_INFO(this->get_logger(), "Created runs directory: %s", runs_dir.c_str());
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to create runs directory: %s", e.what());
                runs_dir = "/tmp";
            }
        }

        // Get run number and timestamp
        int run_number = get_next_run_number(runs_dir);
        auto now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);
        std::tm *timeinfo = std::localtime(&now_c);
        char buffer[50];
        std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", timeinfo);
        
        // Create run subdirectory: run_NNN_YYYYMMDD_HHMMSS/
        std::ostringstream run_dir_ss;
        run_dir_ss << runs_dir << "/run_" << std::setfill('0') << std::setw(3) << run_number 
                   << "_" << buffer;
        log_dir_ = run_dir_ss.str();
        
        try {
            std::filesystem::create_directories(log_dir_);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create run directory: %s", e.what());
            return;
        }

        // Open CSV files with headers
        csv_rtk_gnss_.open(log_dir_ + "/rtk_gnss.csv", std::ios::out);
        if (csv_rtk_gnss_.is_open()) {
            csv_rtk_gnss_ << "Timestamp_us,Latitude,Longitude,Altitude,"
                         << "Fix_Quality,Centimeter_Error,Satellites\n";
        }
        
        csv_spresense_gnss_.open(log_dir_ + "/spresense_gnss.csv", std::ios::out);
        if (csv_spresense_gnss_.is_open()) {
            csv_spresense_gnss_ << "Timestamp_us,Latitude,Longitude,Altitude,Satellites\n";
        }
        
        csv_chassis_imu_.open(log_dir_ + "/chassis_imu.csv", std::ios::out);
        if (csv_chassis_imu_.is_open()) {
            csv_chassis_imu_ << "Timestamp_us,Accel_X,Accel_Y,Accel_Z,Gyro_X,Gyro_Y,Gyro_Z\n";
        }
        
        csv_chassis_sensors_.open(log_dir_ + "/chassis_sensors.csv", std::ios::out);
        if (csv_chassis_sensors_.is_open()) {
            csv_chassis_sensors_ << "Timestamp_us,Encoder_Left,Encoder_Right,Voltage_V,Current_A,Power_W\n";
        }
        
        csv_chassis_cmd_.open(log_dir_ + "/chassis_cmd.csv", std::ios::out);
        if (csv_chassis_cmd_.is_open()) {
            csv_chassis_cmd_ << "Timestamp_us,Left_Speed,Right_Speed,Left_Direction,Right_Direction\n";
        }
        
        csv_mission_state_.open(log_dir_ + "/mission_state.csv", std::ios::out);
        if (csv_mission_state_.is_open()) {
            csv_mission_state_ << "Timestamp_us,Mission_Active,Distance_Remaining_km,"
                              << "Dest_Latitude,Dest_Longitude,Steering_Cmd,Lane_Theta,Lane_B,Lane_Detected\n";
        }
        
        RCLCPP_INFO(this->get_logger(), "CSV logging initialized: Run #%d", run_number);
    }
    
    // ===== Callbacks for all topics =====
    
    void chassis_sensors_callback(const msgs_ifaces::msg::ChassisSensors::SharedPtr msg) {
        chassis_sensors_valid_ = true;
        encoder_left_ = static_cast<float>(msg->mt_lf_encode_msg);
        encoder_right_ = static_cast<float>(msg->mt_rt_encode_msg);
        voltage_ = msg->sys_volt_msg;
        current_ = msg->sys_current_msg;
        power_watts_ = voltage_ * current_;
        
        // CSV logging (event-driven, ~4 Hz)
        if (csv_chassis_sensors_.is_open()) {
            auto timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()
            ).count();
            csv_chassis_sensors_ << timestamp_us << "," << encoder_left_ << "," << encoder_right_ << ","
                               << voltage_ << "," << current_ << "," << power_watts_ << "\n";
            csv_chassis_sensors_.flush();
        }
    }
    
    void chassis_imu_callback(const msgs_ifaces::msg::ChassisIMU::SharedPtr msg) {
        chassis_imu_valid_ = true;
        accel_x_ = static_cast<float>(msg->accel_x);
        accel_y_ = static_cast<float>(msg->accel_y);
        accel_z_ = static_cast<float>(msg->accel_z);
        gyro_x_ = static_cast<float>(msg->gyro_x);
        gyro_y_ = static_cast<float>(msg->gyro_y);
        gyro_z_ = static_cast<float>(msg->gyro_z);
        
        // CSV logging (event-driven, ~10 Hz)
        if (csv_chassis_imu_.is_open()) {
            auto timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()
            ).count();
            csv_chassis_imu_ << timestamp_us << "," << accel_x_ << "," << accel_y_ << "," << accel_z_ << ","
                           << gyro_x_ << "," << gyro_y_ << "," << gyro_z_ << "\n";
           csv_chassis_imu_.flush();
        }
    }
    
    void chassis_cmd_callback(const msgs_ifaces::msg::ChassisCtrl::SharedPtr msg) {
        chassis_cmd_valid_ = true;
        // Note: ChassisCtrl may need to be decoded based on your message structure
        // This is a placeholder - adjust based on actual message fields
        chassis_cmd_left_speed_ = msg->spd_msg;
        chassis_cmd_right_speed_ = msg->spd_msg;
        chassis_cmd_left_direction_ = msg->fdr_msg;
        chassis_cmd_right_direction_ = msg->bdr_msg;
        
        // CSV logging (event-driven, ~50 Hz)
        if (csv_chassis_cmd_.is_open()) {
            auto timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()
            ).count();
            csv_chassis_cmd_ << timestamp_us << "," << chassis_cmd_left_speed_ << "," << chassis_cmd_right_speed_ << ","
                           << static_cast<int>(chassis_cmd_left_direction_) << "," 
                           << static_cast<int>(chassis_cmd_right_direction_) << "\n";
            csv_chassis_cmd_.flush();
        }
    }
    
    void mission_active_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        mission_active_valid_ = true;
        mission_active_ = msg->data;
        write_mission_state_csv();
    }
    
    void distance_remaining_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        distance_remaining_valid_ = true;
        distance_remaining_km_ = msg->data;
        write_mission_state_csv();
    }
    
    void spresense_gnss_callback(const msgs_ifaces::msg::SpresenseGNSS::SharedPtr msg) {
        spresense_valid_ = true;
        spresense_latitude_ = msg->latitude;
        spresense_longitude_ = msg->longitude;
        spresense_altitude_ = msg->altitude;
        spresense_satellites_ = msg->num_satellites;
        
        // CSV logging (event-driven, ~10 Hz)
        if (csv_spresense_gnss_.is_open()) {
            auto timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()
            ).count();
            csv_spresense_gnss_ << timestamp_us << "," << spresense_latitude_ << "," << spresense_longitude_ << ","
                               << spresense_altitude_ << "," << spresense_satellites_ << "\n";
            csv_spresense_gnss_.flush();
        }
    }
    
    void ublox_gnss_callback(const msgs_ifaces::msg::UbloxGNSS::SharedPtr msg) {
        ublox_valid_ = true;
        ublox_latitude_ = msg->latitude;
        ublox_longitude_ = msg->longitude;
        ublox_altitude_ = msg->altitude;
        ublox_fix_quality_ = msg->fix_quality;
        ublox_centimeter_error_ = msg->centimeter_error;
        ublox_satellites_ = msg->satellites_tracked;
        
        // CSV logging (event-driven, ~10 Hz)
        if (csv_rtk_gnss_.is_open()) {
            auto timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()
            ).count();
            csv_rtk_gnss_ << timestamp_us << "," << ublox_latitude_ << "," << ublox_longitude_ << ","
                         << ublox_altitude_ << "," << ublox_fix_quality_ << "," 
                         << ublox_centimeter_error_ << "," << ublox_satellites_ << "\n";
            csv_rtk_gnss_.flush();
        }
    }
    
    void destination_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 2) {
            destination_valid_ = true;
            destination_latitude_ = static_cast<float>(msg->data[0]);
            destination_longitude_ = static_cast<float>(msg->data[1]);
        }
        write_mission_state_csv();
    }
    
    void steering_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        // tpc_rover_ctrl_cmd format: [steer_angle_deg, speed_cmd, detected]
        if (msg->data.size() < 3) {
            return;
        }
        steering_valid_   = true;
        steering_command_ = msg->data[0];  // Steering angle in degrees
    }
    
    void lane_detection_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 3) {
            lane_valid_ = true;
            lane_theta_ = static_cast<float>(msg->data[0]);
            lane_b_ = static_cast<float>(msg->data[1]);
            lane_detected_ = static_cast<bool>(msg->data[2]);
        }
    }
    
    void write_mission_state_csv() {
        // Write mission state whenever any mission-related topic updates
        if (csv_mission_state_.is_open()) {
            auto timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()
            ).count();
            
            csv_mission_state_ << timestamp_us << ","
                             << (mission_active_ ? "true" : "false") << ","
                             << distance_remaining_km_ << ","
                             << destination_latitude_ << "," << destination_longitude_ << ","
                             << steering_command_ << "," << lane_theta_ << "," << lane_b_ << ","
                             << (lane_detected_ ? "true" : "false") << "\n";
            csv_mission_state_.flush();
        }
    }
    
    void publish_telemetry() {
        auto telemetry = msgs_ifaces::msg::TelemetryRelay();
        
        // Set header
        telemetry.header.stamp = this->now();
        telemetry.header.frame_id = "base_link";
        
        // Mission status
        telemetry.mission_active = mission_active_;
        telemetry.distance_remaining_km = distance_remaining_km_;
        
        // Spresense GNSS
        telemetry.spresense_valid = spresense_valid_;
        telemetry.spresense_latitude = spresense_latitude_;
        telemetry.spresense_longitude = spresense_longitude_;
        telemetry.spresense_altitude = spresense_altitude_;
        telemetry.spresense_satellites = spresense_satellites_;
        
        // uBlox RTK GNSS
        telemetry.ublox_valid = ublox_valid_;
        telemetry.ublox_latitude = ublox_latitude_;
        telemetry.ublox_longitude = ublox_longitude_;
        telemetry.ublox_altitude = ublox_altitude_;
        telemetry.ublox_fix_quality = ublox_fix_quality_;
        telemetry.ublox_centimeter_error = ublox_centimeter_error_;
        telemetry.ublox_satellites = ublox_satellites_;
        
        // Chassis command
        telemetry.chassis_cmd_valid = chassis_cmd_valid_;
        telemetry.chassis_cmd_left_speed = chassis_cmd_left_speed_;
        telemetry.chassis_cmd_right_speed = chassis_cmd_right_speed_;
        telemetry.chassis_cmd_left_direction = chassis_cmd_left_direction_;
        telemetry.chassis_cmd_right_direction = chassis_cmd_right_direction_;
        
        // Chassis sensors
        telemetry.chassis_sensors_valid = chassis_sensors_valid_;
        telemetry.encoder_left = encoder_left_;
        telemetry.encoder_right = encoder_right_;
        telemetry.voltage = voltage_;
        telemetry.current = current_;
        telemetry.power_watts = power_watts_;
        
        // Chassis IMU
        telemetry.chassis_imu_valid = chassis_imu_valid_;
        telemetry.accel_x = accel_x_;
        telemetry.accel_y = accel_y_;
        telemetry.accel_z = accel_z_;
        telemetry.gyro_x = gyro_x_;
        telemetry.gyro_y = gyro_y_;
        telemetry.gyro_z = gyro_z_;
        
        // Steering control
        telemetry.steering_valid = steering_valid_;
        telemetry.steering_command = steering_command_;
        
        // Lane detection
        telemetry.lane_valid = lane_valid_;
        telemetry.lane_theta = lane_theta_;
        telemetry.lane_b = lane_b_;
        telemetry.lane_detected = lane_detected_;
        
        // Destination
        telemetry.destination_valid = destination_valid_;
        telemetry.destination_latitude = destination_latitude_;
        telemetry.destination_longitude = destination_longitude_;
        
        pub_telemetry_relay_->publish(telemetry);
    }
};

int main(int argc, char ** argv) {
    // Initialize ROS2 with default context
    rclcpp::init(argc, argv);
    
    // Create context for Domain 5 (main control domain - for subscriptions)
    auto domain5_init_options = rclcpp::InitOptions();
    domain5_init_options.set_domain_id(5);
    auto domain5_context = std::make_shared<rclcpp::Context>();
    domain5_context->init(argc, argv, domain5_init_options);
    
    // Create context for Domain 4 (base station monitoring - for publishing)
    auto domain4_init_options = rclcpp::InitOptions();
    domain4_init_options.set_domain_id(4);
    auto domain4_context = std::make_shared<rclcpp::Context>();
    domain4_context->init(argc, argv, domain4_init_options);
    
    // Create the monitoring node with dual-domain support
    auto node = std::make_shared<MissionMonitoringNodeRpi>(domain5_context, domain4_context);
    
    // Create executor to spin both nodes
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(node->domain4_node_);
    
    RCLCPP_INFO(node->get_logger(), "Starting multi-domain executor (D5 subscriptions + D4 publishing)");
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}

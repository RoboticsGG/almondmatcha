/**
 * mission_monitoring_node_pc.cpp
 * Workspace:  ws_base  |  Package: mission_control  |  Domain: 4/5
 *
 * Purpose:
 *   Telemetry display node on the base station PC.
 *   Subscribes only to the pre-aggregated TelemetryRelay from the RPi.
 *   Writes a CSV log of every received telemetry message for post-run analysis.
 *
 * Subscribed Topics:
 *   /tpc_telemetry_relay (msgs_ifaces/TelemetryRelay) - unified rover state at 5 Hz
 *
 * Parameters:
 *   csv_path (string, default ""): path for the telemetry CSV.
 *     If empty, auto-generates ~/almondmatcha_poc/telemetry_relay_YYYYMMDD_HHMMSS.csv
 *
 * Author: AlmondMatcha Rover Team
 * Date:   February 27, 2026
 */

#include "rclcpp/rclcpp.hpp"
#include "msgs_ifaces/msg/telemetry_relay.hpp"
#include <iomanip>
#include <fstream>
#include <sstream>
#include <chrono>
#include <ctime>
#include <cstdlib>
#include <sys/stat.h>

class MissionMonitoringNodePc : public rclcpp::Node {
public:
    MissionMonitoringNodePc() : Node("mission_monitoring_node_pc") {
        // CSV path parameter — override to write directly to a run directory
        this->declare_parameter("csv_path", std::string(""));
        std::string csv_path = this->get_parameter("csv_path").as_string();

        if (csv_path.empty()) {
            // Auto-generate path: ~/almondmatcha_poc/telemetry_relay_YYYYMMDD_HHMMSS.csv
            const char* home = std::getenv("HOME");
            std::string dir = std::string(home ? home : "/tmp") + "/almondmatcha_poc";
            mkdir(dir.c_str(), 0755);
            auto now = std::chrono::system_clock::now();
            auto t   = std::chrono::system_clock::to_time_t(now);
            struct tm tm_buf;
            localtime_r(&t, &tm_buf);
            char ts[32];
            strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", &tm_buf);
            csv_path = dir + "/telemetry_relay_" + ts + ".csv";
        }

        csv_file_.open(csv_path, std::ios::out | std::ios::trunc);
        if (csv_file_.is_open()) {
            // Write CSV header
            csv_file_ << "wall_time,ros_stamp_sec,ros_stamp_nsec,"
                      << "mission_active,dist_km,"
                      << "ublox_valid,ublox_lat,ublox_lon,ublox_alt_m,ublox_fix,ublox_acc_cm,ublox_sats,"
                      << "spresense_valid,spresense_lat,spresense_lon,spresense_alt_m,spresense_sats,"
                      << "chassis_cmd_valid,cmd_spd_l,cmd_spd_r,"
                      << "chassis_sensors_valid,enc_left,enc_right,voltage,current,power_w,"
                      << "chassis_imu_valid,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,"
                      << "steering_valid,steering_cmd,"
                      << "lane_valid,lane_detected,lane_theta,lane_b,"
                      << "dest_valid,dest_lat,dest_lon\n";
            csv_file_.flush();
            RCLCPP_INFO(this->get_logger(), "Telemetry CSV: %s", csv_path.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "Cannot open CSV path: %s — CSV logging disabled", csv_path.c_str());
        }

        // Subscribe to aggregated telemetry relay ONLY
        sub_telemetry_relay_ = this->create_subscription<msgs_ifaces::msg::TelemetryRelay>(
            "tpc_telemetry_relay", 10,
            std::bind(&MissionMonitoringNodePc::telemetry_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "=== Mission Monitoring Node (Base Station) Initialized ===");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: tpc_telemetry_relay (Domain 5)");
        RCLCPP_INFO(this->get_logger(), "Architecture: Relay-only (no direct topic fallback)");
        RCLCPP_INFO(this->get_logger(), "=========================================================");
    }

    ~MissionMonitoringNodePc() {
        if (csv_file_.is_open()) {
            csv_file_.flush();
            csv_file_.close();
        }
    }

private:
    std::ofstream csv_file_;

    void write_csv_row(const msgs_ifaces::msg::TelemetryRelay::SharedPtr& msg) {
        if (!csv_file_.is_open()) return;
        auto now = std::chrono::system_clock::now();
        auto wall_sec = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()).count() / 1000.0;
        csv_file_ << std::fixed << std::setprecision(3) << wall_sec << ","
                  << msg->header.stamp.sec << "," << msg->header.stamp.nanosec << ","
                  << (msg->mission_active ? 1 : 0) << ","
                  << std::setprecision(6) << msg->distance_remaining_km << ","
                  << (msg->ublox_valid ? 1 : 0) << ","
                  << std::setprecision(8) << msg->ublox_latitude << ","
                  << msg->ublox_longitude << ","
                  << std::setprecision(3) << msg->ublox_altitude << ","
                  << msg->ublox_fix_quality << ","
                  << std::setprecision(1) << msg->ublox_centimeter_error << ","
                  << msg->ublox_satellites << ","
                  << (msg->spresense_valid ? 1 : 0) << ","
                  << std::setprecision(6) << msg->spresense_latitude << ","
                  << msg->spresense_longitude << ","
                  << std::setprecision(3) << msg->spresense_altitude << ","
                  << msg->spresense_satellites << ","
                  << (msg->chassis_cmd_valid ? 1 : 0) << ","
                  << std::setprecision(2) << msg->chassis_cmd_left_speed << ","
                  << msg->chassis_cmd_right_speed << ","
                  << (msg->chassis_sensors_valid ? 1 : 0) << ","
                  << msg->encoder_left << "," << msg->encoder_right << ","
                  << msg->voltage << "," << msg->current << "," << msg->power_watts << ","
                  << (msg->chassis_imu_valid ? 1 : 0) << ","
                  << msg->accel_x << "," << msg->accel_y << "," << msg->accel_z << ","
                  << msg->gyro_x << "," << msg->gyro_y << "," << msg->gyro_z << ","
                  << (msg->steering_valid ? 1 : 0) << ","
                  << msg->steering_command << ","
                  << (msg->lane_valid ? 1 : 0) << ","
                  << (msg->lane_detected ? 1 : 0) << ","
                  << msg->lane_theta << "," << msg->lane_b << ","
                  << (msg->destination_valid ? 1 : 0) << ","
                  << std::setprecision(6) << msg->destination_latitude << ","
                  << msg->destination_longitude << "\n";
        csv_file_.flush();
    }

    void telemetry_callback(const msgs_ifaces::msg::TelemetryRelay::SharedPtr msg) {
        write_csv_row(msg);
        // Clear screen for better display (optional - comment out if not desired)
        // std::cout << "\033[2J\033[1;1H";
        
        RCLCPP_INFO(this->get_logger(), " ");
        RCLCPP_INFO(this->get_logger(), "╔══════════════════════════════════════════════════════════════╗");
        RCLCPP_INFO(this->get_logger(), "║              ROVER TELEMETRY DASHBOARD                       ║");
        RCLCPP_INFO(this->get_logger(), "╚══════════════════════════════════════════════════════════════╝");
        
        // Mission Status
        RCLCPP_INFO(this->get_logger(), " ");
        RCLCPP_INFO(this->get_logger(), "┌─ Mission Status ────────────────────────────────────────────┐");
        RCLCPP_INFO(this->get_logger(), "│ Active: %s", msg->mission_active ? "YES" : "NO");
        RCLCPP_INFO(this->get_logger(), "│ Distance Remaining: %.3f km", msg->distance_remaining_km);
        if (msg->destination_valid) {
            RCLCPP_INFO(this->get_logger(), "│ Destination: %.6f°, %.6f°", 
                msg->destination_latitude, msg->destination_longitude);
        }
        RCLCPP_INFO(this->get_logger(), "└─────────────────────────────────────────────────────────────┘");
        
        // GNSS Position
        RCLCPP_INFO(this->get_logger(), " ");
        RCLCPP_INFO(this->get_logger(), "┌─ GNSS Position ─────────────────────────────────────────────┐");
        
        if (msg->ublox_valid) {
            RCLCPP_INFO(this->get_logger(), "│ RTK (uBlox F9P):");
            RCLCPP_INFO(this->get_logger(), "│   Position: %.8f°, %.8f°", 
                msg->ublox_latitude, msg->ublox_longitude);
            RCLCPP_INFO(this->get_logger(), "│   Altitude: %.2f m", msg->ublox_altitude);
            RCLCPP_INFO(this->get_logger(), "│   Fix Quality: %s", msg->ublox_fix_quality.c_str());
            RCLCPP_INFO(this->get_logger(), "│   Accuracy: %.1f cm", msg->ublox_centimeter_error);
            RCLCPP_INFO(this->get_logger(), "│   Satellites: %d", msg->ublox_satellites);
        } else {
            RCLCPP_WARN(this->get_logger(), "│ RTK: NO DATA");
        }
        
        if (msg->spresense_valid) {
            RCLCPP_INFO(this->get_logger(), "│ Standard GPS (Spresense):");
            RCLCPP_INFO(this->get_logger(), "│   Position: %.6f°, %.6f°", 
                msg->spresense_latitude, msg->spresense_longitude);
            RCLCPP_INFO(this->get_logger(), "│   Altitude: %.2f m", msg->spresense_altitude);
            RCLCPP_INFO(this->get_logger(), "│   Satellites: %d", msg->spresense_satellites);
        }
        
        RCLCPP_INFO(this->get_logger(), "└─────────────────────────────────────────────────────────────┘");
        
        // Chassis Status
        RCLCPP_INFO(this->get_logger(), " ");
        RCLCPP_INFO(this->get_logger(), "┌─ Chassis Status ────────────────────────────────────────────┐");
        
        if (msg->chassis_sensors_valid) {
            RCLCPP_INFO(this->get_logger(), "│ Power: %.2f V, %.2f A (%.2f W)", 
                msg->voltage, msg->current, msg->power_watts);
            RCLCPP_INFO(this->get_logger(), "│ Encoders: L=%.0f, R=%.0f", 
                msg->encoder_left, msg->encoder_right);
        }
        
        if (msg->chassis_cmd_valid) {
            RCLCPP_INFO(this->get_logger(), "│ Command: L_Speed=%.1f, R_Speed=%.1f", 
                msg->chassis_cmd_left_speed, msg->chassis_cmd_right_speed);
        }
        
        if (msg->chassis_imu_valid) {
            RCLCPP_INFO(this->get_logger(), "│ IMU Accel: X=%.2f, Y=%.2f, Z=%.2f", 
                msg->accel_x, msg->accel_y, msg->accel_z);
            RCLCPP_INFO(this->get_logger(), "│ IMU Gyro:  X=%.2f, Y=%.2f, Z=%.2f", 
                msg->gyro_x, msg->gyro_y, msg->gyro_z);
        }
        
        RCLCPP_INFO(this->get_logger(), "└─────────────────────────────────────────────────────────────┘");
        
        // Vision & Navigation
        RCLCPP_INFO(this->get_logger(), " ");
        RCLCPP_INFO(this->get_logger(), "┌─ Vision & Navigation ───────────────────────────────────────┐");
        
        if (msg->steering_valid) {
            RCLCPP_INFO(this->get_logger(), "│ Steering Command: %.2f°", msg->steering_command);
        }
        
        if (msg->lane_valid) {
            RCLCPP_INFO(this->get_logger(), "│ Lane Detection: %s", 
                msg->lane_detected ? "DETECTED" : "NOT DETECTED");
            if (msg->lane_detected) {
                RCLCPP_INFO(this->get_logger(), "│   Theta: %.3f, B: %.3f", 
                    msg->lane_theta, msg->lane_b);
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "└─────────────────────────────────────────────────────────────┘");
        
        // Data freshness indicators
        RCLCPP_INFO(this->get_logger(), " ");
        RCLCPP_INFO(this->get_logger(), "Data Status: RTK=%s | GPS=%s | Sensors=%s | IMU=%s | Steering=%s | Lane=%s",
            msg->ublox_valid ? "✓" : "✗",
            msg->spresense_valid ? "✓" : "✗",
            msg->chassis_sensors_valid ? "✓" : "✗",
            msg->chassis_imu_valid ? "✓" : "✗",
            msg->steering_valid ? "✓" : "✗",
            msg->lane_valid ? "✓" : "✗"
        );
        RCLCPP_INFO(this->get_logger(), " ");
    }

    rclcpp::Subscription<msgs_ifaces::msg::TelemetryRelay>::SharedPtr sub_telemetry_relay_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MissionMonitoringNodePc>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

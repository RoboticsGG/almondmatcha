/**
 * mission_monitoring_node_pc.cpp
 * Workspace:  ws_base  |  Package: mission_control  |  Domain: 4
 *
 * Purpose:
 *   Telemetry display node on the base station PC.
 *   Subscribes only to the pre-aggregated TelemetryRelay from the RPi
 *   (Domain 4). No direct Domain 5 subscriptions — avoids STM32 memory cost.
 *
 * Subscribed Topics (Domain 4):
 *   /tpc_telemetry_relay (msgs_ifaces/TelemetryRelay) - unified rover state at 5 Hz
 *
 * Author: AlmondMatcha Rover Team
 * Date:   February 27, 2026
 */

#include "rclcpp/rclcpp.hpp"
#include "msgs_ifaces/msg/telemetry_relay.hpp"
#include <iomanip>

class MissionMonitoringNodePc : public rclcpp::Node {
public:
    MissionMonitoringNodePc() : Node("mission_monitoring_node_pc") {
        // Subscribe to aggregated telemetry relay ONLY
        sub_telemetry_relay_ = this->create_subscription<msgs_ifaces::msg::TelemetryRelay>(
            "/tpc_telemetry_relay", 10,
            std::bind(&MissionMonitoringNodePc::telemetry_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "=== Mission Monitoring Node (Base Station) Initialized ===");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: /tpc_telemetry_relay (Domain 5)");
        RCLCPP_INFO(this->get_logger(), "Architecture: Relay-only (no direct topic fallback)");
        RCLCPP_INFO(this->get_logger(), "=========================================================");
    }

private:
    void telemetry_callback(const msgs_ifaces::msg::TelemetryRelay::SharedPtr msg) {
        // Clear screen for better display (optional - comment out if not desired)
        // std::cout << "\033[2J\033[1;1H";
        
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "╔══════════════════════════════════════════════════════════════╗");
        RCLCPP_INFO(this->get_logger(), "║              ROVER TELEMETRY DASHBOARD                       ║");
        RCLCPP_INFO(this->get_logger(), "╚══════════════════════════════════════════════════════════════╝");
        
        // Mission Status
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "┌─ Mission Status ────────────────────────────────────────────┐");
        RCLCPP_INFO(this->get_logger(), "│ Active: %s", msg->mission_active ? "YES" : "NO");
        RCLCPP_INFO(this->get_logger(), "│ Distance Remaining: %.3f km", msg->distance_remaining_km);
        if (msg->destination_valid) {
            RCLCPP_INFO(this->get_logger(), "│ Destination: %.6f°, %.6f°", 
                msg->destination_latitude, msg->destination_longitude);
        }
        RCLCPP_INFO(this->get_logger(), "└─────────────────────────────────────────────────────────────┘");
        
        // GNSS Position
        RCLCPP_INFO(this->get_logger(), "");
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
        RCLCPP_INFO(this->get_logger(), "");
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
        RCLCPP_INFO(this->get_logger(), "");
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
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "Data Status: RTK=%s | GPS=%s | Sensors=%s | IMU=%s | Steering=%s | Lane=%s",
            msg->ublox_valid ? "✓" : "✗",
            msg->spresense_valid ? "✓" : "✗",
            msg->chassis_sensors_valid ? "✓" : "✗",
            msg->chassis_imu_valid ? "✓" : "✗",
            msg->steering_valid ? "✓" : "✗",
            msg->lane_valid ? "✓" : "✗"
        );
        RCLCPP_INFO(this->get_logger(), "");
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

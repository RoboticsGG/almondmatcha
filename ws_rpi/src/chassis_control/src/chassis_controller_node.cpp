/**
 * chassis_controller_node.cpp
 * Workspace:  ws_rpi  |  Package: pkg_chassis_control  |  Domain: 5
 *
 * Purpose:
 *   Safety translator between Jetson kinematic commands and STM32 motor commands.
 *   Subscribes to /tpc_rover_ctrl_cmd [steer_angle, speed_cmd, detected] from Jetson,
 *   applies a configurable speed safety cap (set via /srv_spd_limit service),
 *   translates steering angle to ChassisCtrl direction enum, and publishes
 *   /tpc_chassis_cmd to the STM32 chassis_controller node.
 *   Emergency-stop is triggered by /tpc_gnss_mission_active going false.
 *
 * Subscribed Topics:
 *   /tpc_rover_ctrl_cmd  (std_msgs/Float32MultiArray) - [steer_angle°, speed_cmd 0-100%, detected]
 *   /tpc_gnss_mission_active (std_msgs/Bool)          - mission alive flag
 *
 * Published Topics:
 *   /tpc_chassis_cmd (msgs_ifaces/ChassisCtrl) - motor command to STM32
 *
 * Services:
 *   /srv_spd_limit (services_ifaces/SpdLimit) - set speed cap ceiling (0–100% PWM)
 *
 * Author: AlmondMatcha Rover Team
 * Date:   February 27, 2026
 */

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
 * Thin command executor sitting between the Jetson kinematic controller and the STM32.
 *
 * Responsibilities:
 *   - Translate tpc_rover_ctrl_cmd [steer_angle, speed_cmd, detected] into ChassisCtrl
 *   - Apply the srv_spd_limit safety cap (hard maximum speed ceiling)
 *   - Emit a full emergency stop when tpc_gnss_mission_active is true
 *
 * All detection-timeout speed logic lives in the Jetson's rover_kinematic_control node;
 * this node trusts the speed_cmd it receives and only clamps it against the safety cap.
 */
class ChassisController : public rclcpp::Node {
public:
    ChassisController() : Node("chassis_controller_node") {
        RCLCPP_INFO(this->get_logger(), "Initializing Chassis Controller (Domain 5 - Rover Internal)");

        // --- Speed limit safety-cap service ---
        // Sets the hard maximum speed that chassis_controller will ever send to the STM32.
        // The Jetson kinematic node is responsible for computing actual target speed;
        // this cap is an independent safety override (e.g. from base station / operator).
        srv_spd_limit_ = this->create_service<services_ifaces::srv::SpdLimit>(
            "srv_spd_limit",
            std::bind(&ChassisController::handleSpeedLimitRequest,
                     this, std::placeholders::_1, std::placeholders::_2)
        );

        // --- Mission active: emergency stop flag (reliable + transient_local) ---
        rclcpp::QoS qos_reliable(10);
        qos_reliable.reliable().transient_local();

        sub_cc_rcon_ = this->create_subscription<std_msgs::msg::Bool>(
            "tpc_gnss_mission_active", qos_reliable,
            std::bind(&ChassisController::cruiseControlCallback,
                     this, std::placeholders::_1)
        );

        // --- Kinematic control command from Jetson (BEST_EFFORT, 50 Hz) ---
        // Format: Float32MultiArray [steer_angle_deg, speed_cmd (0-255), detected (0|1)]
        // Jetson Python publisher uses BEST_EFFORT QoS; match exactly.
        rclcpp::QoS qos_jetson(10);
        qos_jetson.best_effort();

        sub_ctrl_cmd_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "tpc_rover_ctrl_cmd", qos_jetson,
            std::bind(&ChassisController::ctrlCmdCallback,
                     this, std::placeholders::_1)
        );

        // --- Chassis command publisher → STM32 (Domain 5, reliable) ---
        pub_chassis_cmd_ = this->create_publisher<msgs_ifaces::msg::ChassisCtrl>(
            "tpc_chassis_cmd", qos_reliable
        );

        // Status heartbeat
        status_timer_ = this->create_wall_timer(
            std::chrono::seconds(10),
            [this]() {
                RCLCPP_INFO(this->get_logger(),
                    "Chassis Controller alive — waiting for tpc_rover_ctrl_cmd (spd_cap=%d)",
                    spd_limit_cap_);
            }
        );

        RCLCPP_INFO(this->get_logger(), "Chassis Controller initialized (spd_limit_cap default=%d)",
                   spd_limit_cap_);
    }

    ~ChassisController() = default;

private:
    // === Services ===
    rclcpp::Service<services_ifaces::srv::SpdLimit>::SharedPtr srv_spd_limit_;

    // === Subscribers ===
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_cc_rcon_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_ctrl_cmd_;

    // === Publishers ===
    rclcpp::Publisher<msgs_ifaces::msg::ChassisCtrl>::SharedPtr pub_chassis_cmd_;

    // === Timers ===
    rclcpp::TimerBase::SharedPtr status_timer_;

    // === Control State ===
    float    steer_angle_cmd_ = 0.0f;   // Steering angle in degrees from Jetson (-max to +max)
    float    speed_cmd_       = 0.0f;   // Chassis speed command (0–255) from Jetson kinematic node
    bool     cc_rcon_msg_     = true;   // Emergency stop flag: true = halted (safe default), false = mission active (motion allowed)

    // === Safety Cap (set via srv_spd_limit service) ===
    // Unit: 0–100 (STM32 motor_control.cpp divides by 100.0 to get PWM duty cycle).
    // Values above 100 are meaningless and will exceed 100% duty on the STM32.
    //
    // Default: 50 (50% duty) — conservative safe startup speed.
    // Raise via srv_spd_limit once the operator confirms safe operating conditions.
    uint8_t  spd_limit_cap_   = 50;

    // === Thread Safety ===
    std::mutex data_lock_;
    
    // === Callback Methods ===
    
    /**
     * @brief Callback for mission-active/emergency-stop flag (tpc_gnss_mission_active).
     *
     * When true the rover must halt immediately; all motion outputs are zeroed regardless
     * of whatever the Jetson kinematic node is sending.
     */
    void cruiseControlCallback(const std::shared_ptr<std_msgs::msg::Bool> msg) {
        std::lock_guard<std::mutex> lock(data_lock_);
        // mission_active = true  → allow motion (cc_rcon_msg_ = false)
        // mission_active = false → emergency stop (cc_rcon_msg_ = true)
        bool new_halt = !msg->data;
        if (new_halt != cc_rcon_msg_) {
            cc_rcon_msg_ = new_halt;
            RCLCPP_INFO(this->get_logger(),
                       "Emergency stop: %s (tpc_gnss_mission_active=%s)",
                       new_halt ? "ACTIVE — halting motion" : "CLEARED — resuming normal control",
                       msg->data ? "true" : "false");
        }
    }
    
    /**
     * @brief Service handler for speed safety-cap requests (srv_spd_limit).
     *
     * Sets the hard maximum speed ceiling.  The Jetson kinematic node computes
     * the actual target speed; this cap is an independent operator override that
     * clamps the outgoing spd_msg to the STM32 to at most rover_spd.
     *
     * Unit: 0–100 (percentage duty cycle — the STM32 divides by 100 for PWM).
     * Values above 100 will be accepted but treated as 100% duty on the STM32 hardware.
     * Default at startup: 50 (50% duty).  Set to 0 for an operator-imposed full stop.
     */
    void handleSpeedLimitRequest(
        const std::shared_ptr<services_ifaces::srv::SpdLimit::Request> request,
        std::shared_ptr<services_ifaces::srv::SpdLimit::Response> response) {
        std::lock_guard<std::mutex> lock(data_lock_);
        spd_limit_cap_ = request->rover_spd;
        response->spd_result = "Speed safety cap set to " + std::to_string(request->rover_spd);
        RCLCPP_INFO(this->get_logger(), "Speed safety cap updated to: %d", request->rover_spd);
    }
    
    /**
     * @brief Callback for the kinematic control command from the Jetson (tpc_rover_ctrl_cmd).
     *
     * Expected format: Float32MultiArray [steer_angle_deg, speed_cmd, detected]
     *   data[0]  steer_angle_deg  Steering angle in degrees (+ = right, - = left)
     *   data[1]  speed_cmd        Chassis speed (0–255); detection-timeout logic applied on Jetson
     *   data[2]  detected         Lane visibility flag (informational, 0 or 1)
     */
    void ctrlCmdCallback(const std::shared_ptr<std_msgs::msg::Float32MultiArray> msg) {
        if (msg->data.size() < 3) {
            RCLCPP_WARN(this->get_logger(),
                       "tpc_rover_ctrl_cmd: expected 3 fields, got %zu — ignoring",
                       msg->data.size());
            return;
        }

        {
            std::lock_guard<std::mutex> lock(data_lock_);
            steer_angle_cmd_ = msg->data[0];   // Steering angle (degrees)
            speed_cmd_       = msg->data[1];   // Speed command (0–255)
            // data[2] = detected flag — already factored into speed_cmd by Jetson
        }

        packAndPublishChassisCtrl();
    }
    
    // === Control Logic Methods ===
    
    /**
     * @brief Pack state into a ChassisCtrl message and publish to the STM32.
     *
     * Two modes:
     *   Emergency stop (cc_rcon_msg_ == true): zero all fields immediately.
     *   Normal        (cc_rcon_msg_ == false): translate steer_angle_cmd_ and speed_cmd_
     *                                          into ChassisCtrl fields, applying spd_limit_cap_.
     */
    void packAndPublishChassisCtrl() {
        auto chassis_ctrl = msgs_ifaces::msg::ChassisCtrl();

        {
            std::lock_guard<std::mutex> lock(data_lock_);

            if (cc_rcon_msg_) {
                // Emergency stop — zero everything regardless of incoming commands
                chassis_ctrl.fdr_msg     = 2;    // Straight (no turn)
                chassis_ctrl.ro_ctrl_msg = 0.0f; // Steering: neutral
                chassis_ctrl.spd_msg     = 0;    // Speed: stopped
                chassis_ctrl.bdr_msg     = 0;    // Back drive: stopped
            } else {
                // Normal operation
                translateSteeringAngle(chassis_ctrl);
                applySpeedSafetyCap(chassis_ctrl);
                chassis_ctrl.bdr_msg = 1;  // Forward
            }

            pub_chassis_cmd_->publish(chassis_ctrl);
        }

        RCLCPP_INFO(this->get_logger(),
                   "ChassisCtrl → [fdr:%d  steer:%.2f  spd:%d  bdr:%d]",
                   chassis_ctrl.fdr_msg,
                   chassis_ctrl.ro_ctrl_msg,
                   chassis_ctrl.spd_msg,
                   chassis_ctrl.bdr_msg);
    }
    
    /**
     * @brief Translate steer_angle_cmd_ (degrees) into fdr_msg direction + ro_ctrl_msg magnitude.
     *
     * Sign convention (matches Jetson PID output):
     *   steer_angle_cmd_ > 0  → turn right  (fdr_msg = 1)
     *   steer_angle_cmd_ < 0  → turn left   (fdr_msg = 3)
     *   steer_angle_cmd_ == 0 → straight     (fdr_msg = 2)
     *
     * ro_ctrl_msg carries the absolute angle value forwarded to the STM32 servo driver.
     */
    void translateSteeringAngle(msgs_ifaces::msg::ChassisCtrl& chassis_ctrl) {
        if (steer_angle_cmd_ > 0.0f) {
            chassis_ctrl.fdr_msg     = 1;                              // Right
            chassis_ctrl.ro_ctrl_msg = std::fabs(steer_angle_cmd_);
        } else if (steer_angle_cmd_ < 0.0f) {
            chassis_ctrl.fdr_msg     = 3;                              // Left
            chassis_ctrl.ro_ctrl_msg = std::fabs(steer_angle_cmd_);
        } else {
            chassis_ctrl.fdr_msg     = 2;                              // Straight
            chassis_ctrl.ro_ctrl_msg = 0.0f;
        }
    }
    
    /**
     * @brief Clamp the incoming speed_cmd_ against the operator safety cap.
     *
     * Unit: 0–100 (% duty cycle). The STM32 motor_control.cpp converts to PWM as:
     *     motor_duty = speed_percent / 100.0
     * So 100 = full throttle, 50 = half throttle, 0 = stop.
     * Values above 100 are clamped here before they reach the STM32.
     *
     * The Jetson's rover_kinematic_control node is responsible for detection-based
     * speed modulation (full speed, caution half-speed, and safety stop on timeout).
     * This node simply ensures the final value never exceeds spd_limit_cap_.
     */
    void applySpeedSafetyCap(msgs_ifaces::msg::ChassisCtrl& chassis_ctrl) {
        // Clamp to [0, 100] first (valid duty-cycle range), then apply operator cap
        const uint8_t duty_clamped = static_cast<uint8_t>(
            std::min(100.0f, std::max(0.0f, speed_cmd_))
        );
        chassis_ctrl.spd_msg = std::min(duty_clamped, spd_limit_cap_);
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

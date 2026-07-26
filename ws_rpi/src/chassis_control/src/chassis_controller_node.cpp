/**
 * chassis_controller_node.cpp
 * Workspace:  ws_rpi  |  Package: pkg_chassis_control  |  Domain: 5
 *
 * Purpose:
 *   Safety translator + closed-loop speed controller between Jetson kinematic
 *   commands and STM32 motor commands.
 *   Subscribes to /tpc_rover_ctrl_cmd [steer_angle, speed_cmd, detected] from Jetson,
 *   applies a configurable speed safety cap (set via /srv_spd_limit service),
 *   translates steering angle to ChassisCtrl direction enum, and publishes
 *   /tpc_chassis_cmd to the STM32 chassis_controller node.
 *   Emergency-stop is triggered by /tpc_gnss_mission_active going false.
 *
 *   Speed is closed-loop: /tpc_chassis_sensors (wheel encoders, ~4 Hz from the
 *   sensors STM32) drives a PID that corrects PWM duty for terrain/load so the
 *   chassis holds the commanded speed instead of applying it open-loop. Steering
 *   keeps updating at the full 50 Hz of /tpc_rover_ctrl_cmd; only the speed output
 *   is paced by the slower encoder feed. Falls back to open-loop passthrough if
 *   the encoder feed goes stale (see applySpeedSafetyCap()).
 *
 * Subscribed Topics:
 *   /tpc_rover_ctrl_cmd  (std_msgs/Float32MultiArray) - [steer_angle°, speed_cmd 0-100%, detected]
 *   /tpc_gnss_mission_active (std_msgs/Bool)          - mission alive flag
 *   /tpc_chassis_sensors (msgs_ifaces/ChassisSensors)  - wheel encoder counts, ~4 Hz
 *
 * Published Topics:
 *   /tpc_chassis_cmd (msgs_ifaces/ChassisCtrl) - motor command to STM32
 *   /tpc_chassis_speed_debug (std_msgs/Float32MultiArray) - closed-loop speed
 *       PID internals, ~4 Hz (paced by encoder feed): [measured_left_tps,
 *       measured_right_tps, measured_avg_tps, target_tps, error, pid_output_pct]
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
#include <msgs_ifaces/msg/chassis_sensors.hpp>
#include <services_ifaces/srv/spd_limit.hpp>

// Standard Library
#include <algorithm>
#include <mutex>
#include <string>
#include <thread>

/**
 * @brief Rover Chassis Low-Level Controller Node
 *
 * Command executor + closed-loop speed controller sitting between the Jetson
 * kinematic controller and the STM32.
 *
 * Responsibilities:
 *   - Translate tpc_rover_ctrl_cmd [steer_angle, speed_cmd, detected] into ChassisCtrl
 *   - Apply the srv_spd_limit safety cap (hard maximum speed ceiling)
 *   - Close the speed loop against tpc_chassis_sensors encoder feedback (~4 Hz),
 *     falling back to open-loop passthrough if that feed is stale or disabled
 *   - Emit a full emergency stop when tpc_gnss_mission_active is true
 *
 * All detection-timeout speed logic lives in the Jetson's rover_kinematic_control node;
 * this node trusts the speed_cmd it receives, applies the safety cap, and (when the
 * encoder feed is healthy) corrects the outgoing PWM duty to actually hit that speed.
 */
class ChassisController : public rclcpp::Node {
public:
    ChassisController() : Node("chassis_controller_node") {
        RCLCPP_INFO(this->get_logger(), "Initializing Chassis Controller (Domain 5 - Rover Internal)");

        // --- Closed-loop speed control parameters ---
        // See config/chassis_speed_control_params.yaml for tuned values.
        this->declare_parameter<bool>("use_closed_loop_speed", true);
        this->declare_parameter<double>("speed_kp", 0.05);
        this->declare_parameter<double>("speed_ki", 0.01);
        this->declare_parameter<double>("speed_kd", 0.0);
        this->declare_parameter<double>("speed_integral_limit", 200.0);
        this->declare_parameter<double>("max_ticks_per_sec", 1000.0);
        this->declare_parameter<double>("sensor_timeout_sec", 1.0);

        use_closed_loop_speed_ = this->get_parameter("use_closed_loop_speed").as_bool();
        speed_kp_              = this->get_parameter("speed_kp").as_double();
        speed_ki_              = this->get_parameter("speed_ki").as_double();
        speed_kd_              = this->get_parameter("speed_kd").as_double();
        speed_integral_limit_  = this->get_parameter("speed_integral_limit").as_double();
        max_ticks_per_sec_     = this->get_parameter("max_ticks_per_sec").as_double();
        sensor_timeout_sec_    = this->get_parameter("sensor_timeout_sec").as_double();

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

        // --- Wheel encoder feedback from the sensors STM32 (BEST_EFFORT, ~4 Hz) ---
        // Matches the mbed publisher's sensor_data QoS profile (see chassis_sensors_node.cpp).
        rclcpp::QoS qos_sensor(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        qos_sensor.best_effort();

        sub_chassis_sensors_ = this->create_subscription<msgs_ifaces::msg::ChassisSensors>(
            "tpc_chassis_sensors", qos_sensor,
            std::bind(&ChassisController::chassisSensorsCallback,
                     this, std::placeholders::_1)
        );

        // --- Chassis command publisher → STM32 (Domain 5, reliable) ---
        pub_chassis_cmd_ = this->create_publisher<msgs_ifaces::msg::ChassisCtrl>(
            "tpc_chassis_cmd", qos_reliable
        );

        // --- Speed-loop debug publisher (Domain 5, best-effort, ~4 Hz) ---
        // Exposes the closed-loop speed PID's internal signals so they can be
        // logged and used to tune speed_kp/ki/kd — otherwise they're computed
        // and discarded inside chassisSensorsCallback() with no external trace.
        pub_speed_debug_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "tpc_chassis_speed_debug", qos_sensor
        );

        // Status heartbeat
        status_timer_ = this->create_wall_timer(
            std::chrono::seconds(10),
            [this]() {
                RCLCPP_INFO(this->get_logger(),
                    "Chassis Controller alive — waiting for tpc_rover_ctrl_cmd (spd_cap=%d, closed_loop_speed=%s)",
                    spd_limit_cap_, use_closed_loop_speed_ ? "on" : "off");
            }
        );

        RCLCPP_INFO(this->get_logger(),
                   "Chassis Controller initialized (spd_limit_cap default=%d, closed_loop_speed=%s)",
                   spd_limit_cap_, use_closed_loop_speed_ ? "on" : "off");
    }

    ~ChassisController() = default;

private:
    // === Services ===
    rclcpp::Service<services_ifaces::srv::SpdLimit>::SharedPtr srv_spd_limit_;

    // === Subscribers ===
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_cc_rcon_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_ctrl_cmd_;
    rclcpp::Subscription<msgs_ifaces::msg::ChassisSensors>::SharedPtr sub_chassis_sensors_;

    // === Publishers ===
    rclcpp::Publisher<msgs_ifaces::msg::ChassisCtrl>::SharedPtr pub_chassis_cmd_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_speed_debug_;

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

    // === Closed-loop speed control (encoder feedback from tpc_chassis_sensors) ===
    // Kill-switch — set false (ros2 param set, no rebuild) to force legacy open-loop passthrough.
    bool   use_closed_loop_speed_ = true;
    double speed_kp_              = 0.05;
    double speed_ki_              = 0.01;
    double speed_kd_              = 0.0;
    double speed_integral_limit_  = 200.0;  // Anti-windup clamp on the integral term
    double max_ticks_per_sec_     = 1000.0; // Encoder ticks/sec at 100% duty on flat ground — field-calibrate
    double sensor_timeout_sec_    = 1.0;    // Encoder feed considered stale after this long with no message

    uint8_t  target_speed_pct_       = 0;      // speed_cmd_ clamped to spd_limit_cap_ — PID setpoint & open-loop fallback value
    float    speed_pid_output_pct_   = 0.0f;   // Latest PID output (0-100%), updated at ~4 Hz by chassisSensorsCallback
    bool     speed_loop_was_active_  = false;  // Edge-detect for the closed-loop -> open-loop fallback transition

    bool     has_prev_encoder_   = false;
    int32_t  prev_encode_left_   = 0;
    int32_t  prev_encode_right_  = 0;
    rclcpp::Time last_sensor_time_{0, 0, RCL_ROS_TIME};

    double speed_integral_    = 0.0;
    double speed_last_error_  = 0.0;

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

    /**
     * @brief Callback for wheel encoder feedback (tpc_chassis_sensors, ~4 Hz).
     *
     * Drives the speed PID: computes ticks/sec from consecutive encoder deltas,
     * compares against the setpoint derived from target_speed_pct_, and stores
     * the result in speed_pid_output_pct_ for applySpeedSafetyCap() to consume
     * on the next (50 Hz) tpc_rover_ctrl_cmd cycle.
     *
     * Runs independently of the 50 Hz steering loop — steering keeps updating
     * at full rate even though speed correction only refreshes at encoder rate.
     */
    void chassisSensorsCallback(const std::shared_ptr<msgs_ifaces::msg::ChassisSensors> msg) {
        std::lock_guard<std::mutex> lock(data_lock_);
        const rclcpp::Time now = this->now();

        if (!has_prev_encoder_) {
            // First message — just seed the baseline, no valid dt yet.
            prev_encode_left_  = msg->mt_lf_encode_msg;
            prev_encode_right_ = msg->mt_rt_encode_msg;
            last_sensor_time_  = now;
            has_prev_encoder_  = true;
            return;
        }

        const double dt = (now - last_sensor_time_).seconds();
        const int32_t delta_left  = msg->mt_lf_encode_msg - prev_encode_left_;
        const int32_t delta_right = msg->mt_rt_encode_msg - prev_encode_right_;
        prev_encode_left_  = msg->mt_lf_encode_msg;
        prev_encode_right_ = msg->mt_rt_encode_msg;
        last_sensor_time_  = now;

        if (dt <= 1e-3) {
            return;  // Duplicate/out-of-order timestamp — skip this update
        }

        if (cc_rcon_msg_) {
            // Emergency stop active: motors are commanded to zero, so encoder error
            // against a stale target is meaningless. Reset instead of accumulating.
            resetSpeedPid();
            return;
        }

        const double measured_left_tps  = delta_left / dt;
        const double measured_right_tps = delta_right / dt;
        const double measured_ticks_per_sec = ((delta_left + delta_right) / 2.0) / dt;
        const double target_ticks_per_sec =
            (static_cast<double>(target_speed_pct_) / 100.0) * max_ticks_per_sec_;
        const double error = target_ticks_per_sec - measured_ticks_per_sec;

        // --- PID with anti-windup (integral clamp) ---
        speed_integral_ += error * dt;
        speed_integral_ = std::clamp(speed_integral_, -speed_integral_limit_, speed_integral_limit_);

        const double derivative = (error - speed_last_error_) / dt;
        speed_last_error_ = error;

        const double u = speed_kp_ * error + speed_ki_ * speed_integral_ + speed_kd_ * derivative;
        speed_pid_output_pct_ = static_cast<float>(std::clamp(u, 0.0, 100.0));

        // --- Publish debug signals so the PID is tunable from logs instead of opaque ---
        auto debug_msg = std_msgs::msg::Float32MultiArray();
        debug_msg.data = {
            static_cast<float>(measured_left_tps),
            static_cast<float>(measured_right_tps),
            static_cast<float>(measured_ticks_per_sec),
            static_cast<float>(target_ticks_per_sec),
            static_cast<float>(error),
            speed_pid_output_pct_
        };
        pub_speed_debug_->publish(debug_msg);
    }

    /**
     * @brief Reset PID accumulator state (integral, last error, output).
     *
     * Called whenever the closed loop stops being trusted — on falling back to
     * open-loop passthrough (stale sensor feed) or during emergency stop — so it
     * doesn't resume later with wound-up state from a period it wasn't tracking.
     */
    void resetSpeedPid() {
        speed_integral_       = 0.0;
        speed_last_error_     = 0.0;
        speed_pid_output_pct_ = 0.0f;
    }

    /**
     * @brief True if a wheel encoder message has arrived within sensor_timeout_sec_.
     */
    bool isSensorFresh() const {
        if (!has_prev_encoder_) {
            return false;
        }
        return (this->now() - last_sensor_time_).seconds() < sensor_timeout_sec_;
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
     * @brief Clamp the incoming speed_cmd_ against the operator safety cap, then
     *        apply closed-loop speed correction if the encoder feed is healthy.
     *
     * Unit: 0–100 (% duty cycle). The STM32 motor_control.cpp converts to PWM as:
     *     motor_duty = speed_percent / 100.0
     * So 100 = full throttle, 50 = half throttle, 0 = stop.
     *
     * target_speed_pct_ = min(speed_cmd_, spd_limit_cap_) is always computed first —
     * it's both the open-loop fallback value and the PID setpoint that
     * chassisSensorsCallback() targets (see that method for the ~4 Hz update).
     *
     * The Jetson's rover_kinematic_control node is responsible for detection-based
     * speed modulation (full speed, caution half-speed, and safety stop on timeout).
     * This node ensures the final value never exceeds spd_limit_cap_ and, when
     * possible, corrects the outgoing duty so actual wheel speed matches the target
     * under varying terrain load instead of applying that duty open-loop.
     */
    void applySpeedSafetyCap(msgs_ifaces::msg::ChassisCtrl& chassis_ctrl) {
        // Clamp to [0, 100] first (valid duty-cycle range), then apply operator cap
        const uint8_t duty_clamped = static_cast<uint8_t>(
            std::min(100.0f, std::max(0.0f, speed_cmd_))
        );
        target_speed_pct_ = std::min(duty_clamped, spd_limit_cap_);

        const bool closed_loop_active = use_closed_loop_speed_ && isSensorFresh();

        if (!closed_loop_active && speed_loop_was_active_) {
            // Just fell back — reset so the PID doesn't resume wound-up later.
            resetSpeedPid();
            RCLCPP_WARN(this->get_logger(),
                       "tpc_chassis_sensors feed stale or closed loop disabled — "
                       "falling back to open-loop speed passthrough");
        }
        speed_loop_was_active_ = closed_loop_active;

        const uint8_t output_pct = closed_loop_active
            ? static_cast<uint8_t>(std::clamp(speed_pid_output_pct_, 0.0f, 100.0f))
            : target_speed_pct_;

        // Safety cap always wins, even if the PID output overshoots it.
        chassis_ctrl.spd_msg = std::min(output_pct, spd_limit_cap_);
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

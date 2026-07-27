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
 *       measured_right_tps, measured_avg_tps, target_tps, error_pct, pid_output_pct]
 *       NOTE: error_pct is percent-of-full-scale (the unit the PID operates on),
 *       not ticks/sec — the first four fields remain ticks/sec.
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
#include <vector>
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
        this->declare_parameter<double>("speed_kp", 0.3);
        this->declare_parameter<double>("speed_ki", 0.5);
        this->declare_parameter<double>("speed_kd", 0.0);
        this->declare_parameter<double>("speed_integral_limit", 100.0);
        this->declare_parameter<double>("speed_max_duty_step_pct", 15.0);
        this->declare_parameter<double>("max_ticks_per_sec", 1000.0);
        this->declare_parameter<double>("sensor_timeout_sec", 1.0);

        // --- Auto-calibration of max_ticks_per_sec ---
        this->declare_parameter<bool>("auto_calibrate_max_ticks", true);
        this->declare_parameter<double>("autocal_min_duty_pct", 13.0);
        this->declare_parameter<int>("autocal_min_samples", 40);
        this->declare_parameter<double>("autocal_window_sec", 60.0);
        this->declare_parameter<bool>("auto_enable_closed_loop", true);

        // --- Stall detection ---
        this->declare_parameter<bool>("stall_detect_enabled", true);
        this->declare_parameter<double>("stall_min_duty_pct", 30.0);
        this->declare_parameter<double>("stall_min_ticks_per_sec", 10.0);
        this->declare_parameter<double>("stall_timeout_sec", 2.0);

        use_closed_loop_speed_ = this->get_parameter("use_closed_loop_speed").as_bool();
        speed_kp_              = this->get_parameter("speed_kp").as_double();
        speed_ki_              = this->get_parameter("speed_ki").as_double();
        speed_kd_              = this->get_parameter("speed_kd").as_double();
        speed_integral_limit_  = this->get_parameter("speed_integral_limit").as_double();
        speed_max_duty_step_pct_ = this->get_parameter("speed_max_duty_step_pct").as_double();
        max_ticks_per_sec_     = this->get_parameter("max_ticks_per_sec").as_double();
        sensor_timeout_sec_    = this->get_parameter("sensor_timeout_sec").as_double();

        auto_calibrate_max_ticks_ = this->get_parameter("auto_calibrate_max_ticks").as_bool();
        autocal_min_duty_pct_     = this->get_parameter("autocal_min_duty_pct").as_double();
        autocal_min_samples_      = this->get_parameter("autocal_min_samples").as_int();
        autocal_window_sec_       = this->get_parameter("autocal_window_sec").as_double();
        auto_enable_closed_loop_  = this->get_parameter("auto_enable_closed_loop").as_bool();

        stall_detect_enabled_     = this->get_parameter("stall_detect_enabled").as_bool();
        stall_min_duty_pct_       = this->get_parameter("stall_min_duty_pct").as_double();
        stall_min_ticks_per_sec_  = this->get_parameter("stall_min_ticks_per_sec").as_double();
        stall_timeout_sec_        = this->get_parameter("stall_timeout_sec").as_double();

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
    // Default: 40 (40% duty) — the operational safety ceiling for this rover.
    // The Jetson's target speed is 20%, so this leaves the closed-loop speed
    // controller 20 points of headroom to push through terrain load while still
    // never exceeding a duty the chassis is safe at. Do not raise it to the
    // target speed: the cap clamps the loop's *output*, so cap == target leaves
    // the controller no authority to correct for a ramp at all.
    uint8_t  spd_limit_cap_   = 40;

    // === Closed-loop speed control (encoder feedback from tpc_chassis_sensors) ===
    // Kill-switch — set false (ros2 param set, no rebuild) to force legacy open-loop passthrough.
    bool   use_closed_loop_speed_ = true;
    // Gains are dimensionless / per-second: the loop error is percent-of-full-scale,
    // so these stay valid when max_ticks_per_sec is re-calibrated.
    double speed_kp_              = 0.3;
    double speed_ki_              = 0.5;
    double speed_kd_              = 0.0;
    // Anti-windup clamp, in %·s. The integral's duty authority is ki * this limit
    // (0.5 * 100 = ±50% duty of trim on top of the feedforward term).
    double speed_integral_limit_  = 100.0;
    // Max change in commanded duty per encoder update (~4 Hz), in % points.
    // 15 %/update ≈ 60 %/s — fast enough not to blunt real load correction,
    // slow enough that a bad calibration surges instead of hammering 0<->100.
    double speed_max_duty_step_pct_ = 15.0;
    int    overspeed_strikes_       = 0;   // consecutive samples reading >150% of full scale

    // === Auto-calibration of max_ticks_per_sec ===
    // In open loop the commanded duty is known and the tick rate is measured, so
    // full-scale capability is directly observable: ticks_per_sec / (duty/100).
    bool   auto_calibrate_max_ticks_ = true;
    double autocal_min_duty_pct_     = 13.0;  // below the measured 15-16% cruise duty
    int    autocal_min_samples_      = 40;    // ~10 s of steady driving at 4 Hz
    // Learning window, measured from the moment the rover first drives. The run
    // starts on (near-)flat ground, so this early stretch is the only stretch
    // where ticks-per-duty reflects the drivetrain rather than the terrain. Once
    // it closes the estimate is frozen, so later ramps can never drag it down.
    double autocal_window_sec_       = 60.0;
    bool   auto_enable_closed_loop_  = true;  // bumpless transition + 40% cap make this safe
    std::vector<double> autocal_samples_;     // observed full-scale estimates
    bool   autocal_done_             = false;
    bool   autocal_window_open_      = false;
    rclcpp::Time autocal_window_start_{0, 0, RCL_ROS_TIME};
    uint8_t autocal_prev_duty_       = 0;

    // === Stall detection ===
    bool   stall_detect_enabled_    = true;
    double stall_min_duty_pct_      = 30.0;  // only a genuine block, not low-duty under-power
    double stall_min_ticks_per_sec_ = 10.0;
    double stall_timeout_sec_       = 2.0;
    bool   stall_latched_           = false;
    bool   stall_timing_            = false;
    rclcpp::Time stall_since_{0, 0, RCL_ROS_TIME};

    uint8_t last_commanded_duty_pct_ = 0;  // what actually went out as spd_msg
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
            stall_timing_ = false;
            return;
        }

        const double measured_left_tps  = delta_left / dt;
        const double measured_right_tps = delta_right / dt;
        const double measured_ticks_per_sec = ((delta_left + delta_right) / 2.0) / dt;
        const double target_ticks_per_sec =
            (static_cast<double>(target_speed_pct_) / 100.0) * max_ticks_per_sec_;

        // Both of these run regardless of open/closed loop: calibration has to
        // observe the *open* loop to be meaningful, and a blocked wheel is just
        // as dangerous either way.
        if (auto_calibrate_max_ticks_ && !autocal_done_) {
            updateAutoCalibration(now, measured_ticks_per_sec);
        }
        if (stall_detect_enabled_) {
            updateStallDetection(now, measured_ticks_per_sec);
        }

        if (target_speed_pct_ == 0) {
            // Commanded stop: nothing to regulate, and letting the integral run
            // negative here would have to be unwound before the next start-up.
            resetSpeedPid();
            publishSpeedDebug(measured_left_tps, measured_right_tps,
                              measured_ticks_per_sec, target_ticks_per_sec, 0.0, 0.0f);
            return;
        }

        // --- Error in percent-of-full-scale rather than raw ticks/sec ---
        // Running the loop in the same 0–100% unit as its output keeps the gains
        // valid when max_ticks_per_sec is re-calibrated. With a ticks/sec error the
        // gains are scaled by the calibration constant and would silently need
        // rescaling every time it changes.
        const double measured_pct = (max_ticks_per_sec_ > 1e-6)
            ? (measured_ticks_per_sec / max_ticks_per_sec_) * 100.0
            : 0.0;
        const double error_pct = static_cast<double>(target_speed_pct_) - measured_pct;

        // --- Feedforward + PID trim ---
        // The commanded duty is the feedforward term, so at zero error the output is
        // exactly what open-loop would have sent and the PID only trims for load.
        // Previously the PID had to supply the entire operating point from its
        // integral alone, which speed_integral_limit made arithmetically impossible
        // (ki * limit capped the integral's authority far below the needed duty), so
        // the loop always settled well short of the commanded speed.
        const double derivative = (error_pct - speed_last_error_) / dt;

        const double integral_candidate = std::clamp(
            speed_integral_ + error_pct * dt,
            -speed_integral_limit_, speed_integral_limit_);
        const double u_candidate = static_cast<double>(target_speed_pct_)
                                 + speed_kp_ * error_pct
                                 + speed_ki_ * integral_candidate
                                 + speed_kd_ * derivative;

        // Conditional integration: stop accumulating once the output is saturated
        // and the error would only drive it further out of range (anti-windup on
        // top of the magnitude clamp — matters when a setpoint is unreachable,
        // e.g. climbing at full throttle).
        if (!((u_candidate > 100.0 && error_pct > 0.0) ||
              (u_candidate < 0.0   && error_pct < 0.0))) {
            speed_integral_ = integral_candidate;
        }
        speed_last_error_ = error_pct;

        const double u = static_cast<double>(target_speed_pct_)
                       + speed_kp_ * error_pct
                       + speed_ki_ * speed_integral_
                       + speed_kd_ * derivative;

        // --- Slew-rate limit on the duty output ---
        // Bounds how far the commanded duty can move per encoder update. A
        // badly calibrated max_ticks_per_sec otherwise produces a hard
        // 0 <-> 100 limit cycle at the encoder rate (measured reads as >100%
        // of full scale -> duty slammed to 0 -> wheels stop -> measured 0 ->
        // duty slammed to 100 -> repeat), felt as a ~2 Hz stop-and-spin
        // judder. This turns that failure into a slow surge rather than a
        // drivetrain-hammering square wave -- it does not fix the calibration,
        // it just stops a wrong constant from being violent.
        const double desired = std::clamp(u, 0.0, 100.0);
        const double max_step = speed_max_duty_step_pct_;
        const double prev = static_cast<double>(speed_pid_output_pct_);
        const double stepped = std::clamp(desired, prev - max_step, prev + max_step);
        speed_pid_output_pct_ = static_cast<float>(std::clamp(stepped, 0.0, 100.0));

        // --- Calibration sanity check ---
        // Sustained measured speed far above full scale means max_ticks_per_sec
        // is set well below the true tick rate. Warn rather than silently
        // fighting it, since this is the one input the loop cannot infer.
        if (measured_pct > 150.0) {
            if (++overspeed_strikes_ == 8) {   // ~2 s at the 4 Hz encoder rate
                RCLCPP_WARN(this->get_logger(),
                    "Measured speed reads %.0f%% of full scale — max_ticks_per_sec (%.0f) "
                    "looks too LOW for the real encoders. The speed loop cannot settle "
                    "and will surge. Calibrate it (measured_tps / (duty/100)) or set "
                    "use_closed_loop_speed:=false.",
                    measured_pct, max_ticks_per_sec_);
            }
        } else if (overspeed_strikes_ > 0) {
            overspeed_strikes_ = 0;
        }

        publishSpeedDebug(measured_left_tps, measured_right_tps, measured_ticks_per_sec,
                          target_ticks_per_sec, error_pct, speed_pid_output_pct_);
    }

    /**
     * @brief Publish the speed loop's internal signals so it is tunable from logs.
     *
     * Field 4 (error) is in percent-of-full-scale, matching the unit the PID
     * actually operates on — not ticks/sec. Fields 0–3 stay in ticks/sec so the
     * raw encoder measurement is still visible alongside it.
     */
    void publishSpeedDebug(double measured_left_tps, double measured_right_tps,
                           double measured_ticks_per_sec, double target_ticks_per_sec,
                           double error_pct, float output_pct) {
        auto debug_msg = std_msgs::msg::Float32MultiArray();
        debug_msg.data = {
            static_cast<float>(measured_left_tps),
            static_cast<float>(measured_right_tps),
            static_cast<float>(measured_ticks_per_sec),
            static_cast<float>(target_ticks_per_sec),
            static_cast<float>(error_pct),
            output_pct
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
    /**
     * @brief Learn max_ticks_per_sec from the flat opening stretch of a run.
     *
     * In open loop the commanded duty is known and the tick rate is measured, so
     * full-scale capability is directly observable as ticks_per_sec / (duty/100).
     * That ratio only means "capability" on level ground though — on a ramp it
     * drops because of load, and calibrating from it would bake the ramp into the
     * definition of 100% speed and permanently under-scale the loop.
     *
     * Runs start on near-flat ground, so sampling is confined to a window that
     * opens when the rover first drives and closes autocal_window_sec later. The
     * estimate is then frozen for the rest of the run.
     *
     * The 75th percentile of the window is used, which measured best against both
     * failure modes (100% accurate in every case simulated):
     *   - median  is right on pure flat ground but reads ~55% of true capability
     *     if a ramp starts inside the window, and an under-set value is exactly
     *     what makes the loop judder
     *   - maximum / p90 latches onto wheel slip, where the encoders spin at ~3x
     *     while the rover barely moves, over-reading by 300%
     * p75 leans toward the lightly-loaded (flat) samples without reaching the
     * slip outliers. Erring high is also the safer direction: too high merely
     * saturates smoothly, too low limit-cycles.
     *
     * Without an independent ground-speed reference, *sustained* slip through the
     * whole window would still corrupt this — the spread check below is what
     * makes that visible.
     */
    void updateAutoCalibration(const rclcpp::Time& now, double measured_ticks_per_sec) {
        const double duty = static_cast<double>(last_commanded_duty_pct_);

        // Only steady, meaningful duty tells us anything: below the deadband the
        // wheels barely turn, and a duty that just changed is still accelerating.
        const bool steady = (last_commanded_duty_pct_ == autocal_prev_duty_);
        autocal_prev_duty_ = last_commanded_duty_pct_;

        if (duty < autocal_min_duty_pct_ || measured_ticks_per_sec <= 0.0) {
            return;
        }

        if (!autocal_window_open_) {
            autocal_window_open_ = true;
            autocal_window_start_ = now;
            RCLCPP_INFO(this->get_logger(),
                "Auto-calibration window open (%.0f s) — learning max_ticks_per_sec "
                "from the flat opening stretch of this run.", autocal_window_sec_);
        }

        if (steady) {
            autocal_samples_.push_back(measured_ticks_per_sec / (duty / 100.0));
        }

        if ((now - autocal_window_start_).seconds() < autocal_window_sec_) {
            return;   // still collecting
        }

        autocal_done_ = true;   // window closed: decide once, then never again

        if (static_cast<int>(autocal_samples_.size()) < autocal_min_samples_) {
            RCLCPP_WARN(this->get_logger(),
                "Auto-calibration gave up: only %zu steady samples in %.0f s (need %d). "
                "Keeping max_ticks_per_sec=%.0f. Drive continuously above %.0f%% duty "
                "early in the run, or calibrate by hand.",
                autocal_samples_.size(), autocal_window_sec_, autocal_min_samples_,
                max_ticks_per_sec_, autocal_min_duty_pct_);
            return;
        }

        std::sort(autocal_samples_.begin(), autocal_samples_.end());
        const size_t n = autocal_samples_.size();
        const auto pctile = [&](double q) {
            return autocal_samples_[std::min(static_cast<size_t>(q * n), n - 1)];
        };
        const double learned  = pctile(0.75);
        const double previous = max_ticks_per_sec_;

        // Wide spread means conditions changed during the window (terrain, or
        // slip). The estimate is still usable but worth flagging.
        const double p25 = pctile(0.25);
        if (p25 > 0.0 && (learned / p25) > 1.5) {
            RCLCPP_WARN(this->get_logger(),
                "Auto-calibration samples are widely spread (p25=%.0f, p75=%.0f): the "
                "rover was not on uniform ground for the whole window. Estimate kept "
                "but verify it.", p25, learned);
        }
        max_ticks_per_sec_ = learned;
        resetSpeedPid();   // the error scale just changed under the integrator

        RCLCPP_INFO(this->get_logger(),
            "Auto-calibration complete: max_ticks_per_sec %.0f -> %.0f "
            "(75th pct of %zu samples). Put this in chassis_speed_control_params.yaml "
            "to skip the learning window next run.",
            previous, learned, autocal_samples_.size());

        if (auto_enable_closed_loop_ && !use_closed_loop_speed_) {
            use_closed_loop_speed_ = true;
            RCLCPP_WARN(this->get_logger(),
                "auto_enable_closed_loop is set — switching to closed-loop speed control now.");
        } else if (!use_closed_loop_speed_) {
            RCLCPP_INFO(this->get_logger(),
                "Still running open-loop. Enable with: ros2 param set "
                "/chassis_controller_node use_closed_loop_speed true");
        }
    }

    /**
     * @brief Detect a blocked/stalled drivetrain: duty commanded, wheels not turning.
     *
     * Distinct from the load case the speed loop exists to handle. On a ramp the
     * wheels still turn, just slower, and the right answer is more duty. If they
     * are not turning at all, more duty only heats a locked motor — so this stops
     * instead, and latches so it cannot chatter between stop and retry.
     *
     * Encoders alone cannot tell a stall from wheel slip (slip reads as fast
     * rotation), so this deliberately only claims the near-zero-motion case.
     */
    void updateStallDetection(const rclcpp::Time& now, double measured_ticks_per_sec) {
        const double duty = static_cast<double>(last_commanded_duty_pct_);

        if (duty < stall_min_duty_pct_) {
            // Not being asked to drive — clear the timer, and let a stop clear a latch.
            stall_timing_ = false;
            if (stall_latched_ && last_commanded_duty_pct_ == 0) {
                stall_latched_ = false;
                RCLCPP_INFO(this->get_logger(), "Stall latch cleared (speed commanded to zero).");
            }
            return;
        }

        if (std::fabs(measured_ticks_per_sec) >= stall_min_ticks_per_sec_) {
            stall_timing_ = false;   // moving
            return;
        }

        if (!stall_timing_) {
            stall_timing_ = true;
            stall_since_  = now;
            return;
        }

        if (!stall_latched_ && (now - stall_since_).seconds() >= stall_timeout_sec_) {
            stall_latched_ = true;
            RCLCPP_ERROR(this->get_logger(),
                "STALL: %.0f%% duty commanded but wheels reading %.1f ticks/s for %.1f s — "
                "stopping motors. Blocked wheel or drivetrain fault. Clears when speed "
                "is commanded to zero.",
                duty, measured_ticks_per_sec, stall_timeout_sec_);
        }
    }

    void resetSpeedPid() {
        speed_integral_       = 0.0;
        speed_last_error_     = 0.0;
        // Seed the output with the feedforward term, not zero. The closed loop
        // consumes this value at 50 Hz but only refreshes it at the ~4 Hz encoder
        // rate, so leaving a zero here means that on any transition into closed
        // loop (auto-enable mid-run, or the encoder feed recovering) the motors
        // are commanded to a dead stop for up to one encoder period. Starting at
        // the commanded duty makes those transitions bumpless: with zero error the
        // loop's output *is* the feedforward term, so this is the value it would
        // have computed anyway.
        speed_pid_output_pct_ = static_cast<float>(target_speed_pct_);
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

        // A latched stall overrides everything below the emergency stop: the
        // wheels are not turning under power, so commanding more is pointless
        // and damaging. Cleared when speed is commanded back to zero.
        if (stall_latched_) {
            chassis_ctrl.spd_msg = 0;
            chassis_ctrl.bdr_msg = 0;
        }

        // Remember the duty actually sent — auto-calibration and stall detection
        // both compare measured motion against what was really commanded, not
        // against the pre-cap request.
        last_commanded_duty_pct_ = chassis_ctrl.spd_msg;
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

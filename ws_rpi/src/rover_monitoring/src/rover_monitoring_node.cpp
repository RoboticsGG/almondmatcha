/**
 * rover_monitoring_node.cpp
 * Workspace:  ws_rpi  |  Package: pkg_rover_monitoring  |  Domain: 5
 *
 * Purpose:
 *   Full-fidelity local CSV logger on RPi — the one node responsible for
 *   per-topic CSVs at native sensor rates. Does NOT relay to Domain 4 (that
 *   is done by mission_monitoring_node_rpi, which is kept lean as the future
 *   home for a low-bitrate LPWAN telemetry link and intentionally carries no
 *   local-storage duties).
 *
 * Subscribed Topics (Domain 5):
 *   /tpc_gnss_spresense, /tpc_gnss_ublox, /tpc_chassis_cmd,
 *   /tpc_chassis_imu, /tpc_chassis_sensors, /tpc_rover_ctrl_cmd,
 *   /tpc_chassis_speed_debug
 *
 * Deliberately does NOT subscribe to /tpc_rover_nav_lane — that topic only
 * exists on Domain 6 (lane_detection_node publishes it there, invisible from
 * D5 by design) and this node runs entirely under Domain 5, so it could never
 * receive it anyway. Raw lane data is logged on the Jetson side instead
 * (ws_jetson_lane_detection_*.csv) — D5 and D6 logging are kept separate by
 * design, not bridged.
 *
 * Author: AlmondMatcha Rover Team
 * Date:   February 27, 2026
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "msgs_ifaces/msg/spresense_gnss.hpp"
#include "msgs_ifaces/msg/ublox_gnss.hpp"
#include "msgs_ifaces/msg/chassis_ctrl.hpp"
#include "msgs_ifaces/msg/chassis_sensors.hpp"
#include "msgs_ifaces/msg/chassis_imu.hpp"
#include <fstream>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <filesystem>
#include <cstdlib>
#include <glob.h>

class RoverMonitoringNode : public rclcpp::Node {
public:
    RoverMonitoringNode() : Node("rover_monitoring_node") {
        init_csv_logging();
        
        // QoS profiles to match publishers
        // STM32 mros2: EXACT copy from working chassis_sensors_node
        rclcpp::QoS qos_stm32(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        qos_stm32.best_effort();
        
        rclcpp::QoS qos_reliable_gnss(10);
        qos_reliable_gnss.reliable().transient_local();  // Match GNSS publishers
        
        // Subscribe to chassis sensors (EXACT QoS from chassis_sensors_node)
        sub_chassis_sensors_ = this->create_subscription<msgs_ifaces::msg::ChassisSensors>(
            "tpc_chassis_sensors", qos_stm32,
            std::bind(&RoverMonitoringNode::chassis_sensors_callback, this, std::placeholders::_1)
        );
        
        // Subscribe to chassis IMU (EXACT QoS from chassis_sensors_node)
        sub_chassis_imu_ = this->create_subscription<msgs_ifaces::msg::ChassisIMU>(
            "tpc_chassis_imu", qos_stm32,
            std::bind(&RoverMonitoringNode::chassis_imu_callback, this, std::placeholders::_1)
        );
        
        // Subscribe to mission control
        sub_cc_rcon_ = this->create_subscription<std_msgs::msg::Bool>(
            "tpc_gnss_mission_active", 10, 
            std::bind(&RoverMonitoringNode::cc_rcon_callback, this, std::placeholders::_1)
        );
        
        sub_dis_remain_ = this->create_subscription<std_msgs::msg::Float64>(
            "tpc_gnss_mission_remain_dist", 10, 
            std::bind(&RoverMonitoringNode::dis_remain_callback, this, std::placeholders::_1)
        );
        
        // Subscribe to GNSS data
        sub_gnss_data_ = this->create_subscription<msgs_ifaces::msg::SpresenseGNSS>(
            "tpc_gnss_spresense", 10, 
            std::bind(&RoverMonitoringNode::gnss_data_callback, this, std::placeholders::_1)
        );
        
        sub_rtk_position_ = this->create_subscription<msgs_ifaces::msg::UbloxGNSS>(
            "tpc_gnss_ublox", 10,
            std::bind(&RoverMonitoringNode::rtk_gnss_callback, this, std::placeholders::_1)
        );
        
        // Subscribe to destination coordinate
        sub_pub_despose_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "tpc_rover_dest_coordinate", 10, 
            std::bind(&RoverMonitoringNode::pub_despose_callback, this, std::placeholders::_1)
        );
        
        // Subscribe to rover control commands
        sub_pub_rovercontrol_d2_ = this->create_subscription<msgs_ifaces::msg::ChassisCtrl>(
            "tpc_chassis_cmd", 10,
            std::bind(&RoverMonitoringNode::pub_rovercontrol_callback, this, std::placeholders::_1)
        );

        // Kinematic control command (from Jetson → tpc_rover_ctrl_cmd)
        // Format: Float32MultiArray [steer_angle_deg, speed_cmd, detected]
        // BEST_EFFORT to match rover_kinematic_control_node.py publisher QoS
        sub_steering_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "tpc_rover_ctrl_cmd", qos_stm32,
            std::bind(&RoverMonitoringNode::steering_callback, this, std::placeholders::_1)
        );

        // Closed-loop speed PID debug signals (from chassis_controller_node)
        // Format: Float32MultiArray [measured_left_tps, measured_right_tps, measured_avg_tps,
        //                            target_tps, error, pid_output_pct]
        sub_speed_debug_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "tpc_chassis_speed_debug", qos_stm32,
            std::bind(&RoverMonitoringNode::speed_debug_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "=== CSV Data Logger Node Initialized ===");
        RCLCPP_INFO(this->get_logger(), "Event-driven CSV logging to: %s", log_dir_.c_str());
        RCLCPP_INFO(this->get_logger(), "Per-topic CSVs, each created on its first message:");
        RCLCPP_INFO(this->get_logger(), "  - rtk_gnss.csv (~10 Hz)");
        RCLCPP_INFO(this->get_logger(), "  - spresense_gnss.csv (~10 Hz)");
        RCLCPP_INFO(this->get_logger(), "  - chassis_imu.csv (~10 Hz)");
        RCLCPP_INFO(this->get_logger(), "  - chassis_sensors.csv (~4 Hz)");
        RCLCPP_INFO(this->get_logger(), "  - chassis_cmd.csv (~50 Hz)");
        RCLCPP_INFO(this->get_logger(), "  - mission_state.csv (event-driven)");
        RCLCPP_INFO(this->get_logger(), "  - chassis_speed_pid.csv (~4 Hz)");
        RCLCPP_INFO(this->get_logger(), "========================================");
    }

    ~RoverMonitoringNode() {
        if (csv_rtk_gnss_.is_open()) csv_rtk_gnss_.close();
        if (csv_spresense_gnss_.is_open()) csv_spresense_gnss_.close();
        if (csv_chassis_imu_.is_open()) csv_chassis_imu_.close();
        if (csv_chassis_sensors_.is_open()) csv_chassis_sensors_.close();
        if (csv_chassis_cmd_.is_open()) csv_chassis_cmd_.close();
        if (csv_mission_state_.is_open()) csv_mission_state_.close();
        if (csv_chassis_speed_pid_.is_open()) csv_chassis_speed_pid_.close();
    }

private:
    // Mission data
    bool cc_rcon_ = false;
    float dis_remain_ = 0.0f;
    float current_lat_ = 0.0f;
    float current_long_ = 0.0f;
    float des_lat_ = 0.0f;
    float des_long_ = 0.0f;
    
    // RTK GNSS data (Ublox)
    double rtk_latitude_ = 0.0;
    double rtk_longitude_ = 0.0;
    double rtk_altitude_ = 0.0;
    std::string rtk_fix_quality_ = "No Fix";
    float rtk_centimeter_error_ = 999.9f;
    int32_t rtk_satellites_ = 0;
    std::string rtk_date_ = "";
    std::string rtk_time_ = "";
    float rtk_snr_ = 0.0f;
    double rtk_speed_ = 0.0;

    // Spresense GNSS data
    std::string spresense_date_ = "";
    std::string spresense_time_ = "";
    int spresense_num_satellites_ = 0;
    bool spresense_fix_ = false;
    double spresense_latitude_ = 0.0;
    double spresense_longitude_ = 0.0;
    double spresense_altitude_ = 0.0;

    // Chassis sensors data
    int32_t motor_left_encoder_ = 0;
    int32_t motor_right_encoder_ = 0;
    float system_current_ = 0.0f;
    float system_voltage_ = 0.0f;
    float system_power_watts_ = 0.0f;

    // Steering control (from Jetson)
    float steering_command_ = 0.0f;

    // Chassis IMU data
    int32_t accel_x_ = 0;
    int32_t accel_y_ = 0;
    int32_t accel_z_ = 0;
    int32_t gyro_x_ = 0;
    int32_t gyro_y_ = 0;
    int32_t gyro_z_ = 0;

    // Control commands
    uint8_t fdr_msg_ = 0;
    float ro_ctrl_msg_ = 0.0f;
    uint8_t spd_msg_ = 0;
    uint8_t bdr_msg_ = 0;

    // Separate CSV files for each topic (full-rate event-driven)
    std::ofstream csv_rtk_gnss_;
    std::ofstream csv_spresense_gnss_;
    std::ofstream csv_chassis_imu_;
    std::ofstream csv_chassis_sensors_;
    std::ofstream csv_chassis_cmd_;
    std::ofstream csv_mission_state_;
    std::ofstream csv_chassis_speed_pid_;
    std::string log_dir_;
    bool run_dir_created_  = false;  // run dir is created on first write, not at startup
    bool dir_error_logged_ = false;  // log directory failures once, not per message
    bool file_error_logged_ = false;

    // Subscriptions
    rclcpp::Subscription<msgs_ifaces::msg::ChassisSensors>::SharedPtr sub_chassis_sensors_;
    rclcpp::Subscription<msgs_ifaces::msg::ChassisIMU>::SharedPtr sub_chassis_imu_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_cc_rcon_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_dis_remain_;
    rclcpp::Subscription<msgs_ifaces::msg::SpresenseGNSS>::SharedPtr sub_gnss_data_;
    rclcpp::Subscription<msgs_ifaces::msg::UbloxGNSS>::SharedPtr sub_rtk_position_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_pub_despose_;
    rclcpp::Subscription<msgs_ifaces::msg::ChassisCtrl>::SharedPtr sub_pub_rovercontrol_d2_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_steering_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_speed_debug_;

    int get_next_run_number(const std::string& runs_dir) {
        // Find all existing run directories and get the highest run number
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

    /**
     * @brief Resolve the ws_rpi workspace root.
     *
     * Walks up from the current working directory looking for a directory named
     * ws_rpi that contains src/ (launch_rover_tmux.sh cds into the workspace, so
     * this is the normal path), and otherwise falls back to an explicit
     * $HOME/almondmatcha/ws_rpi.
     *
     * The explicit fallback is the point: the previous version left the path as
     * the bare cwd when it couldn't find ws_rpi, so starting the node from an
     * unexpected directory silently scattered run_NNN_ directories wherever the
     * process happened to launch from. This can only ever return a ws_rpi root.
     */
    std::string resolve_ws_rpi_root() {
        auto path = std::filesystem::current_path();
        while (true) {
            if (path.filename() == "ws_rpi" && std::filesystem::exists(path / "src")) {
                return path.string();
            }
            if (!path.has_parent_path() || path.parent_path() == path) {
                break;
            }
            path = path.parent_path();
        }

        const char* home = std::getenv("HOME");
        std::string fallback = std::string(home ? home : "/root") + "/almondmatcha/ws_rpi";
        RCLCPP_WARN(this->get_logger(),
                    "Not running from inside ws_rpi (cwd=%s) — logging to %s",
                    std::filesystem::current_path().string().c_str(), fallback.c_str());
        return fallback;
    }

    /**
     * @brief Compute (but do not create) this run's log directory.
     *
     * Nothing is written to disk here. The run directory and each CSV are
     * created on first use instead, so a launch that never receives data leaves
     * no empty run_NNN_ directory full of header-only files behind — and does
     * not consume a run number either.
     */
    void init_csv_logging() {
        std::string runs_dir = resolve_ws_rpi_root() + "/runs";

        // Prefer the launcher's directory. launch_rover_tmux.sh computes one
        // run directory per launch and exports it as $ROVER_RUN_DIR, exactly as
        // ws_jetson does, so every logger on this machine writes into the same
        // folder. Allocating independently here meant each launch produced TWO
        // directories on the RPi -- the launcher's, holding the per-node console
        // logs, and this node's, holding the CSVs -- splitting one run across
        // two folders and consuming two run numbers.
        //
        // Falls back to allocating its own when the variable is unset, which is
        // the case when this node is started by hand with `ros2 run`.
        if (const char* from_env = std::getenv("ROVER_RUN_DIR")) {
            if (from_env[0] != '\0') {
                log_dir_ = std::string(from_env);
                RCLCPP_INFO(this->get_logger(),
                            "CSV logging armed: %s (from $ROVER_RUN_DIR)", log_dir_.c_str());
                return;
            }
        }

        int run_number = get_next_run_number(runs_dir);
        auto now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);
        std::tm *timeinfo = std::localtime(&now_c);
        char buffer[50];
        std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", timeinfo);

        std::ostringstream run_dir_ss;
        run_dir_ss << runs_dir << "/run_" << std::setfill('0') << std::setw(3) << run_number
                   << "_" << buffer;
        log_dir_ = run_dir_ss.str();

        RCLCPP_INFO(this->get_logger(),
                    "Event-driven CSV logging armed: Run #%d — files are created on "
                    "first message per topic, at %s",
                    run_number, log_dir_.c_str());
    }

    /**
     * @brief Create the run directory on first actual write.
     */
    bool ensure_run_dir() {
        if (run_dir_created_) {
            return true;
        }
        try {
            std::filesystem::create_directories(log_dir_);
            run_dir_created_ = true;
            RCLCPP_INFO(this->get_logger(), "Created run directory: %s", log_dir_.c_str());
        } catch (const std::exception &e) {
            if (!dir_error_logged_) {
                dir_error_logged_ = true;
                RCLCPP_ERROR(this->get_logger(),
                             "Failed to create run directory %s: %s — logging disabled",
                             log_dir_.c_str(), e.what());
            }
            return false;
        }
        return true;
    }

    /**
     * @brief Open a CSV and write its header on first use; no-op afterwards.
     *
     * @return true if the stream is ready to be written to.
     */
    bool ensure_csv(std::ofstream& out, const char* filename, const char* header) {
        if (out.is_open()) {
            return true;
        }
        if (!ensure_run_dir()) {
            return false;
        }
        out.open(log_dir_ + "/" + filename, std::ios::out);
        if (!out.is_open()) {
            if (!file_error_logged_) {
                file_error_logged_ = true;
                RCLCPP_ERROR(this->get_logger(), "Failed to open %s in %s",
                             filename, log_dir_.c_str());
            }
            return false;
        }
        out << header;
        RCLCPP_INFO(this->get_logger(), "Logging %s (first message received)", filename);
        return true;
    }

    void chassis_sensors_callback(const msgs_ifaces::msg::ChassisSensors::SharedPtr msg) {
        motor_left_encoder_ = msg->mt_lf_encode_msg;
        motor_right_encoder_ = msg->mt_rt_encode_msg;
        system_current_ = msg->sys_current_msg;
        system_voltage_ = msg->sys_volt_msg;
        system_power_watts_ = system_voltage_ * system_current_;

        // Write to CSV immediately (event-driven, ~4 Hz)
        if (ensure_csv(csv_chassis_sensors_, "chassis_sensors.csv", "Timestamp_us,Motor_Left_Encoder,Motor_Right_Encoder,System_Current_A,System_Voltage_V,Power_W\n")) {
            auto timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()
            ).count();

            csv_chassis_sensors_ << timestamp_us << ","
                               << msg->mt_lf_encode_msg << ","
                               << msg->mt_rt_encode_msg << ","
                               << msg->sys_current_msg << ","
                               << msg->sys_volt_msg << ","
                               << system_power_watts_ << "\n";
            csv_chassis_sensors_.flush();
        }
    }

    void chassis_imu_callback(const msgs_ifaces::msg::ChassisIMU::SharedPtr msg) {
        accel_x_ = msg->accel_x;
        accel_y_ = msg->accel_y;
        accel_z_ = msg->accel_z;
        gyro_x_ = msg->gyro_x;
        gyro_y_ = msg->gyro_y;
        gyro_z_ = msg->gyro_z;
        
        // Write to CSV immediately (event-driven, ~10 Hz)
        if (ensure_csv(csv_chassis_imu_, "chassis_imu.csv", "Timestamp_us,Accel_X,Accel_Y,Accel_Z,Gyro_X,Gyro_Y,Gyro_Z\n")) {
            auto timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()
            ).count();
            
            csv_chassis_imu_ << timestamp_us << ","
                           << msg->accel_x << ","
                           << msg->accel_y << ","
                           << msg->accel_z << ","
                           << msg->gyro_x << ","
                           << msg->gyro_y << ","
                           << msg->gyro_z << "\n";
            csv_chassis_imu_.flush();
        }
    }

    void cc_rcon_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        cc_rcon_ = msg->data;
        write_mission_state();
    }
    
    void dis_remain_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        dis_remain_ = msg->data;
        write_mission_state();
    }
    
    void gnss_data_callback(const msgs_ifaces::msg::SpresenseGNSS::SharedPtr msg) {
        spresense_date_ = msg->date;
        spresense_time_ = msg->time;
        spresense_num_satellites_ = msg->num_satellites;
        spresense_fix_ = msg->fix;
        spresense_latitude_ = msg->latitude;
        spresense_longitude_ = msg->longitude;
        spresense_altitude_ = msg->altitude;
        
        // Also use for current position
        current_lat_ = msg->latitude;
        current_long_ = msg->longitude;
        
        // Write to CSV immediately (event-driven, ~10 Hz)
        if (ensure_csv(csv_spresense_gnss_, "spresense_gnss.csv", "Timestamp_us,Date,Time,Num_Satellites,Fix,Latitude,Longitude,Altitude\n")) {
            auto timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()
            ).count();
            
            csv_spresense_gnss_ << timestamp_us << ","
                               << msg->date << ","
                               << msg->time << ","
                               << msg->num_satellites << ","
                               << (msg->fix ? "true" : "false") << ","
                               << msg->latitude << ","
                               << msg->longitude << ","
                               << msg->altitude << "\n";
            csv_spresense_gnss_.flush();
        }
    }
    
    void rtk_gnss_callback(const msgs_ifaces::msg::UbloxGNSS::SharedPtr msg) {
        rtk_date_ = msg->date;
        rtk_time_ = msg->time;
        rtk_latitude_ = msg->latitude;
        rtk_longitude_ = msg->longitude;
        rtk_altitude_ = msg->altitude;
        rtk_fix_quality_ = msg->fix_quality;
        rtk_centimeter_error_ = msg->centimeter_error;
        rtk_satellites_ = msg->satellites_tracked;
        rtk_snr_ = msg->snr;
        rtk_speed_ = msg->speed;
        
        // Write to CSV immediately (event-driven, ~10 Hz)
        if (ensure_csv(csv_rtk_gnss_, "rtk_gnss.csv", "Timestamp_us,Date,Time,Latitude,Longitude,Altitude,Fix_Quality,Centimeter_Error,Satellites,SNR,Speed_ms\n")) {
            auto timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()
            ).count();
            
            csv_rtk_gnss_ << timestamp_us << ","
                         << msg->date << ","
                         << msg->time << ","
                         << msg->latitude << ","
                         << msg->longitude << ","
                         << msg->altitude << ","
                         << msg->fix_quality << ","
                         << msg->centimeter_error << ","
                         << msg->satellites_tracked << ","
                         << msg->snr << ","
                         << msg->speed << "\n";
            csv_rtk_gnss_.flush();
        }
    }
    
    void pub_despose_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 2) {
            des_lat_ = msg->data[0];
            des_long_ = msg->data[1];
        }
        write_mission_state();
    }
    
    void write_mission_state() {
        // Write mission state whenever any mission-related topic updates.
        // Steering_Cmd rides along with whatever its latest value is — it
        // doesn't force a row on its own (the Jetson-side kinematic_ctrl CSV
        // already covers that at full control-loop rate).
        if (ensure_csv(csv_mission_state_, "mission_state.csv", "Timestamp_us,Mission_Active,Distance_Remaining_m,Dest_Latitude,Dest_Longitude,Steering_Cmd\n")) {
            auto timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()
            ).count();

            csv_mission_state_ << timestamp_us << ","
                             << (cc_rcon_ ? "true" : "false") << ","
                             << dis_remain_ << ","
                             << des_lat_ << ","
                             << des_long_ << ","
                             << steering_command_ << "\n";
            csv_mission_state_.flush();
        }
    }

    void steering_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        // tpc_rover_ctrl_cmd format: [steer_angle_deg, speed_cmd, detected]
        if (msg->data.size() < 3) {
            return;
        }
        steering_command_ = msg->data[0];
    }

    void speed_debug_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        // tpc_chassis_speed_debug format: [measured_left_tps, measured_right_tps,
        //                                   measured_avg_tps, target_tps, error, pid_output_pct]
        if (msg->data.size() < 6) {
            return;
        }

        // Write to CSV immediately (event-driven, ~4 Hz — paced by the encoder feed)
        if (ensure_csv(csv_chassis_speed_pid_, "chassis_speed_pid.csv", "Timestamp_us,Measured_Left_TPS,Measured_Right_TPS,Measured_Avg_TPS,Target_TPS,Error_Pct,PID_Output_Pct\n")) {
            auto timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()
            ).count();
            csv_chassis_speed_pid_ << timestamp_us << ","
                                  << msg->data[0] << "," << msg->data[1] << ","
                                  << msg->data[2] << "," << msg->data[3] << ","
                                  << msg->data[4] << "," << msg->data[5] << "\n";
            csv_chassis_speed_pid_.flush();
        }
    }
    
    void pub_rovercontrol_callback(const msgs_ifaces::msg::ChassisCtrl::SharedPtr msg) {
        fdr_msg_ = msg->fdr_msg;
        ro_ctrl_msg_ = msg->ro_ctrl_msg;
        spd_msg_ = msg->spd_msg;
        bdr_msg_ = msg->bdr_msg;
        
        // Write to chassis command CSV immediately (event-driven, ~50 Hz)
        if (ensure_csv(csv_chassis_cmd_, "chassis_cmd.csv", "Timestamp_us,FDR_Msg,RO_Ctrl_Deg,SPD_Msg,BDR_Msg\n")) {
            auto timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()
            ).count();
            
            csv_chassis_cmd_ << timestamp_us << ","
                           << static_cast<int>(msg->fdr_msg) << ","
                           << msg->ro_ctrl_msg << ","
                           << static_cast<int>(msg->spd_msg) << ","
                           << static_cast<int>(msg->bdr_msg) << "\n";
            csv_chassis_cmd_.flush();
        }
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RoverMonitoringNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

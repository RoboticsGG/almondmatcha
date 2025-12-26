#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "msgs_ifaces/msg/spresense_gnss.hpp"
#include "msgs_ifaces/msg/ublox_gnss.hpp"
#include "msgs_ifaces/msg/chassis_ctrl.hpp"
#include "msgs_ifaces/msg/chassis_sensors.hpp"
#include "msgs_ifaces/msg/chassis_imu.hpp"
#include "msgs_ifaces/msg/rover_status.hpp"
#include <fstream>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <filesystem>
#include <glob.h>

class NodeRoverMonitoring : public rclcpp::Node {
public:
    NodeRoverMonitoring() : Node("node_rover_monitoring") {
        init_csv_logging();
        
        // Subscribe to chassis sensors
        sub_chassis_sensors_ = this->create_subscription<msgs_ifaces::msg::ChassisSensors>(
            "/tpc_chassis_sensors", 10,
            std::bind(&NodeRoverMonitoring::chassis_sensors_callback, this, std::placeholders::_1)
        );
        
        // Subscribe to chassis IMU
        sub_chassis_imu_ = this->create_subscription<msgs_ifaces::msg::ChassisIMU>(
            "/tpc_chassis_imu", 10,
            std::bind(&NodeRoverMonitoring::chassis_imu_callback, this, std::placeholders::_1)
        );
        
        // Subscribe to mission control
        sub_cc_rcon_ = this->create_subscription<std_msgs::msg::Bool>(
            "/tpc_gnss_mission_active", 10, 
            std::bind(&NodeRoverMonitoring::cc_rcon_callback, this, std::placeholders::_1)
        );
        
        sub_dis_remain_ = this->create_subscription<std_msgs::msg::Float64>(
            "/tpc_gnss_mission_remain_dist", 10, 
            std::bind(&NodeRoverMonitoring::dis_remain_callback, this, std::placeholders::_1)
        );
        
        // Subscribe to GNSS data
        sub_gnss_data_ = this->create_subscription<msgs_ifaces::msg::SpresenseGNSS>(
            "/tpc_gnss_spresense", 10, 
            std::bind(&NodeRoverMonitoring::gnss_data_callback, this, std::placeholders::_1)
        );
        
        sub_rtk_position_ = this->create_subscription<msgs_ifaces::msg::UbloxGNSS>(
            "/tpc_gnss_ublox", 10,
            std::bind(&NodeRoverMonitoring::rtk_gnss_callback, this, std::placeholders::_1)
        );
        
        // Subscribe to destination coordinate
        sub_pub_despose_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/tpc_rover_dest_coordinate", 10, 
            std::bind(&NodeRoverMonitoring::pub_despose_callback, this, std::placeholders::_1)
        );
        
        // Subscribe to rover control commands
        sub_pub_rovercontrol_d2_ = this->create_subscription<msgs_ifaces::msg::ChassisCtrl>(
            "/tpc_chassis_cmd", 10, 
            std::bind(&NodeRoverMonitoring::pub_rovercontrol_callback, this, std::placeholders::_1)
        );

        // Publisher for aggregated status to base station (Domain 4)
        pub_rover_status_ = this->create_publisher<msgs_ifaces::msg::RoverStatus>(
            "/tpc_rover_status", 10
        );

        // Timer for publishing aggregated status and logging
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000), 
            std::bind(&NodeRoverMonitoring::timer_callback, this)
        );

        RCLCPP_INFO(this->get_logger(), "=== Rover Monitoring Node Initialized ===");
        RCLCPP_INFO(this->get_logger(), "Subscribing to all rover topics (Domain 5)");
        RCLCPP_INFO(this->get_logger(), "Publishing aggregated status to /tpc_rover_status");
        RCLCPP_INFO(this->get_logger(), "CSV logging to: %s", csv_filename_.c_str());
        RCLCPP_INFO(this->get_logger(), "========================================");
    }

    ~NodeRoverMonitoring() {
        if (csv_file_.is_open()) {
            csv_file_.close();
        }
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

    std::ofstream csv_file_;
    std::string csv_filename_;
    rclcpp::Publisher<msgs_ifaces::msg::RoverStatus>::SharedPtr pub_rover_status_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Subscriptions
    rclcpp::Subscription<msgs_ifaces::msg::ChassisSensors>::SharedPtr sub_chassis_sensors_;
    rclcpp::Subscription<msgs_ifaces::msg::ChassisIMU>::SharedPtr sub_chassis_imu_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_cc_rcon_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_dis_remain_;
    rclcpp::Subscription<msgs_ifaces::msg::SpresenseGNSS>::SharedPtr sub_gnss_data_;
    rclcpp::Subscription<msgs_ifaces::msg::UbloxGNSS>::SharedPtr sub_rtk_position_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_pub_despose_;
    rclcpp::Subscription<msgs_ifaces::msg::ChassisCtrl>::SharedPtr sub_pub_rovercontrol_d2_;

    int get_next_run_number(const std::string& log_dir) {
        // Find all existing CSV files and get the highest run number
        glob_t glob_result;
        std::string pattern = log_dir + "/run_*.csv";
        int max_run = 0;
        
        if (glob(pattern.c_str(), GLOB_TILDE, NULL, &glob_result) == 0) {
            for (size_t i = 0; i < glob_result.gl_pathc; ++i) {
                std::string filename = glob_result.gl_pathv[i];
                size_t pos = filename.find("run_");
                if (pos != std::string::npos) {
                    std::string num_str = filename.substr(pos + 4, filename.find("_", pos + 4) - (pos + 4));
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
        // Get the absolute path to ws_rpi directory
        std::string ws_rpi_path = std::filesystem::current_path().string();
        
        // Navigate up to find ws_rpi root if we're in a subdirectory
        while (!ws_rpi_path.empty() && 
               ws_rpi_path.find("ws_rpi") != std::string::npos &&
               !std::filesystem::exists(ws_rpi_path + "/src")) {
            ws_rpi_path = std::filesystem::path(ws_rpi_path).parent_path().string();
        }
        
        // If we couldn't find ws_rpi, use relative path
        if (ws_rpi_path.find("ws_rpi") == std::string::npos) {
            // Try to find ws_rpi in the path
            auto path = std::filesystem::current_path();
            while (path.has_parent_path()) {
                if (path.filename() == "ws_rpi") {
                    ws_rpi_path = path.string();
                    break;
                }
                path = path.parent_path();
            }
        }
        
        // Create runs directory relative to ws_rpi
        std::string log_dir = ws_rpi_path + "/runs";
        
        if (!std::filesystem::exists(log_dir)) {
            try {
                std::filesystem::create_directories(log_dir);
                RCLCPP_INFO(this->get_logger(), "Created log directory: %s", log_dir.c_str());
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to create log directory: %s", e.what());
                log_dir = "/tmp";  // Fallback to /tmp
            }
        }

        // Get run number and timestamp
        int run_number = get_next_run_number(log_dir);
        auto now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);
        std::tm *timeinfo = std::localtime(&now_c);
        char buffer[50];
        std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", timeinfo);
        
        // Format: run_NNN_YYYYMMDD_HHMMSS.csv
        std::ostringstream filename_ss;
        filename_ss << log_dir << "/run_" << std::setfill('0') << std::setw(3) << run_number 
                    << "_" << buffer << ".csv";
        csv_filename_ = filename_ss.str();

        csv_file_.open(csv_filename_, std::ios::out);
        if (!csv_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file for writing!");
            return;
        }

        // Write CSV header with all fields
        csv_file_ << "Timestamp,"
                  // RTK GNSS (Ublox)
                  << "RTK_Date,RTK_Time,RTK_Latitude,RTK_Longitude,RTK_Altitude,"
                  << "RTK_Fix_Quality,RTK_Centimeter_Error,RTK_Satellites,RTK_SNR,RTK_Speed,"
                  // Spresense GNSS
                  << "Spresense_Date,Spresense_Time,Spresense_NumSatellites,Spresense_Fix,"
                  << "Spresense_Latitude,Spresense_Longitude,Spresense_Altitude,"
                  // Chassis Sensors
                  << "Motor_Left_Encoder,Motor_Right_Encoder,System_Current,System_Voltage,"
                  // Chassis IMU
                  << "Accel_X,Accel_Y,Accel_Z,Gyro_X,Gyro_Y,Gyro_Z,"
                  // Mission & Control
                  << "Mission_Active,Distance_Remaining,Dest_Lat,Dest_Long,"
                  << "FDR_Msg,RO_Ctrl,SPD_Msg,BDR_Msg\n";
        
        RCLCPP_INFO(this->get_logger(), "CSV logging initialized: Run #%d", run_number);
    }

    void chassis_sensors_callback(const msgs_ifaces::msg::ChassisSensors::SharedPtr msg) {
        motor_left_encoder_ = msg->mt_lf_encode_msg;
        motor_right_encoder_ = msg->mt_rt_encode_msg;
        system_current_ = msg->sys_current_msg;
        system_voltage_ = msg->sys_volt_msg;
    }

    void chassis_imu_callback(const msgs_ifaces::msg::ChassisIMU::SharedPtr msg) {
        accel_x_ = msg->accel_x;
        accel_y_ = msg->accel_y;
        accel_z_ = msg->accel_z;
        gyro_x_ = msg->gyro_x;
        gyro_y_ = msg->gyro_y;
        gyro_z_ = msg->gyro_z;
    }

    void cc_rcon_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        cc_rcon_ = msg->data;
    }
    
    void dis_remain_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        dis_remain_ = msg->data;
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
    }
    
    void pub_despose_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 2) {
            des_lat_ = msg->data[0];
            des_long_ = msg->data[1];
        }
    }
    
    void pub_rovercontrol_callback(const msgs_ifaces::msg::ChassisCtrl::SharedPtr msg) {
        fdr_msg_ = msg->fdr_msg;
        ro_ctrl_msg_ = msg->ro_ctrl_msg;
        spd_msg_ = msg->spd_msg;
        bdr_msg_ = msg->bdr_msg;
    }
    
    void timer_callback() {
        // Write to CSV
        if (csv_file_.is_open()) {
            auto now = std::chrono::system_clock::now();
            auto time_t = std::chrono::system_clock::to_time_t(now);
            
            csv_file_ << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S") << ","
                      // RTK GNSS
                      << rtk_date_ << "," << rtk_time_ << "," 
                      << rtk_latitude_ << "," << rtk_longitude_ << "," << rtk_altitude_ << ","
                      << rtk_fix_quality_ << "," << rtk_centimeter_error_ << "," 
                      << rtk_satellites_ << "," << rtk_snr_ << "," << rtk_speed_ << ","
                      // Spresense GNSS
                      << spresense_date_ << "," << spresense_time_ << "," 
                      << spresense_num_satellites_ << "," << (spresense_fix_ ? "true" : "false") << ","
                      << spresense_latitude_ << "," << spresense_longitude_ << "," 
                      << spresense_altitude_ << ","
                      // Chassis Sensors
                      << motor_left_encoder_ << "," << motor_right_encoder_ << ","
                      << system_current_ << "," << system_voltage_ << ","
                      // Chassis IMU
                      << accel_x_ << "," << accel_y_ << "," << accel_z_ << ","
                      << gyro_x_ << "," << gyro_y_ << "," << gyro_z_ << ","
                      // Mission & Control
                      << (cc_rcon_ ? "true" : "false") << "," << dis_remain_ << ","
                      << des_lat_ << "," << des_long_ << ","
                      << static_cast<int>(fdr_msg_) << "," << ro_ctrl_msg_ << ","
                      << static_cast<int>(spd_msg_) << "," << static_cast<int>(bdr_msg_) << "\n";
            csv_file_.flush();
        }

        // Publish aggregated status for base station (relayed to Domain 4)
        auto status_msg = msgs_ifaces::msg::RoverStatus();
        status_msg.mission_active = cc_rcon_;
        status_msg.distance_remaining = dis_remain_;
        status_msg.current_latitude = current_lat_;
        status_msg.current_longitude = current_long_;
        status_msg.destination_latitude = des_lat_;
        status_msg.destination_longitude = des_long_;
        status_msg.rtk_latitude = rtk_latitude_;
        status_msg.rtk_longitude = rtk_longitude_;
        status_msg.rtk_altitude = rtk_altitude_;
        status_msg.rtk_fix_quality = rtk_fix_quality_;
        status_msg.rtk_centimeter_error = rtk_centimeter_error_;
        status_msg.rtk_satellites = rtk_satellites_;
        status_msg.fdr_msg = fdr_msg_;
        status_msg.ro_ctrl_msg = ro_ctrl_msg_;
        status_msg.spd_msg = spd_msg_;
        status_msg.bdr_msg = bdr_msg_;
        
        pub_rover_status_->publish(status_msg);
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NodeRoverMonitoring>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

#include <array>
#include <chrono>
#include <ctime>
#include <fstream>
#include <memory>
#include <sstream>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "msgs_ifaces/msg/chassis_sensors.hpp"
#include <rcpputils/filesystem_helper.hpp>

using namespace std::chrono_literals;

const std::string LOG_DIR = "/home/curry/almondmatcha/runs/logs/";

class ChassisSensorsNode : public rclcpp::Node {
public:
    ChassisSensorsNode() : Node("chassis_sensors_node") {
        setupSubscriber();
        setupLogging();
        
        timer_ = this->create_wall_timer(1s, std::bind(&ChassisSensorsNode::writeToCSV, this));
    }

    ~ChassisSensorsNode() {
        if (csv_file_.is_open()) {
            csv_file_.close();
        }
    }

private:
    // ROS2 Components
    rclcpp::Subscription<msgs_ifaces::msg::ChassisSensors>::SharedPtr sub_chassis_sensors_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Data Management
    msgs_ifaces::msg::ChassisSensors latest_sensors_data_;
    bool has_data_ = false;
    std::ofstream csv_file_;

    void setupSubscriber() {
        rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        qos_profile.best_effort();
        qos_profile.transient_local();

        sub_chassis_sensors_ = this->create_subscription<msgs_ifaces::msg::ChassisSensors>(
            "/tpc_chassis_sensors", qos_profile,
            std::bind(&ChassisSensorsNode::sensorsCallback, this, std::placeholders::_1)
        );
        
        RCLCPP_INFO(this->get_logger(), "Subscribed to /tpc_chassis_sensors");
    }

    void setupLogging() {
        if (!rcpputils::fs::exists(LOG_DIR)) {
            try {
                rcpputils::fs::create_directories(LOG_DIR);
                RCLCPP_INFO(this->get_logger(), "Created log directory: %s", LOG_DIR.c_str());
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to create log directory: %s", e.what());
            }
        }

        std::string filename = generateLogFilename();
        csv_file_.open(filename, std::ios::out);
        
        if (!csv_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", filename.c_str());
            return;
        }

        csv_file_ << "timestamp,motor_left_encoder,motor_right_encoder,system_current,system_voltage\n";
        RCLCPP_INFO(this->get_logger(), "Logging to: %s", filename.c_str());
    }

    std::string generateLogFilename() {
        auto now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);
        std::tm *local_time = std::localtime(&now_c);
        
        std::ostringstream oss;
        oss << LOG_DIR << "chassis_sensors_"
            << std::put_time(local_time, "%Y%m%d_%H%M%S")
            << ".csv";
        
        return oss.str();
    }

    void sensorsCallback(const msgs_ifaces::msg::ChassisSensors::SharedPtr msg) {
        latest_sensors_data_ = *msg;
        has_data_ = true;

        RCLCPP_INFO(this->get_logger(),
                    "Chassis Sensors - Encoders: [L:%d, R:%d] | Power: [I:%.2fA, V:%.2fV]",
                    msg->mt_lf_encode_msg, msg->mt_rt_encode_msg,
                    msg->sys_current_msg, msg->sys_volt_msg);
    }

    void writeToCSV() {
        if (!has_data_ || !csv_file_.is_open()) {
            return;
        }

        std::string timestamp = getCurrentTimestamp();
        const auto &sensors = latest_sensors_data_;
        
        csv_file_ << timestamp << ","
                  << sensors.mt_lf_encode_msg << ","
                  << sensors.mt_rt_encode_msg << ","
                  << sensors.sys_current_msg << ","
                  << sensors.sys_volt_msg << "\n";
        csv_file_.flush();
    }

    std::string getCurrentTimestamp() {
        auto now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);
        std::tm *local_time = std::localtime(&now_c);
        
        std::ostringstream oss;
        oss << std::put_time(local_time, "%Y-%m-%d %H:%M:%S");
        
        return oss.str();
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ChassisSensorsNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
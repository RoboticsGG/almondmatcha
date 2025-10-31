#include <array>
#include <chrono>
#include <ctime>
#include <fstream>
#include <memory>
#include <sstream>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "msgs_ifaces/msg/chassis_imu.hpp"
#include <rcpputils/filesystem_helper.hpp>

using namespace std::chrono_literals;

const std::string LOG_DIR = "/home/curry/almondmatcha/runs/logs/";

class ChassisIMUNode : public rclcpp::Node {
public:
    ChassisIMUNode() : Node("chassis_imu_node") {
        setupSubscriber();
        setupLogging();
        
        timer_ = this->create_wall_timer(1s, std::bind(&ChassisIMUNode::writeToCSV, this));
    }

    ~ChassisIMUNode() {
        if (csv_file_.is_open()) {
            csv_file_.close();
        }
    }

private:
    // ROS2 Components
    rclcpp::Subscription<msgs_ifaces::msg::ChassisIMU>::SharedPtr sub_chassis_imu_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Data Management
    msgs_ifaces::msg::ChassisIMU latest_imu_data_;
    bool has_data_ = false;
    std::ofstream csv_file_;

    void setupSubscriber() {
        rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        qos_profile.best_effort();
        qos_profile.transient_local();

        sub_chassis_imu_ = this->create_subscription<msgs_ifaces::msg::ChassisIMU>(
            "/tp_imu_data_d5", qos_profile,
            std::bind(&ChassisIMUNode::imuCallback, this, std::placeholders::_1)
        );
        
        RCLCPP_INFO(this->get_logger(), "Subscribed to /tp_imu_data_d5");
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

        csv_file_ << "timestamp,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z\n";
        RCLCPP_INFO(this->get_logger(), "Logging to: %s", filename.c_str());
    }

    std::string generateLogFilename() {
        auto now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);
        std::tm *local_time = std::localtime(&now_c);
        
        std::ostringstream oss;
        oss << LOG_DIR << "chassis_imu_"
            << std::put_time(local_time, "%Y%m%d_%H%M%S")
            << ".csv";
        
        return oss.str();
    }

    void imuCallback(const msgs_ifaces::msg::ChassisIMU::SharedPtr msg) {
        latest_imu_data_ = *msg;
        has_data_ = true;

        RCLCPP_INFO(this->get_logger(),
                    "Chassis IMU - Accel: [%d, %d, %d] | Gyro: [%d, %d, %d]",
                    msg->accel_x, msg->accel_y, msg->accel_z,
                    msg->gyro_x, msg->gyro_y, msg->gyro_z);
    }

    void writeToCSV() {
        if (!has_data_ || !csv_file_.is_open()) {
            return;
        }

        std::string timestamp = getCurrentTimestamp();
        const auto &chassis_imu = latest_imu_data_;
        
        csv_file_ << timestamp << ","
                  << chassis_imu.accel_x << ","
                  << chassis_imu.accel_y << ","
                  << chassis_imu.accel_z << ","
                  << chassis_imu.gyro_x << ","
                  << chassis_imu.gyro_y << ","
                  << chassis_imu.gyro_z << "\n";
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
    auto node = std::make_shared<ChassisIMUNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

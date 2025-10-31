#include <cstdio>
#include <fstream>
#include <filesystem>
#include <memory>
#include <chrono>
#include <thread>
#include <ctime>

#include "rclcpp/rclcpp.hpp"
#include "msgs_ifaces/msg/chassis_imu.hpp"

using namespace std::chrono_literals;

class Node_ImuData : public rclcpp::Node {
public:
    Node_ImuData() : Node("node_imudata") {
        // QoS to match mROS2 publisher (sensor data)
        rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        qos_profile.best_effort();
        qos_profile.transient_local();

        subscription_ = this->create_subscription<msgs_ifaces::msg::ChassisIMU>(
            "/tp_imu_data_d5", qos_profile,
            std::bind(&Node_ImuData::imuCallback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Subscribed to /tp_imu_data_d5");

        std::string log_dir = "/home/curry/Almond/Datalog";
        if (!std::filesystem::exists(log_dir)) {
            try {
                std::filesystem::create_directories(log_dir);
                RCLCPP_INFO(this->get_logger(), "Created log directory: %s", log_dir.c_str());
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to create log directory: %s", e.what());
            }
        }

        auto now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);
        char datetime_buf[30];
        std::strftime(datetime_buf, sizeof(datetime_buf), "%Y-%m-%d_%H-%M-%S", std::localtime(&now_c));

        std::string filename = log_dir + "/imu_data_log_" + datetime_buf + ".csv";
        bool file_exists = std::filesystem::exists(filename);

        csv_file_.open(filename, std::ios::app);
        if (!csv_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", filename.c_str());
        } else if (!file_exists) {
            csv_file_ << "timestamp,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z\n";
        }
        // Timer to write CSV periodically
        timer_ = this->create_wall_timer(1s, std::bind(&Node_ImuData::writeToCSV, this));
    }

    ~Node_ImuData() {
        if (csv_file_.is_open()) {
            csv_file_.close();
        }
    }

private:
    rclcpp::Subscription<msgs_ifaces::msg::ChassisIMU>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    msgs_ifaces::msg::ChassisIMU latest_msg_;
    bool has_msg_ = false;
    std::ofstream csv_file_;

    void imuCallback(const msgs_ifaces::msg::ChassisIMU::SharedPtr msg) {
        latest_msg_ = *msg;
        has_msg_ = true;

        RCLCPP_INFO(this->get_logger(),
                    "Received -> Accel: X=%d Y=%d Z=%d | Gyro: X=%d Y=%d Z=%d",
                    msg->accel_x, msg->accel_y, msg->accel_z,
                    msg->gyro_x, msg->gyro_y, msg->gyro_z);
    }

    void writeToCSV() {
        if (has_msg_ && csv_file_.is_open()) {
            auto now = std::chrono::system_clock::now();
            std::time_t now_c = std::chrono::system_clock::to_time_t(now);
            char buf[100];
            std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", std::localtime(&now_c));

            const auto &chassis_imu = latest_msg_;
            csv_file_ << buf << ","
                      << chassis_imu.accel_x << ","
                      << chassis_imu.accel_y << ","
                      << chassis_imu.accel_z << ","
                      << chassis_imu.gyro_x << ","
                      << chassis_imu.gyro_y << ","
                      << chassis_imu.gyro_z << "\n";
            csv_file_.flush();
        }
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Node_ImuData>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

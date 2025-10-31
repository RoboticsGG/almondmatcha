#include <cstdio>
#include <fstream>
#include <filesystem>
#include <memory>
#include <chrono>
#include <thread>
#include <ctime>

#include "rclcpp/rclcpp.hpp"
#include "msgs_ifaces/msg/chassis_sensors.hpp"

using namespace std::chrono_literals;

class Node_SensData : public rclcpp::Node {
public:
    Node_SensData() : Node("node_sensdata") {
        // Set QoS to match the mROS2 publisher
        rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        qos_profile.best_effort();
        qos_profile.transient_local();

        subscription_ = this->create_subscription<msgs_ifaces::msg::ChassisSensors>(
            "/tp_sensdata_d5", qos_profile,
            std::bind(&Node_SensData::userCallback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Subscribed to /tp_sensdata_d5");

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

        std::string filename = log_dir + "/sensdata_log_" + datetime_buf + ".csv";
        bool file_exists = std::filesystem::exists(filename);

        csv_file_.open(filename, std::ios::app);
        if (!csv_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", filename.c_str());
        } else if (!file_exists) {
            csv_file_ << "timestamp,mt_lf_encode_msg,mt_rt_encode_msg,sys_current_msg,sys_volt_msg\n";
        }

        // Timer to write CSV periodically
        timer_ = this->create_wall_timer(1s, std::bind(&Node_SensData::writeToCSV, this));
    }

    ~Node_SensData() {
        if (csv_file_.is_open()) {
            csv_file_.close();
        }
    }

private:
    rclcpp::Subscription<msgs_ifaces::msg::ChassisSensors>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    msgs_ifaces::msg::ChassisSensors latest_msg_;
    bool has_msg_ = false;
    std::ofstream csv_file_;

    void userCallback(const msgs_ifaces::msg::ChassisSensors::SharedPtr msg) {
        latest_msg_ = *msg;
        has_msg_ = true;

        RCLCPP_INFO(this->get_logger(),
                    "Received -> LF:%d | RT:%d | I:%.2f | V:%.2f",
                    latest_msg_.mt_lf_encode_msg,
                    latest_msg_.mt_rt_encode_msg,
                    latest_msg_.sys_current_msg,
                    latest_msg_.sys_volt_msg);
    }

    void writeToCSV() {
        if (has_msg_ && csv_file_.is_open()) {
            //auto now = this->now();
            //double timestamp = now.seconds();
            auto now = std::chrono::system_clock::now();
            std::time_t now_c = std::chrono::system_clock::to_time_t(now);

            char buf[100];
            std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", std::localtime(&now_c));

            csv_file_ << buf << ","
                      << latest_msg_.mt_lf_encode_msg << ","
                      << latest_msg_.mt_rt_encode_msg << ","
                      << latest_msg_.sys_current_msg << ","
                      << latest_msg_.sys_volt_msg << "\n";
            csv_file_.flush();
        }
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Node_SensData>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

////////////// Debug Code //////////////

// #include <memory>
// #include "rclcpp/rclcpp.hpp"
// #include "msgs_ifaces/msg/main_sens_data.hpp"

// class SensDataSubscriber : public rclcpp::Node {
// public:
//     SensDataSubscriber() : Node("sensdata_sub_node") {

//         rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
//         qos_profile.best_effort();
//         qos_profile.transient_local();

//         subscription_ = this->create_subscription<msgs_ifaces::msg::MainSensData>(
//             "/tp_sensdata_d5", qos_profile,
//             std::bind(&SensDataSubscriber::topic_callback, this, std::placeholders::_1)
//         );
//         RCLCPP_INFO(this->get_logger(), "Subscribed to /tp_sensdata_d5");
//     }

// private:
//     void topic_callback(const msgs_ifaces::msg::MainSensData::SharedPtr msg) {
//         const auto & data = msg->mainsensdata_msg;

//         RCLCPP_INFO(this->get_logger(),
//                     "LF Encoder: %d | RT Encoder: %d | Current: %.3f A | Voltage: %.3f V",
//                     data.mt_lf_encode_msg,
//                     data.mt_rt_encode_msg,
//                     data.sys_current_msg,
//                     data.sys_volt_msg);
//     }

//     rclcpp::Subscription<msgs_ifaces::msg::MainSensData>::SharedPtr subscription_;
// };

// int main(int argc, char * argv[]) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<SensDataSubscriber>());
//     rclcpp::shutdown();
//     return 0;
// }

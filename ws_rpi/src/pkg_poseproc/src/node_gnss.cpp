#include "rclcpp/rclcpp.hpp"
#include "msgs_ifaces/msg/spresense_gnss.hpp"  
#include <json/json.h>
#include <fstream>
#include <sstream>
#include <chrono>
#include <ctime>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <rcpputils/filesystem_helper.hpp> 

using namespace std::chrono_literals;

class GNSSPublisher : public rclcpp::Node {
public:
    GNSSPublisher() : Node("gnss_publisher") {
        publisher_ = this->create_publisher<msgs_ifaces::msg::SpresenseGNSS>("gnss_data", 10);

        // Generate a unique filename using a timestamp
        log_dir_ = "/home/curry/Almond/Datalog";
        // if (!std::filesystem::exists(log_dir_)) {
        //     try {
        //         std::filesystem::create_directories(log_dir_);
        //         RCLCPP_INFO(this->get_logger(), "Created log directory: %s", log_dir_.c_str());
        //     } catch (const std::exception &e) {
        //         RCLCPP_ERROR(this->get_logger(), "Failed to create log directory: %s", e.what());
        //     }
        // }
         if (!rcpputils::fs::exists(log_dir_)) {
            try {
                rcpputils::fs::create_directories(log_dir_);
                RCLCPP_INFO(this->get_logger(), "Created log directory: %s", log_dir_.c_str());
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to create log directory: %s", e.what());
            }
        }

        // Generate a unique filename using a timestamp
        std::string filename = generateFileName();
        csv_file_.open(filename, std::ios::out);

        if (!csv_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file for writing!");
            return;
        }

        // Write CSV header
        csv_file_ << "Date,Time,NumSatellites,Fix,Latitude,Longitude\n";

        // Open Serial Port
        serial_port_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_port_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "Error opening serial port: /dev/ttyUSB0");
            return;
        }
        
        struct termios options;
        tcgetattr(serial_port_, &options);
        cfsetispeed(&options, B115200);
        cfsetospeed(&options, B115200);
        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        tcsetattr(serial_port_, TCSANOW, &options);

        // Timer to publish every 1 second
        timer_ = this->create_wall_timer(1s, std::bind(&GNSSPublisher::readSerialData, this));
    }

    ~GNSSPublisher() {
        if (serial_port_ != -1) {
            close(serial_port_);
        }
        if (csv_file_.is_open()) {
            csv_file_.close();
        }
    }

private:
    rclcpp::Publisher<msgs_ifaces::msg::SpresenseGNSS>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int serial_port_;
    std::ofstream csv_file_;
    std::string log_dir_;

    std::string generateFileName() {
        auto now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);
        std::tm *timeinfo = std::localtime(&now_c);

        char buffer[50];
        std::strftime(buffer, sizeof(buffer), "gnss_%Y%m%d_%H%M%S.csv", timeinfo);

        return log_dir_ + "/" + std::string(buffer);
    }

    void readSerialData() {
        if (serial_port_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "Serial port not open!");
            return;
        }
    
        char buffer[256];
        int bytes_read = read(serial_port_, buffer, sizeof(buffer) - 1);
    
        if (bytes_read > 0) {
            buffer[bytes_read] = '\0';  // Null-terminate string
            std::string json_data(buffer);
    
            // Trim whitespace and newlines
            json_data.erase(std::remove(json_data.begin(), json_data.end(), '\r'), json_data.end());
            json_data.erase(std::remove(json_data.begin(), json_data.end(), '\n'), json_data.end());
    
            Json::CharReaderBuilder reader;
            Json::Value root;
            std::string errs;
            std::istringstream ss(json_data);
    
            if (Json::parseFromStream(reader, ss, &root, &errs)) {
                std::string date_time = root["time"].asString();
                int numSatellites = root["numSatellites"].asInt();
                bool fix = root["fix"].asBool();
                double latitude = root["latitude"].asDouble();
                double longitude = root["longitude"].asDouble();
                double altitude = root["altitude"].asDouble();
    
                std::string date = date_time.substr(0, date_time.find(" "));
                std::string time = date_time.substr(date_time.find(" ") + 1);
    
                auto msg = msgs_ifaces::msg::SpresenseGNSS();
                msg.date = date;
                msg.time = time;
                msg.num_satellites = numSatellites;
                msg.fix = fix;
                msg.latitude = latitude;
                msg.longitude = longitude;
                msg.altitude = altitude;
    
                publisher_->publish(msg);
                RCLCPP_INFO(this->get_logger(), "Published Spresense GNSS: Date=%s, Time=%s, Sat=%d, Fix=%d, Lat=%f, Lon=%f, Alt=%f",
                            msg.date.c_str(), msg.time.c_str(), msg.num_satellites, msg.fix, msg.latitude, msg.longitude, msg.altitude);
    
                // Write data to CSV
                csv_file_ << date << "," << time << "," << numSatellites << "," << fix 
                          << "," << latitude << "," << longitude << "," << altitude << "\n";
                csv_file_.flush();
            } else {
                //RCLCPP_WARN(this->get_logger(), "Invalid JSON received!");
            }
        } else {
            //RCLCPP_WARN(this->get_logger(), "No data received from GNSS device.");
        }
    }    
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GNSSPublisher>());
    rclcpp::shutdown();
    return 0;
}

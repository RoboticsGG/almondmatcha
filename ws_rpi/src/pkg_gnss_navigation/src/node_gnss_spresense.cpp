#include "rclcpp/rclcpp.hpp"
#include "msgs_ifaces/msg/spresense_gnss.hpp"  
#include <json/json.h>
#include <sstream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

using namespace std::chrono_literals;

class SpresenseGNSSNode : public rclcpp::Node {
public:
    SpresenseGNSSNode() : Node("spresense_gnss_node") {
        // Use reliable + transient_local for GNSS data
        rclcpp::QoS qos_reliable(10);
        qos_reliable.reliable().transient_local();
        
        pub_gnss_spresense_ = this->create_publisher<msgs_ifaces::msg::SpresenseGNSS>("tpc_gnss_spresense", qos_reliable);

        RCLCPP_INFO(this->get_logger(), "CSV logging handled by node_rover_monitoring");

        serial_port_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_port_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "Error opening serial port: /dev/ttyUSB0");
            return;
        }
        
        configureSerialPort();

        timer_ = this->create_wall_timer(1s, std::bind(&SpresenseGNSSNode::readSerialData, this));
    }

    ~SpresenseGNSSNode() {
        if (serial_port_ != -1) {
            close(serial_port_);
        }
    }

private:
    rclcpp::Publisher<msgs_ifaces::msg::SpresenseGNSS>::SharedPtr pub_gnss_spresense_;
    rclcpp::TimerBase::SharedPtr timer_;
    int serial_port_;

    void configureSerialPort() {
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
    }

    void readSerialData() {
        if (serial_port_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "Serial port not open!");
            return;
        }
    
        char buffer[256];
        int bytes_read = read(serial_port_, buffer, sizeof(buffer) - 1);
    
        if (bytes_read <= 0) {
            return;
        }

        buffer[bytes_read] = '\0';
        std::string json_data(buffer);

        json_data.erase(std::remove(json_data.begin(), json_data.end(), '\r'), json_data.end());
        json_data.erase(std::remove(json_data.begin(), json_data.end(), '\n'), json_data.end());

        Json::CharReaderBuilder reader;
        Json::Value root;
        std::string errs;
        std::istringstream ss(json_data);

        if (!Json::parseFromStream(reader, ss, &root, &errs)) {
            return;
        }

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

        pub_gnss_spresense_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Spresense GNSS - Date=%s, Time=%s, Sats=%d, Fix=%d, Lat=%.6f, Lon=%.6f, Alt=%.2f",
                    msg.date.c_str(), msg.time.c_str(), msg.num_satellites, msg.fix, msg.latitude, msg.longitude, msg.altitude);
    }    
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpresenseGNSSNode>());
    rclcpp::shutdown();
    return 0;
}

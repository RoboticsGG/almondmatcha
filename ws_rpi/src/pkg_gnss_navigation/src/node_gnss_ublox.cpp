#include "rclcpp/rclcpp.hpp"
#include "msgs_ifaces/msg/ublox_gnss.hpp"
#include <sstream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cmath>

using namespace std::chrono_literals;

class UbloxGNSSNode : public rclcpp::Node {
public:
    UbloxGNSSNode() : Node("ublox_gnss_node") {
        rclcpp::QoS qos_reliable(10);
        qos_reliable.reliable().transient_local();
        
        pub_gnss_ublox_ = this->create_publisher<msgs_ifaces::msg::UbloxGNSS>("tpc_gnss_ublox", qos_reliable);

        RCLCPP_INFO(this->get_logger(), "CSV logging handled by node_rover_monitoring");

        serial_port_ = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_port_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "Error opening serial port: /dev/ttyACM0");
            return;
        }
        
        configureSerialPort();

        timer_ = this->create_wall_timer(100ms, std::bind(&UbloxGNSSNode::readSerialData, this));
    }

    ~UbloxGNSSNode() {
        if (serial_port_ != -1) {
            close(serial_port_);
        }
    }

private:
    rclcpp::Publisher<msgs_ifaces::msg::UbloxGNSS>::SharedPtr pub_gnss_ublox_;
    rclcpp::TimerBase::SharedPtr timer_;
    int serial_port_;
    std::string nmea_buffer_;

    void configureSerialPort() {
        struct termios options;
        tcgetattr(serial_port_, &options);
        cfsetispeed(&options, B460800);
        cfsetospeed(&options, B460800);
        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        options.c_oflag &= ~OPOST;
        tcsetattr(serial_port_, TCSANOW, &options);
    }

    void readSerialData() {
        if (serial_port_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "Serial port not open!");
            return;
        }
    
        char buffer[512];
        int bytes_read = read(serial_port_, buffer, sizeof(buffer) - 1);
    
        if (bytes_read <= 0) {
            return;
        }

        buffer[bytes_read] = '\0';
        nmea_buffer_ += std::string(buffer);

        // Process complete NMEA sentences
        size_t pos;
        while ((pos = nmea_buffer_.find('\n')) != std::string::npos) {
            std::string sentence = nmea_buffer_.substr(0, pos);
            nmea_buffer_ = nmea_buffer_.substr(pos + 1);
            
            // Remove \r if present
            if (!sentence.empty() && sentence.back() == '\r') {
                sentence.pop_back();
            }

            if (!sentence.empty() && sentence[0] == '$') {
                processNMEASentence(sentence);
            }
        }
    }

    void processNMEASentence(const std::string &sentence) {
        std::vector<std::string> tokens = splitString(sentence, ',');
        
        if (tokens.empty()) return;

        // GGA - Global Positioning System Fix Data
        if (tokens[0].find("GGA") != std::string::npos && tokens.size() >= 15) {
            parseGGA(tokens);
        }
        // GSA - GPS DOP and active satellites
        else if (tokens[0].find("GSA") != std::string::npos && tokens.size() >= 18) {
            parseGSA(tokens);
        }
        // GSV - Satellites in view (for SNR)
        else if (tokens[0].find("GSV") != std::string::npos) {
            parseGSV(tokens);
        }
        // RMC - Recommended Minimum
        else if (tokens[0].find("RMC") != std::string::npos && tokens.size() >= 12) {
            parseRMC(tokens);
        }
    }

    void parseGGA(const std::vector<std::string> &tokens) {
        // $GPGGA,time,lat,N/S,lon,E/W,quality,numSV,HDOP,alt,M,geoidal_sep,M,age,stnID
        
        if (tokens[2].empty() || tokens[4].empty()) return;

        current_data_.time = parseTime(tokens[1]);
        current_data_.latitude = parseLatitude(tokens[2], tokens[3]);
        current_data_.longitude = parseLongitude(tokens[4], tokens[5]);
        
        if (!tokens[6].empty()) {
            int quality = std::stoi(tokens[6]);
            current_data_.fix_quality = getFixQuality(quality);
        }
        
        if (!tokens[7].empty()) {
            current_data_.satellites_tracked = std::stoi(tokens[7]);
        }
        
        if (!tokens[9].empty()) {
            current_data_.altitude = std::stod(tokens[9]);
        }

        publishData();
    }

    void parseRMC(const std::vector<std::string> &tokens) {
        // $GPRMC,time,status,lat,N/S,lon,E/W,speed,course,date,mag_var,E/W,mode
        
        if (tokens.size() < 10) return;
        
        if (!tokens[9].empty()) {
            current_data_.date = parseDate(tokens[9]);
        }
        
        if (!tokens[7].empty()) {
            // Speed in knots, convert to m/s
            double speed_knots = std::stod(tokens[7]);
            current_data_.speed = speed_knots * 0.514444;  // knots to m/s
        }
    }

    void parseGSA(const std::vector<std::string> &tokens) {
        // $GPGSA,mode,fix_type,sat_ids...,PDOP,HDOP,VDOP
        // tokens[2] contains fix type: 1=no fix, 2=2D, 3=3D
        
        if (tokens.size() < 3) return;
        
        // Can use HDOP or PDOP to estimate error
        if (tokens.size() >= 16 && !tokens[15].empty()) {
            float hdop = std::stof(tokens[15]);
            // Rough conversion: HDOP * 5 meters = error in cm
            current_data_.centimeter_error = hdop * 500.0f;
        }
    }

    void parseGSV(const std::vector<std::string> &tokens) {
        // $GPGSV,numMsg,msgNum,numSV,sat_id,elev,azim,SNR,...
        // Extract SNR from visible satellites and average them
        
        if (tokens.size() < 8) return;
        
        float snr_sum = 0.0f;
        int snr_count = 0;
        
        // Each satellite has 4 fields: id, elevation, azimuth, SNR
        for (size_t i = 4; i < tokens.size(); i += 4) {
            if (i + 3 < tokens.size() && !tokens[i + 3].empty()) {
                try {
                    float snr = std::stof(tokens[i + 3]);
                    if (snr > 0) {
                        snr_sum += snr;
                        snr_count++;
                    }
                } catch (...) {}
            }
        }
        
        if (snr_count > 0) {
            current_data_.snr = snr_sum / snr_count;
        }
    }

    std::string parseTime(const std::string &time_str) {
        if (time_str.length() < 6) return "000000";
        return time_str.substr(0, 6);  // HHMMSS
    }

    std::string parseDate(const std::string &date_str) {
        if (date_str.length() < 6) return "20000101";
        // DDMMYY -> YYYYMMDD
        std::string day = date_str.substr(0, 2);
        std::string month = date_str.substr(2, 2);
        std::string year = "20" + date_str.substr(4, 2);
        return year + month + day;
    }

    double parseLatitude(const std::string &lat_str, const std::string &ns) {
        if (lat_str.empty()) return 0.0;
        
        // Format: DDMM.MMMM
        double lat = std::stod(lat_str);
        int degrees = static_cast<int>(lat / 100);
        double minutes = lat - (degrees * 100);
        double decimal_degrees = degrees + (minutes / 60.0);
        
        if (ns == "S") decimal_degrees = -decimal_degrees;
        
        return decimal_degrees;
    }

    double parseLongitude(const std::string &lon_str, const std::string &ew) {
        if (lon_str.empty()) return 0.0;
        
        // Format: DDDMM.MMMM
        double lon = std::stod(lon_str);
        int degrees = static_cast<int>(lon / 100);
        double minutes = lon - (degrees * 100);
        double decimal_degrees = degrees + (minutes / 60.0);
        
        if (ew == "W") decimal_degrees = -decimal_degrees;
        
        return decimal_degrees;
    }

    std::string getFixQuality(int quality) {
        switch (quality) {
            case 0: return "No Fix";
            case 1: return "Auto";
            case 2: return "DGPS";
            case 4: return "RTK Fixed";
            case 5: return "Float";
            default: return "Unknown";
        }
    }

    void publishData() {
        auto msg = msgs_ifaces::msg::UbloxGNSS();
        msg.date = current_data_.date;
        msg.time = current_data_.time;
        msg.latitude = current_data_.latitude;
        msg.longitude = current_data_.longitude;
        msg.altitude = current_data_.altitude;
        msg.satellites_tracked = current_data_.satellites_tracked;
        msg.fix_quality = current_data_.fix_quality;
        msg.snr = current_data_.snr;
        msg.speed = current_data_.speed;
        msg.centimeter_error = current_data_.centimeter_error;

        pub_gnss_ublox_->publish(msg);
        
        RCLCPP_INFO(this->get_logger(), 
            "u-blox RTK GNSS - Date=%s, Time=%s, Sats=%d, Fix=%s, Lat=%.6f, Lon=%.6f, Alt=%.2f, SNR=%.1f, Speed=%.2f m/s, Err=%.1f cm",
            msg.date.c_str(), msg.time.c_str(), msg.satellites_tracked, msg.fix_quality.c_str(),
            msg.latitude, msg.longitude, msg.altitude, 
            msg.snr, msg.speed, msg.centimeter_error);
    }

    std::vector<std::string> splitString(const std::string &str, char delimiter) {
        std::vector<std::string> tokens;
        std::stringstream ss(str);
        std::string token;
        while (std::getline(ss, token, delimiter)) {
            tokens.push_back(token);
        }
        return tokens;
    }

    struct GNSSData {
        std::string date = "20000101";
        std::string time = "000000";
        double latitude = 0.0;
        double longitude = 0.0;
        double altitude = 0.0;
        int32_t satellites_tracked = 0;
        std::string fix_quality = "No Fix";
        float snr = 0.0f;
        double speed = 0.0;
        float centimeter_error = 999.9f;
    };

    GNSSData current_data_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UbloxGNSSNode>());
    rclcpp::shutdown();
    return 0;
}

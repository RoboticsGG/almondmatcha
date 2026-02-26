# TelemetryRelay.msg
# Comprehensive telemetry aggregation from all Domain 5 topics
# Published by mission_monitoring_node_rpi
# Subscribed by mission_monitoring_node_pc

# Header with timestamp
std_msgs/Header header

# ===== Mission Status =====
bool mission_active
float64 distance_remaining_km

# ===== Spresense GNSS (Standard GPS) =====
bool spresense_valid
float32 spresense_latitude
float32 spresense_longitude
float32 spresense_altitude
int32 spresense_satellites

# ===== uBlox RTK GNSS (High Precision) =====
bool ublox_valid
float64 ublox_latitude
float64 ublox_longitude
float64 ublox_altitude
string ublox_fix_quality
float32 ublox_centimeter_error
int32 ublox_satellites

# ===== Chassis Command (sent to STM32) =====
bool chassis_cmd_valid
float32 chassis_cmd_left_speed
float32 chassis_cmd_right_speed
float32 chassis_cmd_left_direction
float32 chassis_cmd_right_direction

# ===== Chassis Sensors (from STM32) =====
bool chassis_sensors_valid
float32 encoder_left
float32 encoder_right
float32 voltage
float32 current
float32 power_watts

# ===== Chassis IMU (from STM32) =====
bool chassis_imu_valid
float32 accel_x
float32 accel_y
float32 accel_z
float32 gyro_x
float32 gyro_y
float32 gyro_z

# ===== Steering Control (from Jetson) =====
bool steering_valid
float32 steering_command

# ===== Lane Detection (from Jetson D6->D5 bridge) =====
bool lane_valid
float32 lane_theta
float32 lane_b
bool lane_detected

# ===== Destination Coordinate =====
bool destination_valid
float32 destination_latitude
float32 destination_longitude

# Chassis IMU (Inertial Measurement Unit) Data
# Contains accelerometer and gyroscope readings from the rover chassis

# Linear acceleration (m/s^2 * 1000 for integer representation)
int32 accel_x  # Acceleration along X-axis
int32 accel_y  # Acceleration along Y-axis
int32 accel_z  # Acceleration along Z-axis

# Angular velocity (rad/s * 1000 for integer representation)
int32 gyro_x   # Angular velocity around X-axis (roll rate)
int32 gyro_y   # Angular velocity around Y-axis (pitch rate)
int32 gyro_z   # Angular velocity around Z-axis (yaw rate)
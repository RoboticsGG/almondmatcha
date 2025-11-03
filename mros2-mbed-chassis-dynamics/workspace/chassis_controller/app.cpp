/* STM32 Nucleo-F767ZI - Multi-Task Motor Control + IMU Publisher
 * 
 * Architecture: 2-task design with Mbed OS threading
 *   Task 1 (motor_control_task): Subscribes to rover control commands, drives motors
 *   Task 2 (imu_reader_task):    Polls IMU sensor and publishes data
 *
 * Subscribes to: pub_rovercontrol (rover control commands)
 * Publishes to: tp_imu_data_d5 (IMU sensor data)
 * ROS Domain: 5
 */

#include "mbed.h"
#include "mros2.h"
#include "mros2-platform.h"
#include "plt_iks4a1.h"

#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "msgs_ifaces/msg/sub_rocon.hpp"
#include "msgs_ifaces/msg/main_rocon.hpp"
#include "msgs_ifaces/msg/sub_gyro_data.hpp"
#include "msgs_ifaces/msg/main_gyro_data.hpp"

#include <tuple>

/* =====================================
 * Sampling Rate Constants (Task Timing)
 * ===================================== */
const uint32_t MOTOR_RESPONSE_PERIOD_MS = 50;   // Motor control task polling rate (~20 Hz)
const uint32_t IMU_SAMPLE_PERIOD_MS = 10;      // IMU polling every 10ms (100 Hz capability)
const uint32_t IMU_PUBLISH_INTERVAL = 10;      // Publish every 10 samples = 10 Hz publish rate
const uint32_t MAIN_LOOP_PERIOD_MS = 100;      // Main loop safety tick

/* =====================================
 * Hardware PWM & Motor Control Pins
 * ===================================== */

// Function declarations
float calculate_steering_pwm_duty(uint8_t steering_direction, float steering_angle_degrees);
std::tuple<float, uint8_t, uint8_t> calculate_motor_direction(uint8_t motor_direction, uint8_t speed_percent);
void apply_motor_control(float steering_duty, uint8_t enable_forward, uint8_t enable_backward, uint8_t pwm_period_us_val, float motor_speed_percent);

// Steering servo (PA_3)
PwmOut steering_servo_pwm(PA_3);

// Motor driver PWM (left and right motors)
PwmOut motor_right_pwm(PA_6);
PwmOut motor_left_pwm(PE_11);

// Motor driver direction pins (H-bridge enable)
DigitalOut motor_right_enable_forward(PF_12);
DigitalOut motor_right_enable_backward(PD_15);
DigitalOut motor_left_enable_forward(PF_13);
DigitalOut motor_left_enable_backward(PE_9);

// Control parameters
uint8_t servo_center_angle = 100;  // Servo center position (degrees)
uint8_t pwm_period_us = 20;        // PWM period (microseconds)
uint8_t current_steering_angle = 0;  // Current steering angle

/* =====================================
 * IMU Sensor Setup
 * ===================================== */
LSM6DSV16X lsm6dsv16x(I2C_SDA, I2C_SCL);
DigitalOut led(LED1);

/* =====================================
 * Shared Data Structures (Thread-Safe)
 * ===================================== */

struct RoverCommandData {
    uint8_t front_direction;        // 0=straight, 1=left, 2=right
    float steering_angle;           // Steering angle in degrees
    uint8_t back_direction;         // 0=forward, 1=backward, 2=stop
    uint8_t motor_speed;            // Motor speed 0-100%
    bool command_updated;           // Flag: new command available
};

struct IMUData {
    int32_t accelerometer[3];
    int32_t gyro[3];
};

// Global shared data with Mutex protection
RoverCommandData rover_cmd = {0, 0.0f, 0, 0, false};
IMUData imu_data = {{0, 0, 0}, {0, 0, 0}};

Mutex rover_cmd_mutex;
Mutex imu_data_mutex;

// Global publisher pointer for IMU task
mros2::Publisher* imu_pub_ptr = NULL;

/* =====================================
 * Motor Control Functions
 * ===================================== */

float calculate_steering_pwm_duty(uint8_t steering_direction, float steering_angle_degrees)
{
    /**
     * Calculate steering servo PWM duty cycle from steering angle
     * steering_direction: 0=straight, 1=left, 2=right (unused in current implementation)
     * steering_angle_degrees: steering angle in degrees relative to center
     * Returns: PWM duty cycle percentage (5%-10% range for standard servo)
     */
    
    // Servo range: typically 90° to 110° maps to 5% to 10% duty cycle (1.0ms to 2.0ms on 20ms period)
    // Center position = 100° = 7.5% duty cycle (1.5ms)
    
    float servo_min_angle = 90.0f;
    float servo_max_angle = 110.0f;
    float servo_min_duty = 5.0f;
    float servo_max_duty = 10.0f;
    
    float target_angle = servo_center_angle + steering_angle_degrees;
    
    // Clamp to servo range
    if (target_angle < servo_min_angle) target_angle = servo_min_angle;
    if (target_angle > servo_max_angle) target_angle = servo_max_angle;
    
    // Map angle to duty cycle: (angle - min_angle) / (max_angle - min_angle) * (max_duty - min_duty) + min_duty
    float duty_cycle = ((target_angle - servo_min_angle) / (servo_max_angle - servo_min_angle)) * 
                       (servo_max_duty - servo_min_duty) + servo_min_duty;
    
    return duty_cycle;
}

std::tuple<float, uint8_t, uint8_t> calculate_motor_direction(uint8_t motor_direction, uint8_t speed_percent)
{
    /**
     * Determine motor direction and apply speed from direction flag
     * motor_direction: 0=forward, 1=backward, 2=stop
     * speed_percent: motor speed 0-100%
     * Returns: (motor_duty, enable_forward, enable_backward) tuple
     */
    
    uint8_t enable_forward = 0, enable_backward = 0;
    float motor_duty = (float)speed_percent / 100.0f;
    
    if (motor_direction == 0) {
        // Forward: enable_forward=1, enable_backward=0
        enable_forward = 1;
        enable_backward = 0;
    } else if (motor_direction == 1) {
        // Backward: enable_forward=0, enable_backward=1
        enable_forward = 0;
        enable_backward = 1;
    } else {
        // Stop: enable_forward=0, enable_backward=0
        enable_forward = 0;
        enable_backward = 0;
        motor_duty = 0.0f;
    }
    
    return std::make_tuple(motor_duty, enable_forward, enable_backward);
}

void apply_motor_control(float steering_duty, uint8_t enable_forward, uint8_t enable_backward, uint8_t pwm_period_us_val, float motor_speed_percent)
{
    /**
     * Apply PWM and direction signals to motor hardware
     * steering_duty: normalized duty cycle (0.0 to 1.0)
     * enable_forward, enable_backward: direction pins for H-bridge
     * pwm_period_us_val: PWM period in microseconds
     * motor_speed_percent: duty cycle as percentage (0-100)
     */
    
    // Configure PWM period (20 us = 50 kHz)
    steering_servo_pwm.period_us(pwm_period_us_val);
    motor_right_pwm.period_us(pwm_period_us_val);
    motor_left_pwm.period_us(pwm_period_us_val);
    
    // Set steering servo PWM (steering_servo_pwm)
    steering_servo_pwm.write(steering_duty);
    
    // Set motor direction pins
    motor_right_enable_forward = enable_forward;
    motor_right_enable_backward = enable_backward;
    motor_left_enable_forward = enable_forward;
    motor_left_enable_backward = enable_backward;
    
    // Set motor speed (duty cycle)
    float normalized_duty = motor_speed_percent / 100.0f;
    motor_right_pwm.write(normalized_duty);
    motor_left_pwm.write(normalized_duty);
}

/* =====================================
 * Task 1: Motor Control Subscriber
 * ===================================== */

void motor_control_task() {
    MROS2_INFO("[motor_control_task] Started");
    
    while (1) {
        // Check if new command available (with mutex protection)
        {
            rover_cmd_mutex.lock();
            if (rover_cmd.command_updated) {
                // Extract command with local copies to minimize lock time
                uint8_t front_dir = rover_cmd.front_direction;
                float steering_ang = rover_cmd.steering_angle;
                uint8_t back_dir = rover_cmd.back_direction;
                uint8_t motor_spd = rover_cmd.motor_speed;
                rover_cmd.command_updated = false;
                rover_cmd_mutex.unlock();
                
                // Calculate servo PWM from steering angle
                float servo_duty = calculate_steering_pwm_duty(front_dir, steering_ang);
                
                // Calculate motor direction and duty from speed
                std::tuple<float, uint8_t, uint8_t> motor_ctrl = calculate_motor_direction(back_dir, motor_spd);
                uint8_t enable_fwd = std::get<1>(motor_ctrl);
                uint8_t enable_bwd = std::get<2>(motor_ctrl);
                
                // Apply motor control signals
                apply_motor_control(servo_duty, enable_fwd, enable_bwd, pwm_period_us, motor_spd);
                
                MROS2_DEBUG("Motor: front_dir=%d angle=%f back_dir=%d speed=%d", 
                           front_dir, steering_ang, back_dir, motor_spd);
            } else {
                rover_cmd_mutex.unlock();
            }
        }
        
        // Polling rate for motor control
        ThisThread::sleep_for(chrono::milliseconds(MOTOR_RESPONSE_PERIOD_MS));
    }
}

/* =====================================
 * Task 2: IMU Reader & Publisher
 * ===================================== */

void imu_reader_task() {
    MROS2_INFO("[imu_reader_task] Started");
    
    if (imu_pub_ptr == NULL) {
        MROS2_ERROR("[imu_reader_task] Publisher not initialized!");
        return;
    }
    
    // IMU variables for local buffering
    int32_t local_accel[3] = {0};
    int32_t local_gyro[3] = {0};
    uint32_t sample_count = 0;
    
    while (1) {
        // Read IMU sensor data
        lsm6dsv16x.Get_X_Axes(local_accel);
        lsm6dsv16x.Get_G_Axes(local_gyro);
        
        // Update shared IMU data (with mutex protection)
        {
            imu_data_mutex.lock();
            imu_data.accelerometer[0] = local_accel[0];
            imu_data.accelerometer[1] = local_accel[1];
            imu_data.accelerometer[2] = local_accel[2];
            imu_data.gyro[0] = local_gyro[0];
            imu_data.gyro[1] = local_gyro[1];
            imu_data.gyro[2] = local_gyro[2];
            imu_data_mutex.unlock();
        }
        
        // Publish IMU data at reduced rate (every IMU_PUBLISH_INTERVAL samples)
        if ((sample_count++ % IMU_PUBLISH_INTERVAL) == 0) {
            // Create ROS2 message
            msgs_ifaces::msg::SubGyroData sub_msg;
            sub_msg.accel_x = local_accel[0];
            sub_msg.accel_y = local_accel[1];
            sub_msg.accel_z = local_accel[2];
            sub_msg.gyro_x  = local_gyro[0];
            sub_msg.gyro_y  = local_gyro[1];
            sub_msg.gyro_z  = local_gyro[2];
            
            msgs_ifaces::msg::MainGyroData main_msg;
            main_msg.maingyrodata_msg = sub_msg;
            
            // Publish to ROS2
            imu_pub_ptr->publish(main_msg);
            
            MROS2_INFO("[imu_reader_task] Accel: X=%ld Y=%ld Z=%ld | Gyro: X=%ld Y=%ld Z=%ld",
                       sub_msg.accel_x, sub_msg.accel_y, sub_msg.accel_z,
                       sub_msg.gyro_x, sub_msg.gyro_y, sub_msg.gyro_z);
        }
        
        // IMU polling rate
        ThisThread::sleep_for(chrono::milliseconds(IMU_SAMPLE_PERIOD_MS));
    }
}

/* =====================================
 * ROS2 Subscription Callback
 * ===================================== */

void rover_control_callback(msgs_ifaces::msg::MainRocon* msg) {
    /**
     * Callback handler for rover control messages
     * Stores command in shared rover_cmd struct for motor_control_task to process
     */
    
    rover_cmd_mutex.lock();
    rover_cmd.front_direction = msg->mainrocon_msg.fdr_msg;
    rover_cmd.steering_angle = msg->mainrocon_msg.ro_ctrl_msg;
    rover_cmd.back_direction = msg->mainrocon_msg.bdr_msg;
    rover_cmd.motor_speed = msg->mainrocon_msg.spd_msg;
    rover_cmd.command_updated = true;
    rover_cmd_mutex.unlock();
    
    MROS2_DEBUG("ROS2 CB: front=%d angle=%f back=%d speed=%d",
               msg->mainrocon_msg.fdr_msg,
               msg->mainrocon_msg.ro_ctrl_msg,
               msg->mainrocon_msg.bdr_msg,
               msg->mainrocon_msg.spd_msg);
}

/* =====================================
 * Main Function
 * ===================================== */

int main()
{
    // Set ROS domain ID (Domain 5)
    setenv("ROS_DOMAIN_ID", "5", 5);

    // Connect to network
    if (mros2_platform::network_connect()) {
        MROS2_ERROR("Failed to connect network! Aborting...");
        return -1;
    }
    MROS2_INFO("Network connected successfully");

    MROS2_INFO("================================");
    MROS2_INFO("Platform: %s", MROS2_PLATFORM_NAME);
    MROS2_INFO("Node: RoverWithIMU (Domain 5)");
    MROS2_INFO("Architecture: 2-Task (Motor Control + IMU Reader)");
    MROS2_INFO("================================");
    
    // Initialize ROS2
    mros2::init(0, NULL);
    mros2::Node node = mros2::Node::create_node("rover_node");
    
    // Create subscriber for rover control commands
    mros2::Subscriber sub = node.create_subscription<msgs_ifaces::msg::MainRocon>(
        "pub_rovercontrol", 10, rover_control_callback);
    
    // Create publisher for IMU data
    mros2::Publisher pub = node.create_publisher<msgs_ifaces::msg::MainGyroData>(
        "tp_imu_data_d5", 10);
    
    // Store publisher pointer for IMU task
    imu_pub_ptr = &pub;
    
    MROS2_INFO("ROS2 Node initialized - Ready to publish/subscribe");
    
    // Initialize IMU sensor
    lsm6dsv16x.begin();
    lsm6dsv16x.Enable_X();  // Enable accelerometer
    lsm6dsv16x.Enable_G();  // Enable gyroscope
    
    uint8_t sensor_id;
    lsm6dsv16x.ReadID(&sensor_id);
    MROS2_INFO("LSM6DSV16X Sensor ID: 0x%02X", sensor_id);
    
    // Launch Task 1: Motor Control (High Priority)
    Thread motor_thread(osPriorityHigh, 2048, NULL, "motor_ctrl");
    motor_thread.start(callback(motor_control_task));
    MROS2_INFO("Motor Control Task started (osPriorityHigh)");
    
    // Launch Task 2: IMU Reader (Normal Priority)
    Thread imu_thread(osPriorityNormal, 2048, NULL, "imu_read");
    imu_thread.start(callback(imu_reader_task));
    MROS2_INFO("IMU Reader Task started (osPriorityNormal)");
    
    // Wait for all tasks and hardware to initialize
    osDelay(1000);
    MROS2_INFO("All initialization complete - ready to operate");
    
    MROS2_INFO("All tasks launched - entering ROS2 spin loop");
    MROS2_INFO("Sampling rates: Motor=%ldms, IMU=%ldms (publish every %ld samples)",
               MOTOR_RESPONSE_PERIOD_MS, IMU_SAMPLE_PERIOD_MS, IMU_PUBLISH_INTERVAL);
    
    // Main loop: Spin ROS2 communication
    mros2::spin();
    
    return 0;
}

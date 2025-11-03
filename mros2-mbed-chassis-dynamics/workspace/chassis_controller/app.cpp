/* STM32 Nucleo-F767ZI - Motor Control + IMU Publisher
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

// ============================================================================
// HARDWARE PINS & CONTROL FUNCTIONS
// ============================================================================

// Function declarations
float frontControl(uint8_t frontDirection, float diff_degree);
std::tuple<float, uint8_t, uint8_t> backControl(uint8_t backDirection, uint8_t dutycycle_PWM);
void motorDrive(float duty, uint8_t EN_A, uint8_t EN_B, uint8_t period_PWM, float percent_dutycycle);

// Steering servo (PA_3)
PwmOut DirectPWM(PA_3);

// Motor driver PWM (left and right motors)
PwmOut MortorRPWM(PA_6);
PwmOut MortorLPWM(PE_11);

// Motor driver direction pins (H-bridge enable)
DigitalOut MortorRAEN(PF_12);
DigitalOut MortorRBEN(PD_15);
DigitalOut MortorLAEN(PF_13);
DigitalOut MortorLBEN(PE_9);

// Control parameters
uint8_t servo_center = 100;  // Servo center position (degrees)
uint8_t period_PWM = 20;     // PWM period (microseconds)
uint8_t degree = 0;          // Current steering angle

// ============================================================================
// IMU SENSOR
// ============================================================================

LSM6DSV16X lsm6dsv16x(I2C_SDA, I2C_SCL);
DigitalOut led(LED1);

int32_t accelerometer[3], anglerate[3];

// ============================================================================
// ROVER CONTROL CALLBACK
// ============================================================================

void userCallback(msgs_ifaces::msg::MainRocon *msg)
{
    MROS2_INFO("Received rover control: FrontDir=%d Angle=%.2f Speed=%d BackDir=%d",
        msg->mainrocon_msg.fdr_msg, msg->mainrocon_msg.ro_ctrl_msg,
        msg->mainrocon_msg.spd_msg, msg->mainrocon_msg.bdr_msg);

    // Calculate steering and motor control
    float steering_duty = frontControl(msg->mainrocon_msg.fdr_msg, msg->mainrocon_msg.ro_ctrl_msg);
    auto [motor_speed, EN_A, EN_B] = backControl(msg->mainrocon_msg.bdr_msg, msg->mainrocon_msg.spd_msg);
    
    // Apply to hardware
    motorDrive(steering_duty, EN_A, EN_B, period_PWM, motor_speed);
}

// ============================================================================
// CONTROL FUNCTIONS
// ============================================================================

float frontControl(uint8_t frontDirection, float diff_degree)
{
    if (frontDirection == 1) {
        degree = servo_center - diff_degree;  // Turn left
    } else if (frontDirection == 3) {
        degree = servo_center + diff_degree;  // Turn right
    } else {
        degree = servo_center;  // Straight
    }
    
    // Convert angle to PWM duty cycle (5%-10% for standard servo)
    return 0.05f + (degree / 180.0f) * (0.10f - 0.05f);
}

std::tuple<float, uint8_t, uint8_t> backControl(uint8_t backDirection, uint8_t dutycycle_PWM)
{
    uint8_t EN_A = 0;
    uint8_t EN_B = 0;
    float motor_speed = dutycycle_PWM / 100.0f;  // Convert percentage to decimal

    if (backDirection == 1) {
        EN_A = 1; EN_B = 0;  // Forward
    } else if (backDirection == 2) {
        EN_A = 0; EN_B = 1;  // Reverse
    } else {
        EN_A = 0; EN_B = 0;  // Stop
    }
    
    return {motor_speed, EN_A, EN_B};
}

void motorDrive(float duty, uint8_t EN_A, uint8_t EN_B, uint8_t period_PWM, float percent_dutycycle)
{
    // Clamp steering duty cycle
    duty = std::max(0.0f, std::min(1.0f, duty));
    
    // Set steering servo
    DirectPWM.period_ms(20);
    DirectPWM.write(duty);

    // Set motor direction (left and right motors)
    MortorLAEN.write(EN_A);
    MortorLBEN.write(EN_B);
    MortorRAEN.write(EN_B);  // Right motor reversed
    MortorRBEN.write(EN_A);

    // Set motor speed
    MortorRPWM.period_us(period_PWM);
    MortorRPWM.write(percent_dutycycle);
    MortorLPWM.period_us(period_PWM);
    MortorLPWM.write(percent_dutycycle);
}

// ============================================================================
// MAIN FUNCTION
// ============================================================================
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

    MROS2_INFO("Platform: %s", MROS2_PLATFORM_NAME);
    MROS2_INFO("Node: RoverWithIMU (Domain 5)");

    // Initialize ROS2 node
    mros2::init(0, NULL);
    mros2::Node node = mros2::Node::create_node("rover_node");

    // Create subscriber for rover control commands
    mros2::Subscriber sub = node.create_subscription<msgs_ifaces::msg::MainRocon>(
        "pub_rovercontrol", 10, userCallback);

    // Create publisher for IMU data
    mros2::Publisher pub = node.create_publisher<msgs_ifaces::msg::MainGyroData>(
        "tp_imu_data_d5", 10);

    MROS2_INFO("Ready to publish/subscribe");
    
    // Initialize IMU sensor
    lsm6dsv16x.begin();
    lsm6dsv16x.Enable_X();  // Enable accelerometer
    lsm6dsv16x.Enable_G();  // Enable gyroscope

    uint8_t sensor_id;
    lsm6dsv16x.ReadID(&sensor_id);
    MROS2_INFO("LSM6DSV16X Sensor ID: 0x%02X", sensor_id);

    int count = 0;

    // Main loop: read IMU and publish data
    while (1) {
        // Motor control handled by subscriber callback
        
        // Read IMU sensor data
        lsm6dsv16x.Get_X_Axes(accelerometer);
        lsm6dsv16x.Get_G_Axes(anglerate);

        // Populate IMU message
        msgs_ifaces::msg::SubGyroData sub_msg;
        sub_msg.accel_x = accelerometer[0];
        sub_msg.accel_y = accelerometer[1];
        sub_msg.accel_z = accelerometer[2];
        sub_msg.gyro_x  = anglerate[0];
        sub_msg.gyro_y  = anglerate[1];
        sub_msg.gyro_z  = anglerate[2];

        msgs_ifaces::msg::MainGyroData main_msg;
        main_msg.maingyrodata_msg = sub_msg;

        // Publish IMU data
        MROS2_INFO("IMU [%d] Accel: X=%d Y=%d Z=%d | Gyro: X=%d Y=%d Z=%d",
                   count++,
                   sub_msg.accel_x, sub_msg.accel_y, sub_msg.accel_z,
                   sub_msg.gyro_x, sub_msg.gyro_y, sub_msg.gyro_z);

        pub.publish(main_msg);
        
        // Note: Add delay here if needed to control publish rate
        // osDelay(100);  // 10 Hz
        // ThisThread::sleep_for(200ms);  // 5 Hz
    }

    mros2::spin();
  return 0;
}

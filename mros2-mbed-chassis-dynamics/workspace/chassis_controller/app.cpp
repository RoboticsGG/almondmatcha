/* STM32 Nucleo-F767ZI - Multi-Task Motor Control + IMU Publisher
 * 
 * Architecture: 2-task design with Mbed OS threading
 *   Task 1 (motor_control_task): Subscribes to rover control commands, drives motors
 *   Task 2 (imu_reader_task):    Polls IMU sensor and publishes data
 *
 * Subscribes to: tpc_chassis_cmd (rover control commands)
 * Publishes to: tpc_chassis_imu (IMU sensor data)
 * ROS Domain: 5
 *
 * Organization:
 *   - motor_control.{h,cpp}: Motor PWM and steering servo control
 *   - led_status.{h,cpp}:    Board status LED indication
 *   - app.cpp:               Main orchestration and ROS2 integration
 */

#include "mbed.h"
#include "mros2.h"
#include "mros2-platform.h"
#include "plt_iks4a1.h"

#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "msgs_ifaces/msg/chassis_ctrl.hpp"
#include "msgs_ifaces/msg/chassis_imu.hpp"

#include "motor_control.h"
#include "led_status.h"
#include "memory_reporter.h"  // single-domain POC: periodic heap/stack JSON to USB serial

// Build timestamp baked in at compile time
#define FW_BUILD_DATE __DATE__
#define FW_BUILD_TIME __TIME__
#define FW_BOARD_NAME "chassis-dynamics"
#define FW_BOARD_IP   "192.168.1.2"

#include <tuple>

/* =====================================
 * Sampling Rate Constants (Task Timing)
 * ===================================== */
const uint32_t IMU_SAMPLE_PERIOD_MS = 10;      // IMU polling every 10ms (100 Hz capability)
const uint32_t IMU_PUBLISH_INTERVAL = 10;      // Publish every 10 samples = 10 Hz publish rate
const uint32_t MAIN_LOOP_PERIOD_MS = 100;      // Main loop safety tick

/* =====================================
 * IMU Sensor & Shared Data
 * ===================================== */

LSM6DSV16X lsm6dsv16x(I2C_SDA, I2C_SCL);

/// IMU sensor data structure (separate from motor commands)
struct IMUData {
    int32_t accelerometer[3];
    int32_t gyro[3];
};

// Global IMU data with Mutex protection
IMUData imu_data = {{0, 0, 0}, {0, 0, 0}};
Mutex imu_data_mutex;

// Global publisher pointer for IMU task
mros2::Publisher* imu_pub_ptr = NULL;

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

    // ===== MULTICAST DISCOVERY WAIT =====
    // mros2::spin() is already running so the RTPS stack processes incoming
    // SPDP multicast packets from FastDDS on 239.255.0.1 (port 8650 for domain 5).
    // embeddedRTPS only supports multicast discovery — no unicast initial peers.
    // 3 s is enough for bidirectional SPDP+SEDP exchange at 500 ms SPDP period.
    MROS2_INFO("[imu_reader_task] Waiting 3 s for multicast SPDP exchange...");
    ThisThread::sleep_for(chrono::milliseconds(3000));
    MROS2_INFO("[imu_reader_task] Discovery wait done — starting IMU publish");
    // ==================================

    // IMU variables for local buffering
    int32_t local_accel[3] = {0};
    int32_t local_gyro[3] = {0};
    uint32_t sample_count = 0;
    bool mem_reporter_started = false;

    // Uptime timer for heartbeat line timestamps.
    // Printed every IMU publish cycle (1 Hz) so the raw serial log has
    // human-readable status interleaved with the 1 Hz STM32_STATS JSON.
    // The serial collector ignores [HB#...] lines — only STM32_STATS is saved.
    Timer hb_timer;
    hb_timer.start();

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
            msgs_ifaces::msg::ChassisIMU imu_msg;
            imu_msg.accel_x = local_accel[0];
            imu_msg.accel_y = local_accel[1];
            imu_msg.accel_z = local_accel[2];
            imu_msg.gyro_x  = local_gyro[0];
            imu_msg.gyro_y  = local_gyro[1];
            imu_msg.gyro_z  = local_gyro[2];
            
            // Publish to ROS2
            imu_pub_ptr->publish(imu_msg);

            // Start memory reporter after first publish so boot output is clean
            if (!mem_reporter_started) {
                mem_reporter_started = true;
                memory_reporter_start("chassis");
                MROS2_INFO("Memory reporter started (1s interval)");
            }

            // Heartbeat: human-readable status line at each IMU publish (1 Hz).
            // Format: [HB#<uptime_ms>] IMU Accel(X:.. Y:.. Z:..) Gyro(X:.. Y:.. Z:..) Cmd(spd:.. steer:..)
            // The serial collector ignores [HB#...] lines — only STM32_STATS JSON
            // is captured to CSV; heartbeats go to the raw serial log only.
            int hb_ts = (int)chrono::duration_cast<chrono::milliseconds>(
                                 hb_timer.elapsed_time()).count();
            // Snapshot latest motor command (best effort, non-blocking)
            rover_cmd_mutex.lock();
            uint8_t cmd_spd   = rover_cmd.motor_speed;
            float   cmd_steer = rover_cmd.steering_angle;
            rover_cmd_mutex.unlock();
            mr_serial_lock();
            printf("\r\n[HB#%d] IMU Accel(X:%ld Y:%ld Z:%ld) Gyro(X:%ld Y:%ld Z:%ld) Cmd(spd:%u steer:%.1f)",
                   hb_ts,
                   (long)imu_msg.accel_x, (long)imu_msg.accel_y, (long)imu_msg.accel_z,
                   (long)imu_msg.gyro_x,  (long)imu_msg.gyro_y,  (long)imu_msg.gyro_z,
                   (unsigned)cmd_spd, (double)cmd_steer);
            fflush(stdout);
            mr_serial_unlock();
        }
        
        // IMU polling rate
        ThisThread::sleep_for(chrono::milliseconds(IMU_SAMPLE_PERIOD_MS));
    }
}

/* =====================================
 * ROS2 Subscription Callback
 * ===================================== */

void rover_control_callback(msgs_ifaces::msg::ChassisCtrl* msg) {
    /**
     * Callback handler for rover control messages
     * Stores command in shared rover_cmd struct for motor_control_task to process
     */
    
    rover_cmd_mutex.lock();
    rover_cmd.front_direction = msg->fdr_msg;
    rover_cmd.steering_angle = msg->ro_ctrl_msg;
    rover_cmd.back_direction = msg->bdr_msg;
    rover_cmd.motor_speed = msg->spd_msg;
    rover_cmd.command_updated = true;
    rover_cmd_mutex.unlock();
    
    MROS2_DEBUG("\nROS2 CB: front=%d angle=%f back=%d speed=%d",
               msg->fdr_msg,
               msg->ro_ctrl_msg,
               msg->bdr_msg,
               msg->spd_msg);
}

/* =====================================
 * Main Function
 * ===================================== */

int main()
{
    // Set ROS domain ID (Domain 5)
    setenv("ROS_DOMAIN_ID", "5", 5);

    // Initialize status LED (off during initialization)
    set_status_led(false);
    MROS2_INFO("Status LED initialized (off during init)");

    // Connect to network
    if (mros2_platform::network_connect()) {
        MROS2_ERROR("Failed to connect network! Aborting...");
        // Leave LED off to indicate initialization failure
        return -1;
    }
    MROS2_INFO("Network connected successfully");

    MROS2_INFO("========================================");
    MROS2_INFO("  Firmware: %s", FW_BOARD_NAME);
    MROS2_INFO("  Built:    %s %s", FW_BUILD_DATE, FW_BUILD_TIME);
    MROS2_INFO("  Board IP: %s", FW_BOARD_IP);
    MROS2_INFO("  Platform: %s", MROS2_PLATFORM_NAME);
    MROS2_INFO("  Domain:   %d", rtps::Config::DOMAIN_ID);
    MROS2_INFO("  SPDP max: %d  HB: %d ms  SPDP: %d ms",
               rtps::Config::SPDP_MAX_NUMBER_FOUND_PARTICIPANTS,
               rtps::Config::SF_WRITER_HB_PERIOD_MS,
               rtps::Config::SPDP_RESEND_PERIOD_MS);
    MROS2_INFO("  Node: RoverWithIMU");
    MROS2_INFO("  Pub:  tpc_chassis_imu (10 Hz)");
    MROS2_INFO("  Sub:  tpc_chassis_cmd");
    MROS2_INFO("========================================");
    
    // Initialize ROS2
    mros2::init(0, NULL);
    mros2::Node node = mros2::Node::create_node("rover_node");
    
    // Create subscriber for rover control commands
    mros2::Subscriber sub = node.create_subscription<msgs_ifaces::msg::ChassisCtrl>(
        "tpc_chassis_cmd", 10, rover_control_callback);
    
    // Create publisher for IMU data
    mros2::Publisher pub = node.create_publisher<msgs_ifaces::msg::ChassisIMU>(
        "tpc_chassis_imu", 10);
    
    // Store publisher pointer for IMU task
    imu_pub_ptr = &pub;
    
    MROS2_INFO("ROS2 Node initialized - Ready to publish/subscribe");

    // ===== DISCOVERY NOTE =====
    // mros2::spin() is called immediately below — the RTPS stack starts
    // processing multicast SPDP on 239.255.0.1 as soon as spin runs.
    // embeddedRTPS only supports multicast discovery (no unicast initial peers).
    // imu_reader_task() contains a 3 s internal wait before publishing.
    // No osDelay() here: blocking here would silence mros2 and delay
    // multicast SPDP exchange.
    // ==========================

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
    // 4096 B required: mros2 publish() traverses CDR serialization → embeddedRTPS → lwIP UDP
    // send, which consumes ~1.5–2 KB of call stack on top of local variables and printf.
    // 2048 B caused a stack overflow → NULL-ptr HardFault (UFSR INVSTATE, PC=0x00000000).
    Thread imu_thread(osPriorityNormal, 4096, NULL, "imu_read");
    imu_thread.start(callback(imu_reader_task));
    MROS2_INFO("IMU Reader Task started (osPriorityNormal)");
    
    // Wait for all tasks and hardware to initialize
    osDelay(1000);
    MROS2_INFO("All initialization complete - ready to operate");
    
    // Turn status LED on solid to indicate ready state
    set_status_led(true);
    MROS2_INFO("Status LED turned on solid (ready to operate)");
    
    MROS2_INFO("All tasks launched - entering ROS2 spin loop");
    MROS2_INFO("Sampling rates: Motor=%ldms, IMU=%ldms (publish every %ld samples)",
               MOTOR_RESPONSE_PERIOD_MS, IMU_SAMPLE_PERIOD_MS, IMU_PUBLISH_INTERVAL);

    // Main loop: Spin ROS2 communication
    // Memory reporter is started inside imu_reader_task() after first publish,
    // so boot output appears before the JSON stats stream.
    mros2::spin();
    
    return 0;
}

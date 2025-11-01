#include "mbed.h"
#include "plt_iks4a1.h"

#include "mros2.h"
#include "mros2-platform.h"
#include "msgs_ifaces/msg/sub_gyro_data.hpp"
#include "msgs_ifaces/msg/main_gyro_data.hpp"

// IMU driver
LSM6DSV16X lsm6dsv16x(I2C_SDA, I2C_SCL);
DigitalOut led(LED1);

// Buffers
int32_t accelerometer[3], anglerate[3];

// LED heartbeat
void blinkLed() {
    led = !led;
}

int main() {
    // Connect to network
    if (mros2_platform::network_connect()) {
        MROS2_ERROR("failed to connect and setup network! aborting...");
        return -1;
    } else {
        MROS2_INFO("successfully connected to network\r\n---");
    }

    MROS2_INFO("%s start!", MROS2_PLATFORM_NAME);
    MROS2_INFO("app name: imu_publisher");

    // Initialize ROS2 node
    mros2::init(0, NULL);
    mros2::Node node = mros2::Node::create_node("imu_node");

    // Create publisher (MainGyroData)
    mros2::Publisher pub = node.create_publisher<msgs_ifaces::msg::MainGyroData>("tp_imu_data_d5", 10);

    osDelay(500);
    MROS2_INFO("ready to publish messages\r\n---");

    // Init IMU
    lsm6dsv16x.begin();
    lsm6dsv16x.Enable_X();
    lsm6dsv16x.Enable_G();

    uint8_t id;
    lsm6dsv16x.ReadID(&id);
    MROS2_INFO("LSM6DSV16X ID = 0x%02X", id);

    int count = 0;

    while (1) {
        // Read IMU values
        lsm6dsv16x.Get_X_Axes(accelerometer);
        lsm6dsv16x.Get_G_Axes(anglerate);

        // Fill SubGyroData
        msgs_ifaces::msg::SubGyroData sub_msg;
        sub_msg.accel_x = accelerometer[0];
        sub_msg.accel_y = accelerometer[1];
        sub_msg.accel_z = accelerometer[2];
        sub_msg.gyro_x  = anglerate[0];
        sub_msg.gyro_y  = anglerate[1];
        sub_msg.gyro_z  = anglerate[2];

        
        msgs_ifaces::msg::MainGyroData main_msg;

        // Publish
        MROS2_INFO("publishing accel: X=%d Y=%d Z=%d | gyro: X=%d Y=%d Z=%d | Count=%d",
                   sub_msg.accel_x, sub_msg.accel_y, sub_msg.accel_z,
                   sub_msg.gyro_x, sub_msg.gyro_y, sub_msg.gyro_z,
                   count++);
        pub.publish(main_msg);

        blinkLed();
        osDelay(1000); // 1 Hz loop
    }

    mros2::spin();
    return 0;
}

/* Rover Control + IMU Publisher Example
 * Combines motor/steering control subscriber (Code A)
 * with IMU sensor publisher (Code B).
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

// ---------------- Motor & Steering Control (from Code A) ----------------
float frontControl(uint8_t frontDirection, float diff_degree);
std::tuple<float, uint8_t, uint8_t> backControl(uint8_t backDirection, uint8_t dutycycle_PWM);
void motorDrive(float duty,uint8_t EN_A,uint8_t EN_B ,uint8_t period_PWM, float percent_dutycycle);

PwmOut DirectPWM(PA_3);
PwmOut MortorRPWM(PA_6);
PwmOut MortorLPWM(PE_11);

DigitalOut MortorRAEN(PF_12);
DigitalOut MortorRBEN(PD_15);
DigitalOut MortorLAEN(PF_13);
DigitalOut MortorLBEN(PE_9);

uint8_t servo_center = 100;
uint8_t period_PWM = 20;

void userCallback(msgs_ifaces::msg::MainRocon *msg)
{
    MROS2_INFO("########## Subscribe topic pub_rovercontrol ###############");
    MROS2_INFO("fdr_msg: %d, ro_ctrl_msg: %.2f, spd_msg: %d, bdr_msg: %d",
        msg->mainrocon_msg.fdr_msg, msg->mainrocon_msg.ro_ctrl_msg,
        msg->mainrocon_msg.spd_msg, msg->mainrocon_msg.bdr_msg);

    float dutycy = frontControl(msg->mainrocon_msg.fdr_msg, msg->mainrocon_msg.ro_ctrl_msg);
    auto [percent_dutycycle, EN_A, EN_B] = backControl(msg->mainrocon_msg.bdr_msg, msg->mainrocon_msg.spd_msg);
    motorDrive(dutycy, EN_A, EN_B, period_PWM, percent_dutycycle);
}

float frontControl(uint8_t frontDirection, float diff_degree) {
    uint8_t degree = 0;
    if (frontDirection == 1) {
        degree = servo_center - diff_degree;
    } else if (frontDirection == 3) {
        degree = servo_center + diff_degree;
    } else {
        degree = servo_center;
    }
    return 0.05f + (degree / 180.0f) * (0.10f - 0.05f);
}

std::tuple<float, uint8_t, uint8_t> backControl(uint8_t backDirection, uint8_t dutycycle_PWM) {
    uint8_t EN_A = 0;
    uint8_t EN_B = 0;
    float cal_percent_dutycycle = dutycycle_PWM / 100.0f;

    if (backDirection == 1) {
        EN_A = 1; EN_B = 0;
    } else if (backDirection == 2) {
        EN_A = 0; EN_B = 1;
    } else {
        EN_A = 0; EN_B = 0;
    }
    return {cal_percent_dutycycle, EN_A, EN_B};
}

void motorDrive(float duty,uint8_t EN_A,uint8_t EN_B ,uint8_t period_PWM, float percent_dutycycle){
  duty = std::max(0.0f, std::min(1.0f, duty));
  DirectPWM.period_ms(20);
  DirectPWM.write(duty);

  MortorLAEN.write(EN_A);
  MortorLBEN.write(EN_B);
  MortorRAEN.write(EN_B);
  MortorRBEN.write(EN_A);

  MortorRPWM.period_us(period_PWM);
  MortorRPWM.write(percent_dutycycle);
  MortorLPWM.period_us(period_PWM);
  MortorLPWM.write(percent_dutycycle);
}

// ---------------- IMU Publisher (from Code B) ----------------
LSM6DSV16X lsm6dsv16x(I2C_SDA, I2C_SCL);
DigitalOut led(LED1);

int32_t accelerometer[3], anglerate[3];

void blinkLed() {
    led = !led;
}

// ---------------- Main ----------------
int main()
{
  setenv("ROS_DOMAIN_ID", "10", 5);

  if (mros2_platform::network_connect()) {
    MROS2_ERROR("failed to connect and setup network! aborting,,,");
    return -1;
  } else {
    MROS2_INFO("successfully connect and setup network \n---");
  }

  MROS2_INFO("%s start!", MROS2_PLATFORM_NAME);
  MROS2_INFO("app name: RoverWithIMU");

  mros2::init(0, NULL);
  mros2::Node node = mros2::Node::create_node("rover_node");

  // Subscriber for rover control
  mros2::Subscriber sub = node.create_subscription<msgs_ifaces::msg::MainRocon>(
      "pub_rovercontrol", 10, userCallback);

  // Publisher for IMU data
  mros2::Publisher pub = node.create_publisher<msgs_ifaces::msg::MainGyroData>(
      "tp_imu_data_d5", 10);

  osDelay(1000);
  MROS2_INFO("ready to pub/sub message\r\n---");

  // Init IMU
  lsm6dsv16x.begin();
  lsm6dsv16x.Enable_X();
  lsm6dsv16x.Enable_G();

  uint8_t id;
  lsm6dsv16x.ReadID(&id);
  MROS2_INFO("LSM6DSV16X ID = 0x%02X", id);

  int count = 0;

  while (1) {
      // ----------- Rover subscriber runs in callback -----------

      // ----------- IMU Publisher -----------
      lsm6dsv16x.Get_X_Axes(accelerometer);
      lsm6dsv16x.Get_G_Axes(anglerate);

      msgs_ifaces::msg::SubGyroData sub_msg;
      sub_msg.accel_x = accelerometer[0];
      sub_msg.accel_y = accelerometer[1];
      sub_msg.accel_z = accelerometer[2];
      sub_msg.gyro_x  = anglerate[0];
      sub_msg.gyro_y  = anglerate[1];
      sub_msg.gyro_z  = anglerate[2];

      msgs_ifaces::msg::MainGyroData main_msg;
      main_msg.maingyrodata_msg = sub_msg;  // Embed sub message

      MROS2_INFO("publishing accel: X=%d Y=%d Z=%d | gyro: X=%d Y=%d Z=%d | Count=%d",
                   sub_msg.accel_x, sub_msg.accel_y, sub_msg.accel_z,
                   sub_msg.gyro_x, sub_msg.gyro_y, sub_msg.gyro_z,
                   count++);

      pub.publish(main_msg);
      blinkLed();
      osDelay(1000); // publish at 1 Hz
  }

  mros2::spin();
  return 0;
}

/* mros2 example
 * Copyright (c) 2022 mROS-base
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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

#include <tuple>
//#include "geometry_msgs/msg/pose.hpp"

float frontControl(uint8_t frontDirection, float diff_degree);
std::tuple<float, uint8_t, uint8_t> backControl(uint8_t backDirection, uint8_t dutycycle_PWM);
void motorDrive(float duty,uint8_t EN_A,uint8_t EN_B ,uint8_t period_PWM, float percent_dutycycle);

PwmOut DirectPWM(PA_3);
PwmOut MortorRPWM(PA_6);  //a = PD_15, b = PF_12 
PwmOut MortorLPWM(PE_11); //A = PE_9, PF_13

// DigitalOut MortorFWEN(PE_13);
// DigitalOut MortorBWEN(PF_13);

DigitalOut MortorRAEN(PF_12);
DigitalOut MortorRBEN(PD_15);
DigitalOut MortorLAEN(PF_13);
DigitalOut MortorLBEN(PE_9);


uint8_t servo_center = 100;
uint8_t period_PWM = 20;

void userCallback(msgs_ifaces::msg::MainRocon *msg)
{
    MROS2_INFO("########## Subscribe topic pub_rovercontrol ###############");
    MROS2_INFO("fdr_msg: %d, ro_ctrl_msg: %.2f, spd_msg: %d, bdr_msg: %d", msg->mainrocon_msg.fdr_msg, msg->mainrocon_msg.ro_ctrl_msg, msg->mainrocon_msg.spd_msg, msg->mainrocon_msg.bdr_msg);
    float dutycy = frontControl(msg->mainrocon_msg.fdr_msg, msg->mainrocon_msg.ro_ctrl_msg);
    auto [percent_dutycycle, EN_A, EN_B] = backControl(msg->mainrocon_msg.bdr_msg, msg->mainrocon_msg.spd_msg);
    motorDrive(dutycy, EN_A, EN_B, period_PWM, percent_dutycycle);
    MROS2_INFO("fdr_msg: %d, ro_ctrl_msg: %.2f, spd_msg: %d, bdr_msg: %d", 
        msg->mainrocon_msg.fdr_msg, msg->mainrocon_msg.ro_ctrl_msg, 
        msg->mainrocon_msg.spd_msg, msg->mainrocon_msg.bdr_msg);
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

    if (backDirection == 1.0) {
        EN_A = 1;
        EN_B = 0;
    } else if (backDirection == 2.0) {
        EN_A = 0;
        EN_B = 1;
    } else {
        EN_A = 0;
        EN_B = 0;
    }
    return {cal_percent_dutycycle, EN_A, EN_B};
}

void motorDrive(float duty,uint8_t EN_A,uint8_t EN_B ,uint8_t period_PWM, float percent_dutycycle){
  duty = std::max(0.0f, std::min(1.0f, duty));

  ////////////////////////////////////
  DirectPWM.period_ms(20);
  DirectPWM.write(duty);
  ////////////////////////////////////
//   MortorFWEN.write(EN_A);
//   MortorBWEN.write(EN_B);
  MortorLAEN.write(EN_A);
  MortorLBEN.write(EN_B);

  MortorRAEN.write(EN_B);
  MortorRBEN.write(EN_A);
  
  MortorRPWM.period_us(period_PWM);
  MortorRPWM.write(percent_dutycycle);
  MortorLPWM.period_us(period_PWM);
  MortorLPWM.write(percent_dutycycle);
}

int main()
{
  setenv("ROS_DOMAIN_ID", "10", 5);
  /* connect to the network */
  if (mros2_platform::network_connect())
  {
    MROS2_ERROR("failed to connect and setup network! aborting,,,");
    return -1;
  }
  else
  {
    MROS2_INFO("successfully connect and setup network\r\n---");
  }

  MROS2_INFO("%s start!", MROS2_PLATFORM_NAME);
  MROS2_INFO("app name: STM32Layer2");
  MROS2_INFO("topic name: pub_rovercontrol");

  mros2::init(0, NULL);
  MROS2_DEBUG("mROS 2 initialization is completed");

  mros2::Node node = mros2::Node::create_node("mros2_node");
  //mros2::Publisher pub = node.create_publisher<std_msgs::msg::String>("to_linux", 10);
  
  mros2::Subscriber sub = node.create_subscription<msgs_ifaces::msg::MainRocon>("pub_rovercontrol", 10, userCallback);
  
  osDelay(1000);
  MROS2_INFO("ready to pub/sub message\r\n---");

  mros2::spin();
  return 0;
}
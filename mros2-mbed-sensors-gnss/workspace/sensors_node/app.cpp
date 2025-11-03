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

#include "mros2.h"
#include "mros2-platform.h"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "msgs_ifaces/msg/main_sens_data.hpp"
#include "msgs_ifaces/msg/sub_sens_data.hpp"
#include <cstdlib>

// ------------------- ENCODERS -------------------
InterruptIn encA_A(PA_15);
InterruptIn encA_B(PB_5);

InterruptIn encB_A(PB_3);
InterruptIn encB_B(PB_4);

volatile int32_t countA = 0;
volatile int32_t countB = 0;

void encA_A_rise() { countA += (encA_B.read() == 0) ? 1 : -1; }
void encA_A_fall() { countA += (encA_B.read() == 1) ? 1 : -1; }

void encB_A_rise() { countB += (encB_B.read() == 0) ? 1 : -1; }
void encB_A_fall() { countB += (encB_B.read() == 1) ? 1 : -1; }

// ------------------- INA226 -------------------
I2C i2c(PB_9, PB_8); // SDA, SCL
const int INA226_ADDR = 0x40 << 1;
const float Rshunt = 0.1f;

int16_t read_register(uint8_t reg) {
    char cmd[1] = { reg };
    if (i2c.write(INA226_ADDR, cmd, 1, true) != 0) return -1;
    char data[2];
    if (i2c.read(INA226_ADDR, data, 2) != 0) return -1;
    return (data[0] << 8) | data[1];
}


int main()
{
  setenv("ROS_DOMAIN_ID", "6", 6);
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
  MROS2_INFO("app name: STM32Layer5");

  mros2::init(0, NULL);
  MROS2_DEBUG("mROS 2 initialization is completed");

  mros2::Node node = mros2::Node::create_node("mros2_node_2");

  mros2::Publisher PubSensData = node.create_publisher<msgs_ifaces::msg::MainSensData>("tp_sensdata_d5", 10);
  //mros2 : 1 message type มีได้แค่ 1 topic **มากกว่านั้น compile ไม่ผ่าน
  encA_A.rise(&encA_A_rise);
  encA_A.fall(&encA_A_fall);
  encB_A.rise(&encB_A_rise);
  encB_A.fall(&encB_A_fall);

  i2c.frequency(400000);

  osDelay(200);
  MROS2_INFO("ready to pub/sub message\r\n---");

  while (true) {
        int16_t bus_raw = read_register(0x02);
        int16_t shunt_raw = read_register(0x01);

        float bus_voltage = bus_raw * 1.25e-3f;      // 1.25 mV per LSB
        float shunt_voltage = shunt_raw * 2.5e-6f;   // 2.5 uV per LSB
        float current = shunt_voltage / Rshunt;

        //Publish encoder counts
        msgs_ifaces::msg::MainSensData msgs;
        msgs.mainsensdata_msg.mt_lf_encode_msg = countA;
        msgs.mainsensdata_msg.mt_rt_encode_msg = countB;
        msgs.mainsensdata_msg.sys_current_msg = current;
        msgs.mainsensdata_msg.sys_volt_msg = bus_voltage;
        PubSensData.publish(msgs);

        printf("MotorA: %ld | MotorB: %ld | Vbus: %.3f V | I: %.3f A\r\n", 
               countA, countB, bus_voltage, current);

        ThisThread::sleep_for(200ms);
    }

  mros2::spin();
  return 0;
}
/*
MIT License

Copyright (c) [2024] 
Organization: Perlatecnica APS ETS
Author: Mauro D'Angelo

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef STTS22H_H
#define STTS22H_H

#include "mbed.h"
#include "stts22h_registers.h"

typedef enum
{
  STTS22H_OK = 0,
  STTS22H_ERROR =-1
} STTS22HStatusTypeDef;

class STTS22H{
public:
    STTS22H(PinName sda, PinName scl);
    STTS22H(PinName mosi, PinName miso, PinName sck, PinName cs);

    ~STTS22H();

    enum InterfaceType {
        INTERFACE_I2C,
        INTERFACE_SPI
    };

    void initialize();
    STTS22HStatusTypeDef begin();
    STTS22HStatusTypeDef end();
    STTS22HStatusTypeDef ReadID(uint8_t *Id);
    STTS22HStatusTypeDef Enable();
    STTS22HStatusTypeDef Disable();
    STTS22HStatusTypeDef GetOutputDataRate(float *Odr);
    STTS22HStatusTypeDef SetOutputDataRate(float Odr);
    STTS22HStatusTypeDef GetTemperature(float *Value);
    STTS22HStatusTypeDef Get_DRDY_Status(uint8_t *Status);
    STTS22HStatusTypeDef SetLowTemperatureThreshold(float Value);
    STTS22HStatusTypeDef SetHighTemperatureThreshold(float Value);
    STTS22HStatusTypeDef GetTemperatureLimitStatus(uint8_t *HighLimit, uint8_t *LowLimit);
    STTS22HStatusTypeDef Read_Reg(uint8_t Reg, uint8_t *Data);
    STTS22HStatusTypeDef Write_Reg(uint8_t Reg, uint8_t Data);
    STTS22HStatusTypeDef Set_One_Shot();
    STTS22HStatusTypeDef Get_One_Shot_Status(uint8_t *Status);

    bool readRegister(uint8_t reg, uint8_t *value, uint16_t len);
    bool writeRegister(uint8_t reg, const uint8_t *value, uint16_t len);
 

private:
    I2C* i2c;
    SPI* spi;
    DigitalOut* cs_pin;

    #ifdef IKS4A1
        uint8_t stts22h_8bit_address = (0x38 << 1); // 8 bits device address
    #else // DEFAULT ADDRESS
        uint8_t stts22h_8bit_address = (0x38 << 1); // 8 bits device address
    #endif

    uint32_t spi_speed;

    float temp_odr;
    uint8_t temp_is_enabled;

    int32_t block_data_update_set(uint8_t val);
    float_t from_lsb_to_celsius(int16_t lsb);
    int32_t temp_data_rate_set(stts22h_odr_temp_t val);
    int32_t temp_data_rate_get(stts22h_odr_temp_t *val);
    int32_t block_data_update_get(uint8_t *val);
    int32_t temp_flag_data_ready_get(uint8_t *val);
    int32_t temperature_raw_get(int16_t *buff);
    int32_t dev_id_get(uint8_t *buff);
    int32_t dev_status_get(stts22h_dev_status_t *val);
    int32_t smbus_interface_set(stts22h_smbus_md_t val);
    int32_t smbus_interface_get(stts22h_smbus_md_t *val);
    int32_t auto_increment_set(uint8_t val);
    int32_t auto_increment_get(uint8_t *val);
    int32_t temp_trshld_high_set(uint8_t val);
    int32_t temp_trshld_high_get(uint8_t *val);
    int32_t temp_trshld_low_set(uint8_t val);
    int32_t temp_trshld_low_get(uint8_t *val);
    int32_t temp_trshld_src_get(stts22h_temp_trlhd_src_t *val);

};

#endif // STTS22H_H
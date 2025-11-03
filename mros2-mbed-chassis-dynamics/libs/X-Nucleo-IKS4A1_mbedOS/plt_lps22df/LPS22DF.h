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

#ifndef LPS22DF_H
#define LPS22DF_H

#include "mbed.h"
#include "LPS22DF_registers.h"

#define LPS22DF_I2C_BUS          0U
#define LPS22DF_SPI_4WIRES_BUS   1U
#define LPS22DF_SPI_3WIRES_BUS   2U

typedef enum {
        LPS22DF_OK = 0,
        LPS22DF_ERROR = -1
    } LPS22DFStatusTypeDef;

class LPS22DF{
public:
    LPS22DF(PinName sda, PinName scl);
    LPS22DF(PinName mosi, PinName miso, PinName sck, PinName cs);

    ~LPS22DF();

    enum InterfaceType {
        INTERFACE_I2C,
        INTERFACE_SPI
    };

    void initialize();

    LPS22DFStatusTypeDef begin();
    LPS22DFStatusTypeDef end();
    LPS22DFStatusTypeDef ReadID(uint8_t *Id);
    LPS22DFStatusTypeDef Enable();
    LPS22DFStatusTypeDef Disable();
    LPS22DFStatusTypeDef GetOutputDataRate(float *Odr);
    LPS22DFStatusTypeDef SetOutputDataRate(float Odr);

    LPS22DFStatusTypeDef SetOutputDataRate_When_Enabled(float Odr);
    LPS22DFStatusTypeDef SetOutputDataRate_When_Disabled(float Odr);

    LPS22DFStatusTypeDef GetPressure(float *Value);
    LPS22DFStatusTypeDef Get_PRESS_DRDY_Status(uint8_t *Status);

    LPS22DFStatusTypeDef GetTemperature(float *Value);
    LPS22DFStatusTypeDef Get_TEMP_DRDY_Status(uint8_t *Status);

    LPS22DFStatusTypeDef Read_Reg(uint8_t Reg, uint8_t *Data);
    LPS22DFStatusTypeDef Write_Reg(uint8_t Reg, uint8_t Data);

    LPS22DFStatusTypeDef Set_One_Shot();
    LPS22DFStatusTypeDef Get_One_Shot_Status(uint8_t *Status);

    bool readRegister(uint8_t reg, uint8_t *value, uint16_t len);
    bool writeRegister(uint8_t reg, const uint8_t *value, uint16_t len);
 
 private:
    I2C* i2c;
    SPI* spi;
    DigitalOut* cs_pin;

    #ifdef IKS4A1
        uint8_t stts22h_8bit_address = (0x5D << 1); // 8 bits device address
    #else // DEFAULT ADDRESS
        uint8_t stts22h_8bit_address = ((uint8_t)(0x5D << 1)); // 8 bits device address
    #endif

    uint32_t spi_speed;
    uint32_t     bus_type; /*0 means I2C, 1 means SPI 4-Wires, 2 means SPI-3-Wires */
    uint8_t      initialized;
    uint8_t      enabled;
    lps22df_md_t last_md;


    float_t lps22df_from_lsb_to_hPa(int32_t lsb);
    float_t lps22df_from_lsb_to_celsius(int16_t lsb);

    int32_t lps22df_id_get(lps22df_id_t *val);
    int32_t lps22df_bus_mode_set(lps22df_bus_mode_t *val);
    int32_t lps22df_bus_mode_get(lps22df_bus_mode_t *val);
    int32_t lps22df_init_set(lps22df_init_t val);
    int32_t lps22df_status_get(lps22df_stat_t *val);
    int32_t lps22df_pin_conf_set(lps22df_pin_conf_t *val);
    int32_t lps22df_pin_conf_get(lps22df_pin_conf_t *val);
    int32_t lps22df_all_sources_get(lps22df_all_sources_t *val);
    int32_t lps22df_mode_set(lps22df_md_t *val);
    int32_t lps22df_mode_get(lps22df_md_t *val);
    int32_t lps22df_trigger_sw(lps22df_md_t *md);
    int32_t lps22df_data_get(lps22df_data_t *data);
    int32_t lps22df_fifo_mode_set(lps22df_fifo_md_t *val);
    int32_t lps22df_fifo_mode_get(lps22df_fifo_md_t *val);
    int32_t lps22df_fifo_level_get(uint8_t *val);
    int32_t lps22df_fifo_data_get(uint8_t samp, lps22df_fifo_data_t *data);
    int32_t lps22df_interrupt_mode_set(lps22df_int_mode_t *val);
    int32_t lps22df_interrupt_mode_get(lps22df_int_mode_t *val);
    int32_t lps22df_pin_int_route_set(lps22df_pin_int_route_t *val);
    int32_t lps22df_pin_int_route_get(lps22df_pin_int_route_t *val);
    int32_t lps22df_int_on_threshold_mode_set(lps22df_int_th_md_t *val);
    int32_t lps22df_int_on_threshold_mode_get(lps22df_int_th_md_t *val);
    int32_t lps22df_reference_mode_set(lps22df_ref_md_t *val);
    int32_t lps22df_reference_mode_get(lps22df_ref_md_t *val);
    int32_t lps22df_opc_set(int16_t val);
    int32_t lps22df_opc_get(int16_t *val);
};



#endif
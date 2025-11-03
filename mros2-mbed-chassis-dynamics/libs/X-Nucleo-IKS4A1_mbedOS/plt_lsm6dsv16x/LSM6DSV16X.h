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

#ifndef LSM6DSV16X_H
#define LSM6DSV16X_H

#include "mbed.h"
#include <cstdint>
#include "registers.h"



class LSM6DSV16X {
public:
    LSM6DSV16X(PinName sda, PinName scl);
    LSM6DSV16X(PinName mosi, PinName miso, PinName sck, PinName cs);

    ~LSM6DSV16X();

    enum InterfaceType {
        INTERFACE_I2C,
        INTERFACE_SPI
    };

    void initialize();
    void readSensorData();
    LSM6DSV16XStatusTypeDef ReadID(uint8_t *val);
    LSM6DSV16XStatusTypeDef begin(void);
    LSM6DSV16XStatusTypeDef end();
    LSM6DSV16XStatusTypeDef Enable_X();
    LSM6DSV16XStatusTypeDef Get_X_Axes(int32_t *Acceleration);
    LSM6DSV16XStatusTypeDef Set_X_FS(int32_t FullScale);
    LSM6DSV16XStatusTypeDef Get_X_ODR(float *Odr);
    LSM6DSV16XStatusTypeDef Set_X_ODR(float Odr, LSM6DSV16X_ACC_Operating_Mode_t Mode = LSM6DSV16X_ACC_HIGH_PERFORMANCE_MODE);
    LSM6DSV16XStatusTypeDef Set_X_ODR_When_Enabled(float Odr);
    LSM6DSV16XStatusTypeDef Set_X_ODR_When_Disabled(float Odr);
    LSM6DSV16XStatusTypeDef Disable_X();
    LSM6DSV16XStatusTypeDef Enable_G();
    LSM6DSV16XStatusTypeDef Disable_G();
    LSM6DSV16XStatusTypeDef Get_G_Sensitivity(float *Sensitivity);
    float Convert_G_Sensitivity(lsm6dsv16x_gy_full_scale_t full_scale);
    LSM6DSV16XStatusTypeDef Get_G_ODR(float *Odr);
    LSM6DSV16XStatusTypeDef Set_G_ODR(float Odr, LSM6DSV16X_GYRO_Operating_Mode_t Mode = LSM6DSV16X_GYRO_HIGH_PERFORMANCE_MODE);
    LSM6DSV16XStatusTypeDef Set_G_ODR_When_Enabled(float Odr);
    LSM6DSV16XStatusTypeDef Set_G_ODR_When_Disabled(float Odr);
    LSM6DSV16XStatusTypeDef Get_G_FS(int32_t  *FullScale);
    LSM6DSV16XStatusTypeDef Set_G_FS(int32_t FullScale);
    LSM6DSV16XStatusTypeDef Get_G_AxesRaw(int16_t *Value);
    LSM6DSV16XStatusTypeDef Get_G_Axes(int32_t *AngularRate);
    LSM6DSV16XStatusTypeDef Get_G_DRDY_Status(uint8_t *Status);
    LSM6DSV16XStatusTypeDef Set_G_Power_Mode(uint8_t PowerMode);
    LSM6DSV16XStatusTypeDef Set_G_Filter_Mode(uint8_t LowHighPassFlag, uint8_t FilterMode);
    LSM6DSV16XStatusTypeDef Get_G_AxesRaw_When_Aval(int16_t *Value);
    LSM6DSV16XStatusTypeDef Get_X_AxesRaw_When_Aval(int16_t *Value);
    LSM6DSV16XStatusTypeDef Get_X_AxesRaw(int16_t *Value);
    LSM6DSV16XStatusTypeDef Get_Temp_ODR(float *Odr);    
    LSM6DSV16XStatusTypeDef Set_Temp_ODR(float Odr);
    LSM6DSV16XStatusTypeDef Get_Temp_Raw(int16_t *value);
    LSM6DSV16XStatusTypeDef Test_IMU(uint8_t XTestType, uint8_t GTestType);
    LSM6DSV16XStatusTypeDef Test_X_IMU(uint8_t TestType);
    LSM6DSV16XStatusTypeDef Test_G_IMU(uint8_t TestType = LSM6DSV16X_GY_ST_POSITIVE);
    LSM6DSV16XStatusTypeDef QVAR_Enable();
    LSM6DSV16XStatusTypeDef QVAR_Disable();
    LSM6DSV16XStatusTypeDef QVAR_GetData(float *Data);
    LSM6DSV16XStatusTypeDef QVAR_GetImpedance(uint16_t *val);
    LSM6DSV16XStatusTypeDef QVAR_SetImpedance(uint16_t val);
    LSM6DSV16XStatusTypeDef QVAR_GetStatus(uint8_t *val);
    LSM6DSV16XStatusTypeDef Get_MLC_Status(lsm6dsv16x_mlc_status_mainpage_t *status);
    LSM6DSV16XStatusTypeDef Get_MLC_Output(lsm6dsv16x_mlc_out_t *output);
    LSM6DSV16XStatusTypeDef Enable_Rotation_Vector();
    LSM6DSV16XStatusTypeDef Disable_Rotation_Vector();
    LSM6DSV16XStatusTypeDef Enable_Gravity_Vector();
    LSM6DSV16XStatusTypeDef Disable_Gravity_Vector();
    LSM6DSV16XStatusTypeDef Enable_Gyroscope_Bias();
    LSM6DSV16XStatusTypeDef Disable_Gyroscope_Bias();
    LSM6DSV16XStatusTypeDef Set_SFLP_Batch(bool GameRotation, bool Gravity, bool gBias);
    LSM6DSV16XStatusTypeDef Set_SFLP_ODR(float odr);
    LSM6DSV16XStatusTypeDef Set_SFLP_GBIAS(float x, float y, float z);
    LSM6DSV16XStatusTypeDef Reset_SFLP(void);
    LSM6DSV16XStatusTypeDef Enable_Block_Data_Update();
    LSM6DSV16XStatusTypeDef Disable_Block_Data_Update();
    LSM6DSV16XStatusTypeDef Enable_Auto_Increment();
    LSM6DSV16XStatusTypeDef Disable_Auto_Increment();
    LSM6DSV16XStatusTypeDef Device_Reset(LSM6DSV16X_Reset_t flags);
    LSM6DSV16XStatusTypeDef Get_6D_Orientation_XL(uint8_t *XLow);
    LSM6DSV16XStatusTypeDef Get_6D_Orientation_XH(uint8_t *XHigh);
    LSM6DSV16XStatusTypeDef Get_6D_Orientation_YL(uint8_t *YLow);
    LSM6DSV16XStatusTypeDef Get_6D_Orientation_YH(uint8_t *YHigh);
    LSM6DSV16XStatusTypeDef Get_6D_Orientation_ZL(uint8_t *ZLow);
    LSM6DSV16XStatusTypeDef Get_6D_Orientation_ZH(uint8_t *ZHigh);
    LSM6DSV16XStatusTypeDef Set_6D_Orientation_Threshold(uint8_t Threshold);
    LSM6DSV16XStatusTypeDef Enable_6D_Orientation(LSM6DSV16X_SensorIntPin_t IntPin);
    LSM6DSV16XStatusTypeDef Disable_6D_Orientation();
    LSM6DSV16XStatusTypeDef Get_X_Event_Status(LSM6DSV16X_Event_Status_t *Status);
    LSM6DSV16XStatusTypeDef Enable_Single_Tap_Detection(LSM6DSV16X_SensorIntPin_t IntPin);
    LSM6DSV16XStatusTypeDef Enable_Double_Tap_Detection(LSM6DSV16X_SensorIntPin_t IntPin);
    LSM6DSV16XStatusTypeDef Disable_Double_Tap_Detection();
    LSM6DSV16XStatusTypeDef Set_Tap_Threshold(uint8_t Threshold);
    LSM6DSV16XStatusTypeDef Set_Tap_Shock_Time(uint8_t Time);
    LSM6DSV16XStatusTypeDef Set_Tap_Quiet_Time(uint8_t Time);
    LSM6DSV16XStatusTypeDef Set_Tap_Duration_Time(uint8_t Time);
    LSM6DSV16XStatusTypeDef Enable_Tilt_Detection(LSM6DSV16X_SensorIntPin_t IntPin);
    LSM6DSV16XStatusTypeDef Disable_Tilt_Detection();
    LSM6DSV16XStatusTypeDef Enable_Pedometer(LSM6DSV16X_SensorIntPin_t IntPin);
    LSM6DSV16XStatusTypeDef Disable_Pedometer();
    LSM6DSV16XStatusTypeDef Get_Step_Count(uint16_t *StepCount);
    LSM6DSV16XStatusTypeDef Step_Counter_Reset();
    LSM6DSV16XStatusTypeDef Enable_Free_Fall_Detection(LSM6DSV16X_SensorIntPin_t IntPin);
    LSM6DSV16XStatusTypeDef Disable_Free_Fall_Detection();
    LSM6DSV16XStatusTypeDef Set_Free_Fall_Threshold(uint8_t Threshold);
    LSM6DSV16XStatusTypeDef Set_Free_Fall_Duration(uint8_t Duration);
    LSM6DSV16XStatusTypeDef Enable_Wake_Up_Detection(LSM6DSV16X_SensorIntPin_t IntPin);
    LSM6DSV16XStatusTypeDef Disable_Wake_Up_Detection();
    LSM6DSV16XStatusTypeDef Set_Wake_Up_Threshold(uint32_t Threshold);
    LSM6DSV16XStatusTypeDef Set_Wake_Up_Duration(uint8_t Duration);
    LSM6DSV16XStatusTypeDef FIFO_Reset();
    LSM6DSV16XStatusTypeDef FIFO_Get_Num_Samples(uint16_t *NumSamples);
    LSM6DSV16XStatusTypeDef FIFO_Get_Tag(uint8_t *Tag);
    LSM6DSV16XStatusTypeDef FIFO_Get_Rotation_Vector(float *rvec);
    LSM6DSV16XStatusTypeDef FIFO_Get_Data(uint8_t *Data);
    LSM6DSV16XStatusTypeDef FIFO_Get_Full_Status(uint8_t *Status);
    LSM6DSV16XStatusTypeDef FIFO_Set_INT1_FIFO_Full(uint8_t Status);
    LSM6DSV16XStatusTypeDef FIFO_Set_INT2_FIFO_Full(uint8_t Status);
    LSM6DSV16XStatusTypeDef FIFO_Set_Watermark_Level(uint8_t Watermark);
    LSM6DSV16XStatusTypeDef FIFO_Set_Stop_On_Fth(uint8_t Status);
    LSM6DSV16XStatusTypeDef FIFO_Set_Mode(uint8_t Mode);
    LSM6DSV16XStatusTypeDef FIFO_Get_X_Axes(int32_t *Acceleration);
    LSM6DSV16XStatusTypeDef FIFO_Set_X_BDR(float Bdr);
    LSM6DSV16XStatusTypeDef FIFO_Get_G_Axes(int32_t *AngularVelocity);
    LSM6DSV16XStatusTypeDef FIFO_Set_G_BDR(float Bdr);
    LSM6DSV16XStatusTypeDef FIFO_Get_Status(lsm6dsv16x_fifo_status_t *Status);
    LSM6DSV16XStatusTypeDef FIFO_Get_Gravity_Vector(float *gvec);
    LSM6DSV16XStatusTypeDef FIFO_Get_Gyroscope_Bias(float *gbias);
    LSM6DSV16XStatusTypeDef FIFO_Enable_Timestamp();
    LSM6DSV16XStatusTypeDef FIFO_Disable_Timestamp();
    LSM6DSV16XStatusTypeDef FIFO_Set_Timestamp_Decimation(uint8_t decimation);
    LSM6DSV16XStatusTypeDef FIFO_Get_Timestamp(uint32_t *timestamp);
    
    bool readRegister(uint8_t reg, uint8_t *value, uint16_t len);
    bool writeRegister(uint8_t reg, const uint8_t *value, uint16_t len);
    
    void set_SDO_SAO_TO_GND();
    void set_SDO_SAO_TO_VCC();


private:
    I2C* i2c;
    SPI* spi;
    DigitalOut* cs_pin;

    #ifdef IKS4A1
        uint8_t lsm6ds01tis_8bit_address = (0x6B << 1); // 8 bits device address
    #else // DEFAULT ADDRESS
        uint8_t lsm6ds01tis_8bit_address = (0x6B << 1); // 8 bits device address
    #endif

    uint32_t spi_speed;

    lsm6dsv16x_data_rate_t acc_odr;
    lsm6dsv16x_data_rate_t gyro_odr;
    lsm6dsv16x_xl_full_scale_t acc_fs;
    lsm6dsv16x_gy_full_scale_t gyro_fs;
    lsm6dsv16x_fifo_mode_t fifo_mode;
    uint8_t acc_is_enabled;
    uint8_t gyro_is_enabled;
    uint8_t initialized;
    
    float_t from_sflp_to_mg(int16_t lsb);
    float_t from_fs2_to_mg(int16_t lsb);
    float_t from_fs4_to_mg(int16_t lsb);
    float_t from_fs8_to_mg(int16_t lsb);
    float_t from_fs16_to_mg(int16_t lsb);
    float_t from_fs125_to_mdps(int16_t lsb);
    float_t from_fs250_to_mdps(int16_t lsb);
    float_t from_fs500_to_mdps(int16_t lsb);
    float_t from_fs1000_to_mdps(int16_t lsb);
    float_t from_fs2000_to_mdps(int16_t lsb);
    float_t from_fs4000_to_mdps(int16_t lsb);
    float_t from_lsb_to_celsius(int16_t lsb);
    float_t from_lsb_to_nsec(uint32_t lsb);
    float_t from_lsb_to_mv(int16_t lsb);

    void bytecpy(uint8_t *target, uint8_t *source);

    uint16_t npy_float_to_half(float_t f);
    uint16_t npy_floatbits_to_halfbits(uint32_t f);
    LSM6DSV16XStatusTypeDef npy_halfbits_to_floatbits(uint16_t h, uint32_t *f);
    LSM6DSV16XStatusTypeDef npy_half_to_float(uint16_t h, float *f);
    LSM6DSV16XStatusTypeDef sflp2q(float quat[4], uint16_t sflp[3]);

    
    
    int32_t auto_increment_set(uint8_t val);
    int32_t block_data_update_set(uint8_t val);
    int32_t xl_data_rate_set(lsm6dsv16x_data_rate_t val);   
    int32_t xl_data_rate_get(lsm6dsv16x_data_rate_t *val);
    int32_t xl_self_test_set(lsm6dsv16x_xl_self_test_t val);
    int32_t xl_full_scale_set(lsm6dsv16x_xl_full_scale_t val);
    int32_t xl_full_scale_get(lsm6dsv16x_xl_full_scale_t *val);
    int32_t xl_mode_set(lsm6dsv16x_xl_mode_t val); 
    int32_t xl_mode_get(lsm6dsv16x_xl_mode_t *val);
    int32_t angular_rate_raw_get(int16_t *val);
    int32_t acceleration_raw_get(int16_t *val);
    int32_t temperature_raw_get(int16_t *val);
    int32_t dual_acceleration_raw_get(int16_t *val);
    int32_t all_sources_get(lsm6dsv16x_all_sources_t *val);
    int32_t mem_bank_set(lsm6dsv16x_mem_bank_t val);
    int32_t mem_bank_get(lsm6dsv16x_mem_bank_t *val);
    int32_t gy_data_rate_set(lsm6dsv16x_data_rate_t val);
    int32_t gy_full_scale_set(lsm6dsv16x_gy_full_scale_t val);
    int32_t gy_full_scale_get(lsm6dsv16x_gy_full_scale_t *val);
    int32_t gy_data_rate_get(lsm6dsv16x_data_rate_t *val);
    int32_t gy_mode_set(lsm6dsv16x_gy_mode_t val);
    int32_t gy_mode_get(lsm6dsv16x_gy_mode_t *val);
    int32_t gy_lp1_set(uint8_t val);
    int32_t gy_self_test_set(lsm6dsv16x_gy_self_test_t val);
    int32_t gy_self_test_get(lsm6dsv16x_gy_self_test_t *val);
    int32_t filt_gy_lp1_bandwidth_set(lsm6dsv16x_filt_gy_lp1_bandwidth_t val);
    int32_t flag_data_ready_get(lsm6dsv16x_data_ready_t *val);
    int32_t ah_qvar_raw_get(int16_t *val);
    int32_t ah_qvar_zin_get(lsm6dsv16x_ah_qvar_zin_t *val);
    int32_t ah_qvar_zin_set(lsm6dsv16x_ah_qvar_zin_t val);
    int32_t mlc_out_get(lsm6dsv16x_mlc_out_t *val);
    int32_t sflp_game_rotation_set(uint8_t val);
    int32_t sflp_game_rotation_get(uint8_t *val);
    int32_t sflp_data_rate_set(lsm6dsv16x_sflp_data_rate_t val);
    int32_t sflp_data_rate_get(lsm6dsv16x_sflp_data_rate_t *val);
    int32_t sflp_game_gbias_set(lsm6dsv16x_sflp_gbias_t *val);
    int32_t sh_master_set(uint8_t val);
    int32_t sh_master_get(uint8_t *val);
    int32_t ln_pg_write(uint16_t address, uint8_t *buf, uint8_t len);
    int32_t reset_set(lsm6dsv16x_reset_t val);
    int32_t reset_get(lsm6dsv16x_reset_t *val);
    int32_t fifo_status_get(lsm6dsv16x_fifo_status_t *val);
    int32_t _6d_threshold_set(lsm6dsv16x_6d_threshold_t val);
    float Convert_X_Sensitivity(lsm6dsv16x_xl_full_scale_t full_scale);
    int32_t tap_mode_set(lsm6dsv16x_tap_mode_t val);
    int32_t tap_mode_get(lsm6dsv16x_tap_mode_t *val);
    int32_t stpcnt_mode_get(lsm6dsv16x_stpcnt_mode_t *val);
    int32_t stpcnt_steps_get(uint16_t *val);
    int32_t stpcnt_rst_step_set(uint8_t val);
    int32_t stpcnt_rst_step_get(uint8_t *val);
    int32_t stpcnt_debounce_set(uint8_t val);
    int32_t stpcnt_debounce_get(uint8_t *val);
    int32_t stpcnt_period_set(uint16_t val);
    int32_t stpcnt_period_get(uint16_t *val);
    int32_t ln_pg_read(uint16_t address, uint8_t *buf, uint8_t len);
    int32_t stpcnt_mode_set(lsm6dsv16x_stpcnt_mode_t val);
    int32_t ff_thresholds_set(lsm6dsv16x_ff_thresholds_t val);
    int32_t ff_thresholds_get(lsm6dsv16x_ff_thresholds_t *val);
    int32_t ff_time_windows_set(uint8_t val);
    int32_t ff_time_windows_get(uint8_t *val);
    int32_t act_thresholds_get(lsm6dsv16x_act_thresholds_t *val);
    int32_t act_wkup_time_windows_set(lsm6dsv16x_act_wkup_time_windows_t val);
    int32_t act_wkup_time_windows_get(lsm6dsv16x_act_wkup_time_windows_t *val);
    int32_t act_thresholds_set(lsm6dsv16x_act_thresholds_t *val);
    int32_t fifo_watermark_set(uint8_t val);
    int32_t fifo_mode_set(lsm6dsv16x_fifo_mode_t val);
    int32_t fifo_sflp_batch_set(lsm6dsv16x_fifo_sflp_raw_t val);
    int32_t fifo_sflp_batch_get(lsm6dsv16x_fifo_sflp_raw_t *val);
    int32_t fifo_stop_on_wtm_set(uint8_t val);
    int32_t fifo_xl_batch_set(lsm6dsv16x_fifo_xl_batch_t val);
    int32_t fifo_gy_batch_set(lsm6dsv16x_fifo_gy_batch_t val);
    int32_t timestamp_set(uint8_t val);
    int32_t fifo_timestamp_batch_set(lsm6dsv16x_fifo_timestamp_batch_t val);
};

#endif // LSM6DSV16X_H
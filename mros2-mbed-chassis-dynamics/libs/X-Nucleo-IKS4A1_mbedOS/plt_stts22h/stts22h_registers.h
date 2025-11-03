/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STTS22H_REGS_H
#define STTS22H_REGS_H

#include <stdint.h>
#include <math.h>

#ifndef float_t
#define float_t float
#endif


#ifndef MEMS_SHARED_TYPES
#define MEMS_SHARED_TYPES

typedef union{
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;

typedef union{
  int16_t i16bit;
  uint8_t u8bit[2];
} axis1bit16_t;

typedef union{
  int32_t i32bit[3];
  uint8_t u8bit[12];
} axis3bit32_t;

typedef union{
  int32_t i32bit;
  uint8_t u8bit[4];
} axis1bit32_t;


typedef struct{
  uint8_t bit0       : 1;
  uint8_t bit1       : 1;
  uint8_t bit2       : 1;
  uint8_t bit3       : 1;
  uint8_t bit4       : 1;
  uint8_t bit5       : 1;
  uint8_t bit6       : 1;
  uint8_t bit7       : 1;
} stts22h_bitwise_t;

#define PROPERTY_DISABLE                (0U)
#define PROPERTY_ENABLE                 (1U)

#endif /* MEMS_SHARED_TYPES */


typedef int32_t (*stts22h_write_ptr)(void *, uint8_t, uint8_t*, uint16_t);
typedef int32_t (*stts22h_read_ptr) (void *, uint8_t, uint8_t*, uint16_t);

typedef struct {
  /** Component mandatory fields **/
  stts22h_write_ptr  write_reg;
  stts22h_read_ptr   read_reg;
  /** Customizable optional pointer **/
  void *handle;
} stts22h_ctx_t;


#ifndef MEMS_UCF_SHARED_TYPES
#define MEMS_UCF_SHARED_TYPES

typedef struct {
  uint8_t address;
  uint8_t data;
} stts22h_ucf_line_t;

#endif /* MEMS_UCF_SHARED_TYPES */


/** Device Identification (Who am I) **/
#define STTS22H_ID              0xA0U

#define STTS22H_WHOAMI                       0x01U
#define STTS22H_TEMP_H_LIMIT                 0x02U
typedef struct {
  uint8_t thl                 : 8;
} stts22h_temp_h_limit_t;

#define STTS22H_TEMP_L_LIMIT                 0x03U
typedef struct {
  uint8_t tll                 : 8;
} stts22h_temp_l_limit_t;

#define STTS22H_CTRL                         0x04U
typedef struct {
  uint8_t one_shot            : 1;
  uint8_t time_out_dis        : 1;
  uint8_t freerun             : 1;
  uint8_t if_add_inc          : 1;
  uint8_t avg                 : 2;
  uint8_t bdu                 : 1;
  uint8_t low_odr_start       : 1;
} stts22h_ctrl_t;

#define STTS22H_STATUS                       0x05U
typedef struct {
  uint8_t busy                : 1;
  uint8_t over_thh            : 1;
  uint8_t under_thl           : 1;
  uint8_t not_used_01         : 5;
} stts22h_status_t;

#define STTS22H_TEMP_L_OUT                   0x06U
#define STTS22H_TEMP_H_OUT                   0x07U
#define STTS22H_SOFTWARE_RESET               0x0CU
typedef struct {
  uint8_t not_used_01         : 1;
  uint8_t sw_reset            : 1;
  uint8_t not_used_02         : 4;
  uint8_t low_odr_enable      : 1;
  uint8_t not_used_03         : 1;
} stts22h_software_reset_t;

typedef union{
  stts22h_temp_h_limit_t      temp_h_limit;
  stts22h_temp_l_limit_t      temp_l_limit;
  stts22h_ctrl_t              ctrl;
  stts22h_status_t            status;
  stts22h_software_reset_t    software_reset;
  stts22h_bitwise_t                   bitwise;
  uint8_t                     byte;
} stts22h_reg_t;


int32_t stts22h_read_reg(stts22h_ctx_t *ctx, uint8_t reg, uint8_t* data,
                         uint16_t len);
int32_t stts22h_write_reg(stts22h_ctx_t *ctx, uint8_t reg, uint8_t* data,
                          uint16_t len);

extern float_t stts22h_from_lsb_to_celsius(int16_t lsb);

typedef enum {
  STTS22H_POWER_DOWN   = 0x00,
  STTS22H_ONE_SHOT     = 0x01,
  STTS22H_1Hz          = 0x04,
  STTS22H_25Hz         = 0x02,
  STTS22H_50Hz         = 0x12,
  STTS22H_100Hz        = 0x22,
  STTS22H_200Hz        = 0x32,
} stts22h_odr_temp_t;
int32_t stts22h_temp_data_rate_set(stts22h_ctx_t *ctx,
                                   stts22h_odr_temp_t val);
int32_t stts22h_temp_data_rate_get(stts22h_ctx_t *ctx,
                                   stts22h_odr_temp_t *val);

int32_t stts22h_block_data_update_set(stts22h_ctx_t *ctx, uint8_t val);
int32_t stts22h_block_data_update_get(stts22h_ctx_t *ctx, uint8_t *val);

int32_t stts22h_temp_flag_data_ready_get(stts22h_ctx_t *ctx, uint8_t *val);

int32_t stts22h_temperature_raw_get(stts22h_ctx_t *ctx, int16_t *buff);

int32_t stts22h_dev_id_get(stts22h_ctx_t *ctx, uint8_t *buff);

typedef struct {
  uint8_t busy             : 1;
} stts22h_dev_status_t;
int32_t stts22h_dev_status_get(stts22h_ctx_t *ctx, stts22h_dev_status_t *val);

typedef enum {
  STTS22H_SMBUS_TIMEOUT_ENABLE    = 0,
  STTS22H_SMBUS_TIMEOUT_DISABLE   = 1,
} stts22h_smbus_md_t;
int32_t stts22h_smbus_interface_set(stts22h_ctx_t *ctx,
                                    stts22h_smbus_md_t val);
int32_t stts22h_smbus_interface_get(stts22h_ctx_t *ctx,
                                    stts22h_smbus_md_t *val);

int32_t stts22h_auto_increment_set(stts22h_ctx_t *ctx, uint8_t val);
int32_t stts22h_auto_increment_get(stts22h_ctx_t *ctx, uint8_t *val);

int32_t stts22h_temp_trshld_high_set(stts22h_ctx_t *ctx, uint8_t val);
int32_t stts22h_temp_trshld_high_get(stts22h_ctx_t *ctx, uint8_t *val);

int32_t stts22h_temp_trshld_low_set(stts22h_ctx_t *ctx, uint8_t val);
int32_t stts22h_temp_trshld_low_get(stts22h_ctx_t *ctx, uint8_t *val);

typedef struct {
  uint8_t under_thl             : 1;
  uint8_t over_thh              : 1;
} stts22h_temp_trlhd_src_t;
int32_t stts22h_temp_trshld_src_get(stts22h_ctx_t *ctx,
                                    stts22h_temp_trlhd_src_t *val);
#endif /* STTS22H_REGS_H */

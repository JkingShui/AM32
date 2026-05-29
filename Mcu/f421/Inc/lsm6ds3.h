#ifndef __BSP_LSM6DS3_H
#define __BSP_LSM6DS3_H

#ifdef __cplusplus
extern "C" {
#endif

#include "at32f421_i2c.h"

#define LSM6DS3_I2C_ADDR    0x6B << 1

#define LSM6DS3_WHO_AM_I    0x0F
#define LSM6DS3_CTRL1_XL    0x10
#define LSM6DS3_CTRL2_G     0x11
#define LSM6DS3_CTRL3_C     0x12
#define LSM6DS3_OUTZ_L_G    0x26

uint8_t lsm6ds3_init(i2c_type *i2c_v);
uint8_t lsm6ds3_read_gyro_z(int16_t *z_data);
float lsm6ds3_convert_to_dps(int16_t raw);

#ifdef __cplusplus
}
#endif

#endif

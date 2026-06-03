#include "lsm6ds3.h"
#include "i2c_application.h"

#define GYRO_SCALE_2000DPS 0.07f

static int16_t gyro_z_offset = 0;
static i2c_type *i2c_x;
static i2c_handle_type hi2cx;

static void wk_i2c2_init(void)
{
  crm_periph_clock_enable(CRM_I2C2_PERIPH_CLOCK, TRUE);

  gpio_init_type gpio_init_struct;

  gpio_default_para_init(&gpio_init_struct);

  /* add user code begin i2c2_init 1 */

  /* add user code end i2c2_init 1 */

  /* configure the SCL pin */
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE11, GPIO_MUX_5);
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_OPEN_DRAIN;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init_struct.gpio_pins = GPIO_PINS_11;
  gpio_init(GPIOA, &gpio_init_struct);

  /* configure the SDA pin */
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE12, GPIO_MUX_5);
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_OPEN_DRAIN;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init_struct.gpio_pins = GPIO_PINS_12;
  gpio_init(GPIOA, &gpio_init_struct);

  i2c_init(I2C2, I2C_FSMODE_DUTY_2_1, 100000);
  i2c_own_address1_set(I2C2, I2C_ADDRESS_MODE_7BIT, 0x0);
//   i2c_ack_enable(I2C2, TRUE);
//   i2c_clock_stretch_enable(I2C2, TRUE);
//   i2c_general_call_enable(I2C2, FALSE);
//   i2c_reset(I2C2);
  i2c_enable(I2C2, TRUE);
}


uint8_t lsm6ds3_init(i2c_type *i2c_v)
{
    uint8_t who_am_i;
	uint32_t i;

    i2c_x = i2c_v;
    hi2cx.i2cx = i2c_x;
    i2c_config(&hi2cx);

    wk_i2c2_init();
    i2c_memory_read(&hi2cx, I2C_MEM_ADDR_WIDIH_8, LSM6DS3_I2C_ADDR, LSM6DS3_WHO_AM_I, &who_am_i, 1, 0xFFFFFF);

    if(who_am_i != 0x69)
    {
        return 1;
    }

    uint8_t ctrl1_xl = 0x60;
    uint8_t ctrl2_g = 0x4C;
    uint8_t ctrl3_c = 0x04;
    i2c_memory_write(&hi2cx, I2C_MEM_ADDR_WIDIH_8, LSM6DS3_I2C_ADDR, LSM6DS3_CTRL1_XL, &ctrl1_xl, 1, 0xFFFFFF);
    i2c_memory_write(&hi2cx, I2C_MEM_ADDR_WIDIH_8, LSM6DS3_I2C_ADDR, LSM6DS3_CTRL2_G, &ctrl2_g, 1, 0xFFFFFF);
    i2c_memory_write(&hi2cx, I2C_MEM_ADDR_WIDIH_8, LSM6DS3_I2C_ADDR, LSM6DS3_CTRL3_C, &ctrl3_c, 1, 0xFFFFFF);

    gyro_z_offset = 0;
    for(i = 0; i < 10; i++)
    {
        uint8_t buf[2];
        // lsm6ds3_i2c_read(LSM6DS3_OUTZ_L_G, buf, 2);
        i2c_memory_read(&hi2cx, I2C_MEM_ADDR_WIDIH_8, LSM6DS3_I2C_ADDR, LSM6DS3_OUTZ_L_G, buf, 2, 0xFFFFFF);

        gyro_z_offset += (int16_t)(buf[1] << 8 | buf[0]);
    }
    gyro_z_offset /= 10;

    return 0;
}

uint8_t lsm6ds3_read_gyro_z(int16_t *z_data)
{
    uint8_t buf[2] = {0};

    i2c_status_type status = i2c_memory_read(&hi2cx, I2C_MEM_ADDR_WIDIH_8, LSM6DS3_I2C_ADDR, LSM6DS3_OUTZ_L_G, buf, 2, 0xFFFFFF);
    if(status != I2C_OK)
    {
        return 2;
    }

    *z_data = (int16_t)(buf[1] << 8 | buf[0]) - gyro_z_offset;

    return 1;  // 返回1表示成功
}

float lsm6ds3_convert_to_dps(int16_t raw)
{
    return (float)raw * GYRO_SCALE_2000DPS;
}

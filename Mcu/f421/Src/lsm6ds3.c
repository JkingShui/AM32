#include "lsm6ds3.h"

#define GYRO_SCALE_2000DPS 0.061035f

static int16_t gyro_z_offset = 0;
static i2c_type *i2c_x;


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
  i2c_ack_enable(I2C2, TRUE);
  i2c_clock_stretch_enable(I2C2, TRUE);
  i2c_general_call_enable(I2C2, FALSE);
  i2c_enable(I2C2, TRUE);
}

static void lsm6ds3_i2c_write(uint8_t reg, uint8_t data)
{
    while(i2c_flag_get(i2c_x, I2C_BUSYF_FLAG) != RESET);

    i2c_start_generate(i2c_x);
    while(i2c_flag_get(i2c_x, I2C_STARTF_FLAG) == RESET);

    i2c_7bit_address_send(i2c_x, LSM6DS3_I2C_ADDR, I2C_DIRECTION_TRANSMIT);
    while(i2c_flag_get(i2c_x, I2C_ADDR7F_FLAG) == RESET);
    i2c_flag_clear(i2c_x, I2C_ADDR7F_FLAG);

    while(i2c_flag_get(i2c_x, I2C_TDBE_FLAG) == RESET);
    i2c_data_send(i2c_x, reg);

    while(i2c_flag_get(i2c_x, I2C_TDBE_FLAG) == RESET);
    i2c_data_send(i2c_x, data);

    while(i2c_flag_get(i2c_x, I2C_TDC_FLAG) == RESET);

    i2c_stop_generate(i2c_x);
    i2c_flag_clear(i2c_x, I2C_STOPF_FLAG);
}

static void lsm6ds3_i2c_read(uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t i;

    while(i2c_flag_get(i2c_x, I2C_BUSYF_FLAG) != RESET);

    i2c_ack_enable(i2c_x, TRUE);
    // 发送起始条件（GENSTART=1）
    i2c_start_generate(i2c_x);

    // EV1：起始条件产生完成（STARTF=1），软件先读取STS1，然后将地址写入DT寄存器
    while(i2c_flag_get(i2c_x, I2C_STARTF_FLAG) == RESET);
    i2c_7bit_address_send(i2c_x, LSM6DS3_I2C_ADDR, I2C_DIRECTION_TRANSMIT);

    // EV2：成功匹配到地址（ADDR7F=1），软件先读取STS1，再读取STS2清除ADDR7F位，此时主机进入发送阶段
    while(i2c_flag_get(i2c_x, I2C_ADDR7F_FLAG) == RESET);
    i2c_flag_clear(i2c_x, I2C_ADDR7F_FLAG);

    // EV3：向DT寄存器写入数据，此时数据会被立即送到移位寄存器并释放SCL总线，此时TDBE仍然为1
    while(i2c_flag_get(i2c_x, I2C_TDBE_FLAG) == RESET);
    i2c_data_send(i2c_x, reg);
    // EV4：此时DT数据寄存器空，移位寄存器非空，向DT寄存器写入数据，此时TDBE清零
    // TDBE位在倒数第二个字节发送完成后置起

    // EV5：TDC=1，字节发送结束，主机发送停止条件（STOPF=1），硬件自动清除TDBE位和TDC位
    while(i2c_flag_get(i2c_x, I2C_TDC_FLAG) == RESET);

    // 发送起始条件（GENSTART=1）
    i2c_start_generate(i2c_x);
    // EV1：起始条件产生完成（STARTF=1），软件先读取STS1，然后将地址写入DT寄存器
    while(i2c_flag_get(i2c_x, I2C_STARTF_FLAG) == RESET);


    // EV2：成功匹配到地址（ADDR7F=1），软件先读取STS1，再读取STS2清除ADDR7F位，此时主机进入接收阶段
    i2c_7bit_address_send(i2c_x, LSM6DS3_I2C_ADDR, I2C_DIRECTION_RECEIVE);
    while(i2c_flag_get(i2c_x, I2C_ADDR7F_FLAG) == RESET);
    i2c_flag_clear(i2c_x, I2C_ADDR7F_FLAG);

    if(len == 1)
    {
        // EV4：接收完倒数第二个字节后，软件需立即将ACKEN位清0，GENSTOP位置1
        i2c_ack_enable(i2c_x, FALSE);
        i2c_stop_generate(i2c_x);
        // EV3：在接收到字节后，RDBF位被置1，软件读取数据寄存器(I2C_DT)，RDBF位被清0
        while(i2c_flag_get(i2c_x, I2C_RDBF_FLAG) == RESET);
        data[0] = i2c_data_receive(i2c_x);
        // EV3：在接收到字节后，RDBF位被置1，软件读取数据寄存器(I2C_DT)，RDBF位被清0
    }
    else if(len == 2)
    {
        i2c_ack_enable(i2c_x, FALSE);

        while(i2c_flag_get(i2c_x, I2C_RDBF_FLAG) == RESET);
        data[0] = i2c_data_receive(i2c_x);

        i2c_stop_generate(i2c_x);

        while(i2c_flag_get(i2c_x, I2C_RDBF_FLAG) == RESET);
        data[1] = i2c_data_receive(i2c_x);
    }
    else
    {
        i = 0;
        while(i < len)
        {
            while(i2c_flag_get(i2c_x, I2C_RDBF_FLAG) == RESET);
            data[i] = i2c_data_receive(i2c_x);

            if(i == len - 3)
            {
                i2c_ack_enable(i2c_x, FALSE);
            }
            else if(i == len - 1)
            {
                i2c_stop_generate(i2c_x);
            }

            i++;
        }
    }

    i2c_flag_clear(i2c_x, I2C_STOPF_FLAG);
    i2c_ack_enable(i2c_x, TRUE);
}

uint8_t lsm6ds3_init(i2c_type *i2c_v)
{
    uint8_t who_am_i;
	uint32_t i;

    i2c_x = i2c_v;

    wk_i2c2_init();

    lsm6ds3_i2c_read(LSM6DS3_WHO_AM_I, &who_am_i, 1);
    if(who_am_i != 0x69)
    {
        return 1;
    }

    lsm6ds3_i2c_write(LSM6DS3_CTRL1_XL, 0x00);
    lsm6ds3_i2c_write(LSM6DS3_CTRL2_G, 0x4C);
    lsm6ds3_i2c_write(LSM6DS3_CTRL3_C, 0x04);


    gyro_z_offset = 0;
    for(i = 0; i < 10; i++)
    {
        uint8_t buf[2];
        lsm6ds3_i2c_read(LSM6DS3_OUTZ_L_G, buf, 2);
        gyro_z_offset += (int16_t)(buf[1] << 8 | buf[0]);
    }
    gyro_z_offset /= 10;

    return 0;
}

uint8_t lsm6ds3_read_gyro_z(int16_t *z_data)
{
    uint8_t buf[2];

    lsm6ds3_i2c_read(LSM6DS3_OUTZ_L_G, buf, 2);

    *z_data = (int16_t)(buf[1] << 8 | buf[0]) - gyro_z_offset;

    return 0;
}

float lsm6ds3_convert_to_dps(int16_t raw)
{
    return (float)raw * GYRO_SCALE_2000DPS;
}

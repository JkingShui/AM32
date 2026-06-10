#include "lsm6ds3.h"
#include "i2c_application.h"
#include "main.h"
#include "eeprom.h"

static int16_t gyro_z_offset = 0;
i2c_type *i2c_x;
i2c_handle_type hi2cx;
uint8_t buf_arry[2];
volatile int16_t gyro_buf;
extern EEprom_t eepromBuffer;

void i2c_lowlevel_init(i2c_handle_type* hi2c)
{
    crm_periph_clock_enable(CRM_I2C2_PERIPH_CLOCK, TRUE);
    // scl
    gpio_init_type gpio_init_struct;
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_OPEN_DRAIN;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
    gpio_init_struct.gpio_pins = GPIO_PINS_11;
    gpio_init(GPIOA, &gpio_init_struct);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE11, GPIO_MUX_5);
    // sda
    gpio_init_struct.gpio_pins = GPIO_PINS_12;
    gpio_init(GPIOA, &gpio_init_struct);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE12, GPIO_MUX_5);
    // nvic
    nvic_irq_enable(DMA1_Channel5_4_IRQn, 0, 0);
    nvic_irq_enable(I2C2_EVT_IRQn, 0, 0);
    nvic_irq_enable(I2C2_ERR_IRQn, 0, 0);
    // dma
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
    hi2c->dma_rx_channel = DMA1_CHANNEL5;
    dma_reset(hi2c->dma_rx_channel);
    dma_default_para_init(&hi2c->dma_init_struct);
    hi2c->dma_init_struct.peripheral_inc_enable    = FALSE;
    hi2c->dma_init_struct.memory_inc_enable        = TRUE;
    hi2c->dma_init_struct.peripheral_data_width    = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
    hi2c->dma_init_struct.memory_data_width        = DMA_MEMORY_DATA_WIDTH_BYTE;
    hi2c->dma_init_struct.loop_mode_enable         = FALSE;
    hi2c->dma_init_struct.priority                 = DMA_PRIORITY_LOW;
    hi2c->dma_init_struct.direction                = DMA_DIR_MEMORY_TO_PERIPHERAL;
    dma_init(hi2c->dma_rx_channel, &hi2c->dma_init_struct);

    i2c_init(I2C2, I2C_FSMODE_DUTY_2_1, 100000);
}


uint8_t lsm6ds3_init()
{
    uint8_t buf[2];

    uint8_t who_am_i;
	uint32_t i;


    hi2cx.i2cx = I2C2;
    i2c_config(&hi2cx);

    i2c_memory_read(&hi2cx, I2C_MEM_ADDR_WIDIH_8, LSM6DS3_I2C_ADDR, LSM6DS3_WHO_AM_I, &who_am_i, 1, 10000);
    // i2c_memory_read_dma(&hi2cx, I2C_MEM_ADDR_WIDIH_8, LSM6DS3_I2C_ADDR, LSM6DS3_WHO_AM_I, &who_am_i, 1, 10000);

    if(who_am_i != 0x69)
    {
        // 初始化失败
        uart_print_string("lsm6ds3_init failed\n");
        uart_print_string("who_am_i = 0x");
        uart_print_number(who_am_i);
        uart_print_string("\n");
        while(1);
    }

    uint8_t ctrl1_xl = 0x4A;
    uint8_t ctrl2_g = 0x4C;
    uint8_t ctrl3_c = 0x04;
    i2c_memory_write(&hi2cx, I2C_MEM_ADDR_WIDIH_8, LSM6DS3_I2C_ADDR, LSM6DS3_CTRL1_XL, &ctrl1_xl, 1, 10000);
    i2c_memory_write(&hi2cx, I2C_MEM_ADDR_WIDIH_8, LSM6DS3_I2C_ADDR, LSM6DS3_CTRL2_G, &ctrl2_g, 1, 10000);
    i2c_memory_write(&hi2cx, I2C_MEM_ADDR_WIDIH_8, LSM6DS3_I2C_ADDR, LSM6DS3_CTRL3_C, &ctrl3_c, 1, 10000);

    gyro_z_offset = 0;
    for(i = 0; i < 10; i++)
    {
        // lsm6ds3_i2c_read(LSM6DS3_OUTZ_L_G, buf, 2);
        i2c_memory_read(&hi2cx, I2C_MEM_ADDR_WIDIH_8, LSM6DS3_I2C_ADDR, LSM6DS3_OUTZ_L_G, buf, 2, 10000);

        gyro_z_offset += (int16_t)(buf[1] << 8 | buf[0]);
    }
    gyro_z_offset /= 10;

    // 初始化后可以dma读取陀螺仪
    hi2cx.status = I2C_END;

    return 0;
}

uint8_t lsm6ds3_read_gyro_z(int16_t *z_data)
{
    if (hi2cx.status == I2C_END)
    {
        gyro_buf = (buf_arry[1] << 8) | buf_arry[0];
        *z_data = gyro_buf - gyro_z_offset;
        // 下一次读取
        i2c_status_type status = i2c_memory_read_dma(&hi2cx, I2C_MEM_ADDR_WIDIH_8, LSM6DS3_I2C_ADDR, LSM6DS3_OUTZ_L_G, buf_arry, 2, 10000);
        if(status != I2C_OK)
        {
            return 2;
        }
        return 1;
    } else {
        return 0;  // 返回0代表等待中
    }
   
    
}

float lsm6ds3_convert_to_dps(int16_t raw)
{
    // 500为满量程
    float dps = (float)raw * GYRO_SCALE;
    return eepromBuffer.gyro.reverse ? -dps : dps;
}

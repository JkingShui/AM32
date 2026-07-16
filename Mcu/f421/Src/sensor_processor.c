#include "sensor_processor.h"
#include "lsm6ds3.h"
#include "at32f421_tmr.h"
#include "peripherals.h"
#include "targets.h"
#include "uart_print.h"
#include "functions.h"

extern int32_t smoothed_raw_current;

/**
 * @brief 获取当前时间与上次时间的间隔（微秒）
 * 
 * UTILITY_TIMER配置：
 * - 预分频器: div = 119
 * - CPU频率: 120MHz
 * - 定时器频率: 120MHz / 120 = 1MHz
 * - 每个计数: 1微秒
 * - 最大值: 0xFFFF = 65535 (约65.5ms)
 * 
 * @param last_time 上次时间戳（UTILITY_TIMER->cval的值）
 * @return uint32_t 时间间隔（微秒），正确处理溢出情况
 */
uint32_t get_time_interval_us(uint32_t last_time)
{
    uint32_t this_time = UTILITY_TIMER->cval;
    
    if (this_time >= last_time) {
        // 正常情况：当前时间 >= 上次时间
        return this_time - last_time;
    } else {
        // 溢出情况：定时器从0xFFFF溢出到0
        // 间隔 = (0xFFFF - last_time) + this_time + 1
        return (0xFFFF - last_time) + this_time + 1;
    }
}

/**
 * @brief PWM输入值双区间线性映射并滤波
 * 
 * 输入范围：500-2500 us（标准舵机信号）
 * 映射规则：
 *   servo_range_a → servo_mid  → 映射到 -500 → 0
 *   servo_mid     → servo_range_b → 映射到 0 → 500
 * 
 * @param pwm_high_time 输入PWM高电平时间（500-2500 us）
 * @param servo_range_a 范围A（通常是1000 us）
 * @param servo_mid     中位（通常是1500 us）
 * @param servo_range_b 范围B（通常是2000 us）
 * @param filter_alpha  滤波系数（0-100，越大响应越快，越小滤波越强）
 * @return int32_t 映射后的结果（-500 到 500）
 */
int32_t map_pwm_to_slider(uint32_t pwm_high_time, 
                          uint16_t servo_range_a, 
                          uint16_t servo_mid, 
                          uint16_t servo_range_b,
                          uint8_t filter_alpha)
{
    static int32_t filtered_output = 0;  // 滤波后的值
    int32_t raw_output;
    
    // 输入范围限制（500-2500 us）
    if (pwm_high_time < 500) pwm_high_time = 500;
    if (pwm_high_time > 2500) pwm_high_time = 2500;
    
    // 双区间线性映射
    if (pwm_high_time < servo_mid) {
        raw_output = map(pwm_high_time, servo_mid - servo_range_a, servo_mid, -500, 0);
    } else if (pwm_high_time > servo_mid) {
        raw_output = map(pwm_high_time, servo_mid, servo_mid + servo_range_b, 0, 500);
    } else {
        raw_output = 0;
    }
    
    // 输出范围限制
    if (raw_output < -500) raw_output = -500;
    if (raw_output > 500) raw_output = 500;
    
    // 一阶低通滤波：filtered = alpha * raw + (1 - alpha) * filtered
    // 使用整数运算避免浮点
    filtered_output = (filter_alpha * raw_output + (100 - filter_alpha) * filtered_output) / 100;
    
    return filtered_output;
}

static sensor_data_t sensor_data;
extern uint16_t ADC_raw_volts;
extern uint16_t ADCDataDMA[4];
extern uint16_t ADC_raw_current;

volatile uint8_t pwm2_data_ready = 0;
uint32_t pwm2_capture_high_time = 0;

void sensor_processor_init(void) {
    sensor_data.gyro_z = 0;
    sensor_data.gyro_z_dps = 0.0f;
    sensor_data.pwm_high_time = 0;
    sensor_data.output_pwm = 0;
    sensor_data.gain = 0.0f;
}

volatile uint16_t loop_time = 0;
// TODO 降低更新频率，降低的值还需要确定
void sensor_processor_update_gyro(void) {
    // 定时更新一次
    uint8_t ret = lsm6ds3_read_gyro_z(&sensor_data.gyro_z);
    if(ret == 1)
    {
        sensor_data.gyro_z_dps = lsm6ds3_convert_to_dps(sensor_data.gyro_z);
    } else if(ret == 0)
    {
        // 0代表等待中，不处理
    } else
    {
        // 其他错误码，设置为0
        sensor_data.gyro_z_dps = 0.0f;
    }
        
}

void sensor_processor_update_pwm_input(void) {
    // 手轮输入
    if (pwm2_data_ready) {
        sensor_data.pwm_high_time = pwm2_capture_high_time;
        pwm2_data_ready = 0;
    }
    // 感度，从原来电流读取 TODO 可以滤波重一点
    sensor_data.gain = 3.0f * smoothed_raw_current / 4096.0f;
}

// TODO 看下循环时间耗时
volatile uint32_t last_calculate_time = 0;
volatile float filtered_error = 0.0f;
void sensor_processor_calculate(void) {
    
    uint32_t this_calculate_time = UTILITY_TIMER->cval;
    uint32_t gap_time = get_time_interval_us(last_calculate_time);// 时间间隔，微秒
    
    // 误差 = 手轮 - 陀螺仪 - 中间值
    int32_t input = map_pwm_to_slider(sensor_data.pwm_high_time, eepromBuffer.gyro.servo_range_a, eepromBuffer.gyro.servo_mid, eepromBuffer.gyro.servo_range_b, 30);
    float effective_gain = (sensor_data.gain > 0.01f) ? sensor_data.gain : 1.0f;
    float error = input / effective_gain - sensor_data.gyro_z_dps - sensor_data.slider;
    
    // 积分项
    sensor_data.integral += 10.0f * gap_time / 1000000 * error;
    // 积分项限制
    if (sensor_data.integral > 500) {
        sensor_data.integral = 500;
    }
    if (sensor_data.integral < -500) {
        sensor_data.integral = -500;
    }

    filtered_error = 0.3 * filtered_error + 0.7 * error;

    // pi运算
    sensor_data.slider = 0.8f * filtered_error + sensor_data.integral;

    // 输出，映射回去
    int32_t output_pwm_value = sensor_data.slider * effective_gain;
    // TODO 测试gain 设置1
    // int32_t output_pwm_value = sensor_data.slider;
    if (output_pwm_value > 0) {
        sensor_data.output_pwm = map(output_pwm_value, 0, 500, eepromBuffer.gyro.servo_mid, eepromBuffer.gyro.servo_mid + eepromBuffer.gyro.servo_range_b);
    } else {
        sensor_data.output_pwm = map(output_pwm_value, -500, 0, eepromBuffer.gyro.servo_mid - eepromBuffer.gyro.servo_range_a, eepromBuffer.gyro.servo_mid);
    }

    // 串口输出：pwm_high_time, gyro_z_dps, output_pwm_value （CSV逗号分隔，gyro保留2位小数）
    // 每100次输出一次，避免串口阻塞影响控制频率
    static uint16_t uart_print_cnt = 0;
    if (++uart_print_cnt >= 100) {
        uart_print_cnt = 0;
        uart_print_number((int32_t)sensor_data.pwm_high_time);
        uart_print_char(',');
        uart_print_number((int32_t)output_pwm_value);
        uart_print_char(',');
        {
            float g = sensor_data.gain;
            int32_t gi = (int32_t)g;
            int32_t gf = (int32_t)((g - (float)gi) * 100.0f);
            if (gf < 0) gf = -gf;
            uart_print_number(gi);
            uart_print_char('.');
            if (gf < 10) uart_print_char('0');
            uart_print_number(gf);
        }
        uart_print_string("\r\n");
    }

    last_calculate_time = this_calculate_time;
    loop_time = gap_time;
}

void sensor_processor_update_output(void) {
    tmr_channel_value_set(TMR15, TMR_SELECT_CHANNEL_1, sensor_data.output_pwm);
}

sensor_data_t* sensor_processor_get_data(void) {
    return &sensor_data;
}
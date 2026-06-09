#ifndef __SENSOR_PROCESSOR_H
#define __SENSOR_PROCESSOR_H

#include <stdint.h>
#include "main.h"
#include "common.h"

typedef struct {
    float slider;// 中间值
    float integral;// 积分项
    int16_t gyro_z;// 陀螺仪原始数据
    float gyro_z_dps;// 陀螺仪角度
    uint32_t pwm_high_time;// 输入高电平时间，us
    uint16_t output_pwm;// 输出PWM值，us
    float gain;// 感度
} sensor_data_t;


/**
 * @brief 获取当前时间与上次时间的间隔（微秒）
 * @param last_time 上次时间戳（UTILITY_TIMER->cval的值）
 * @return uint32_t 时间间隔（微秒）
 */
uint32_t get_time_interval_us(uint32_t last_time);

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
                          uint8_t filter_alpha);

void sensor_processor_init(void);
void sensor_processor_update_gyro(void);
void sensor_processor_update_pwm_input(void);
void sensor_processor_calculate(void);
void sensor_processor_update_output(void);
sensor_data_t* sensor_processor_get_data(void);

extern volatile uint8_t pwm2_data_ready;
extern uint32_t pwm2_capture_high_time;

#endif
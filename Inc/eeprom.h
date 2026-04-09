/**
 * @file eeprom.h
 * @brief EEPROM数据结构定义和操作函数声明
 * 
 * 该文件定义了EEPROM数据结构，用于存储ESC（电子调速器）的配置参数，
 * 包括电机参数、控制参数、保护参数等。同时声明了EEPROM操作相关的函数。
 */
#include "main.h"

#pragma once

/**
 * @brief EEPROM数据结构
 * 
 * 该联合体定义了EEPROM的数据结构，包含了ESC的所有配置参数。
 * 可以通过结构体成员访问具体参数，也可以通过buffer数组进行整体操作。
 */
typedef union EEprom_u {
    struct {
        uint8_t reserved_0; //0 保留字段
        uint8_t eeprom_version; //1 EEPROM版本号
        uint8_t reserved_1; //2 保留字段
        struct {
            uint8_t major; //3 固件主版本号
            uint8_t minor; //4 固件次版本号
        } version;
        uint8_t max_ramp;    //5 最大油门斜率（0.1%/ms 到 25%/ms）
        uint8_t minimum_duty_cycle; //6 最小占空比（0.2% 到 51%）
        uint8_t disable_stick_calibration; //7 禁用摇杆校准（0=启用，1=禁用）
        uint8_t absolute_voltage_cutoff;  //8 绝对电压截止值（1-100，以0.5V为增量）
        uint8_t current_P; //9 电流PID控制器P参数（0-255）
        uint8_t current_I; //10 电流PID控制器I参数（0-255）
        uint8_t current_D; //11 电流PID控制器D参数（0-255）
        uint8_t active_brake_power; //12 主动制动功率（1-5%占空比）
        char reserved_eeprom_3[4]; //13-16 保留字段
        uint8_t dir_reversed; // 17 方向反转（0=正常，1=反转）
        uint8_t bi_direction; // 18 双向模式（0=单向，1=双向）
        uint8_t use_sine_start; // 19 使用正弦启动（0=禁用，1=启用）
        uint8_t comp_pwm; // 20 互补PWM模式（0=禁用，1=启用）
        uint8_t variable_pwm; // 21 可变PWM频率（0=禁用，1=启用）
        uint8_t stuck_rotor_protection; // 22 卡转子保护级别（0-3，0=禁用）
        uint8_t advance_level; // 23 提前角度级别（0-10）
        uint8_t pwm_frequency; // 24 PWM频率设置（0-7，对应不同频率）
        uint8_t startup_power; // 25 启动功率（0-100%）
        uint8_t motor_kv; // 26 电机KV值（100-2000）
        uint8_t motor_poles; // 27 电机极对数（2-20）
        uint8_t brake_on_stop; // 28 停止时制动（0=禁用，1=启用拖拽制动，2=启用完全制动）
        uint8_t stall_protection; // 29 失速保护级别（0-3，0=禁用）
        uint8_t beep_volume; // 30 蜂鸣音量（0-100%）
        uint8_t telemetry_on_interval; // 31 遥测间隔（0-255ms）
        struct {
            uint8_t low_threshold; // 32  servo低阈值（1000-2000us）
            uint8_t high_threshold; // 33  servo高阈值（1000-2000us）
            uint8_t neutral; // 34  servo中立点（1000-2000us）
            uint8_t dead_band; // 35  servo死区（0-255us）
        } servo; // 伺服信号配置
        uint8_t low_voltage_cut_off; // 36 低压截止（0-100%，基于电池类型）
        uint8_t low_cell_volt_cutoff; // 37 低电芯电压截止（0-100%，基于电芯类型）
        uint8_t rc_car_reverse; // 38 RC车反转模式（0=禁用，1=启用）
        uint8_t use_hall_sensors; // 39 使用霍尔传感器（0=禁用，1=启用）
        uint8_t sine_mode_changeover_thottle_level; // 40 正弦模式切换油门级别（0-100）
        uint8_t drag_brake_strength; // 41 拖拽制动强度（0-100%）
        uint8_t driving_brake_strength; // 42 行车制动强度（0-100%）
        struct {
            uint8_t temperature; // 43 温度限制（0-120°C）
            uint8_t current; // 44 电流限制（0-100A）
        } limits; // 保护限制
        uint8_t sine_mode_power; // 45 正弦模式功率（0-100%）
        uint8_t input_type; // 46 输入类型（0=PWM，1=DShot）
        uint8_t auto_advance; // 47 自动提前角度（0=禁用，1=启用）
        uint8_t tune[128]; // 48-175 调优参数
        struct {
            uint8_t can_node; // 176 CAN节点ID
            uint8_t esc_index; // 177 ESC索引
            uint8_t require_arming; // 178 需要arming（0=禁用，1=启用）
            uint8_t telem_rate; // 179 遥测速率（0-255ms）
            uint8_t require_zero_throttle; // 180 需要零油门（0=禁用，1=启用）
            uint8_t filter_hz; // 181 滤波器频率（0-1000Hz）
            uint8_t debug_rate; // 182 调试速率（0-255ms）
            uint8_t term_enable; // 183 终端启用（0=禁用，1=启用）
            uint8_t reserved[8]; // 184-191 保留字段
        } can; // CAN总线配置
    };
    uint8_t buffer[192]; // 原始数据缓冲区
} EEprom_t;

/**
 * @brief EEPROM缓冲区
 * 
 * 全局EEPROM数据缓冲区，用于存储和访问ESC的配置参数。
 */
extern EEprom_t eepromBuffer;

// void save_to_flash(uint8_t *data);
// void read_flash(uint8_t* data, uint32_t address);
// void save_to_flash_bin(uint8_t *data, int length, uint32_t add);

/**
 * @brief 从FLASH中读取二进制数据
 * 
 * @param data 指向目标缓冲区的指针，用于存储读取的数据
 * @param add FLASH的起始地址
 * @param out_buff_len 要读取的数据长度（字节）
 */
void read_flash_bin(uint8_t* data, uint32_t add, int out_buff_len);

/**
 * @brief 无库依赖地将数据保存到FLASH
 * 
 * @param data 指向要保存的数据的指针
 * @param length 要保存的数据长度（字节）
 * @param add FLASH的起始地址
 */
void save_flash_nolib(uint8_t* data, int length, uint32_t add);

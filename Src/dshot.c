/*
 * dshot.c
 *
 *  Created on: Apr. 22, 2020
 *      Author: Alka
 */

#include "dshot.h"
#include "IO.h"
#include "common.h"
#include "functions.h"
#include "sounds.h"
#include "targets.h"
#if DRONECAN_SUPPORT
#include "DroneCAN/DroneCAN.h"
#endif

int dpulse[16] = { 0 };

const char gcr_encode_table[16] = {
    0b11001, 0b11011, 0b10010, 0b10011, 0b11101, 0b10101, 0b10110, 0b10111,
    0b11010, 0b01001, 0b01010, 0b01011, 0b11110, 0b01101, 0b01110, 0b01111
};

typedef struct {
    uint16_t temp_count;
    uint16_t voltage_count;
    uint16_t current_count;
    uint8_t last_sent_extended;
} dshot_telem_scheduler_t;

static dshot_telem_scheduler_t telem_scheduler = {0};

// These divisors create ratios regardless of input rate:
// - Temperature: every 200 calls (4Hz at 800Hz input)
// - Voltage: every 200 calls (4Hz at 800Hz input)
// - Current: every 40 calls (20Hz at 800Hz input)
// - eRPM: fills all other slots

#define TEMP_EDT_RATE_DIVISOR    200
#define VOLTAGE_EDT_RATE_DIVISOR 200
#define CURRENT_EDT_RATE_DIVISOR 40


char send_EDT_init;
char send_EDT_deinit;
char EDT_ARM_ENABLE = 0;
char EDT_ARMED = 0;
int shift_amount = 0;
uint32_t gcrnumber;
extern int zero_crosses;
extern char send_telemetry;
extern uint8_t max_duty_cycle_change;
int dshot_full_number;
extern char play_tone_flag;
extern char send_esc_info_flag;
uint8_t command_count = 0;
uint8_t last_command = 0;
uint8_t high_pin_count = 0;
uint32_t gcr[37] = { 0 };
uint16_t dshot_frametime;
uint16_t dshot_goodcounts;
uint16_t dshot_badcounts;
uint8_t dshot_extended_telemetry = 0;
uint16_t processtime = 0;
uint16_t halfpulsetime = 0;

uint8_t programming_mode;
uint16_t position;
uint8_t  new_byte;

/**
 * @brief 解析 DShot 协议输入信号
 * 
 * 该函数通过分析 DMA 捕获的脉冲时间序列，解码 DShot 协议的命令和数据，
 * 并执行相应的操作，如电机控制、命令处理、遥测数据处理等。
 */
void computeDshotDMA()
{
    // 计算帧时间（整个 DShot 帧的持续时间）
    dshot_frametime = dma_buffer[31] - dma_buffer[0];
    
    // 计算半脉冲时间（用于判断位值的阈值）
    halfpulsetime = dshot_frametime >> 5;
    
    // 验证帧时间是否在有效范围内
    if ((dshot_frametime > dshot_frametime_low) && (dshot_frametime < dshot_frametime_high)) {
        // 重置信号超时计数器
        signaltimeout = 0;
        
        // 解析 16 个脉冲的宽度，确定每个位的值
        for (int i = 0; i < 16; i++) {
            // 计算相邻两个捕获值的差（脉冲宽度）
            // 转换为 uint16_t 以正确处理定时器溢出
            const uint16_t pdiff = dma_buffer[(i << 1) + 1] - dma_buffer[(i << 1)];
            
            // 根据脉冲宽度判断位值（大于半脉冲时间为 1，否则为 0）
            dpulse[i] = (pdiff > halfpulsetime);
        }
        
        // 计算 CRC 校验值
        uint8_t calcCRC = ((dpulse[0] ^ dpulse[4] ^ dpulse[8]) << 3 | 
                          (dpulse[1] ^ dpulse[5] ^ dpulse[9]) << 2 | 
                          (dpulse[2] ^ dpulse[6] ^ dpulse[10]) << 1 | 
                          (dpulse[3] ^ dpulse[7] ^ dpulse[11]));
        
        // 提取接收到的 CRC 校验值
        uint8_t checkCRC = (dpulse[12] << 3 | dpulse[13] << 2 | dpulse[14] << 1 | dpulse[15]);
        
        // 处理遥测反转（在未武装状态下检测）
        if (!armed) {
            if (dshot_telemetry == 0) {
                // 检查输入引脚状态，判断是否需要反转遥测
                if (getInputPinState()) {
                    high_pin_count++;
                    if (high_pin_count > 100) {
                        dshot_telemetry = 1;
                    }
                }
            }
        }
        
        // 如果遥测反转，对 CRC 进行处理
        if (dshot_telemetry) {
            checkCRC = ~checkCRC + 16;
        }
        
        // 从位值中提取命令值（11 位）
        // tocheck为11位油门，其中0-47代表指令，48-2047代表油门
        int tocheck = (dpulse[0] << 10 | dpulse[1] << 9 | dpulse[2] << 8 | 
                      dpulse[3] << 7 | dpulse[4] << 6 | dpulse[5] << 5 | 
                      dpulse[6] << 4 | dpulse[7] << 3 | dpulse[8] << 2 | 
                      dpulse[9] << 1 | dpulse[10]);
        
        // 验证 CRC 校验
        if (calcCRC == checkCRC) {
            // 重置信号超时计数器
            signaltimeout = 0;
            // 增加成功计数
            dshot_goodcounts++;
            
            // 检查是否需要发送遥测数据
            if (dpulse[11] == 1) {
                send_telemetry = 1;
            }
            
            // 处理编程模式
            if(programming_mode > 0){  
                if(programming_mode == 1){ // 开始编程模式
                    position = tocheck;    // 设置 EEPROM 缓冲区位置
                    programming_mode = 2;
                    return;
                }
               if(programming_mode == 2){ // 接收新的设置值
                    new_byte = tocheck;   // 存储新的设置值
                    programming_mode = 3;
                    return;
                }
                if(programming_mode == 3){ // 提交设置值
                    if(tocheck == 37){  // 确认提交命令
                        eepromBuffer.buffer[position] = new_byte;
                        programming_mode = 0;
                    }
                }
                return; // 编程模式下不处理 DShot 信号
            }
            
            // 处理电机控制信号（值大于 47）
            if (tocheck > 47) {
                if (EDT_ARMED) {
                    newinput = tocheck;  // 设置新的输入值
                    dshotcommand = 0;    // 清除命令
                    command_count = 0;    // 重置命令计数器
                    return;
                }
            }
            
            // 处理 DShot 命令（值在 1-47 之间）
            if ((tocheck <= 47) && (tocheck > 0)) {
                newinput = 0;          // 清除输入值
                dshotcommand = tocheck; // 设置命令值
            }
            
            // 处理零值命令（通常是停转命令）
            if (tocheck == 0) {
                if (EDT_ARM_ENABLE == 1) {
                    EDT_ARMED = 0;  // 解除武装
                }
#if DRONECAN_SUPPORT
                if (DroneCAN_active()) {
                    // 允许 DroneCAN 覆盖 DShot 输入
                    return;
                }
#endif
                newinput = 0;          // 清除输入值
                dshotcommand = 0;      // 清除命令
                command_count = 0;      // 重置命令计数器
            }
            
            // 处理 DShot 命令（当电机未运行且已武装时）
            if ((dshotcommand > 0) && (running == 0) && armed) {
                if (dshotcommand != last_command) {
                    last_command = dshotcommand;
                    command_count = 0;
                }
                // 1-5，电调鸣叫，低频->高频
                if (dshotcommand <= 5) { // 信标命令，立即执行
                    command_count = 6;
                }
                command_count++;
                if (command_count >= 6) { // 命令重复 6 次后执行
                    command_count = 0;
                    
                    // 处理具体命令
                    switch (dshotcommand) {
                    case 1: // 播放音调 1
                        play_tone_flag = 1;
                        break;
                    case 2: // 播放音调 2
                        play_tone_flag = 2;
                        break;
                    case 3: // 播放音调 3
                        play_tone_flag = 3;
                        break;
                    case 4: // 播放音调 4
                        play_tone_flag = 4;
                        break;
                    case 5: // 播放音调 5
                        play_tone_flag = 5;
                        break;
                    case 6: // 发送电调信息
                        send_esc_info_flag = 1;
                        break;
                    case 7: // 设置正向旋转
                        eepromBuffer.dir_reversed = 0;
                        forward = 1 - eepromBuffer.dir_reversed;
                        break;
                    case 8: // 设置反向旋转
                        eepromBuffer.dir_reversed = 1;
                        forward = 1 - eepromBuffer.dir_reversed;
                        break;
                    case 9: // 禁用双向模式
                        eepromBuffer.bi_direction = 0;
                        break;
                    case 10: // 启用双向模式
                        eepromBuffer.bi_direction = 1;
                        break;
                    case 12: // 保存设置到 EEPROM
                        saveEEpromSettings();
                        play_tone_flag = 1 + eepromBuffer.dir_reversed;
                        break;
                    case 13: // 启用扩展遥测
                        dshot_extended_telemetry = 1;
                        send_EDT_init = 1;
                        if (EDT_ARM_ENABLE == 1) {
                            EDT_ARMED = 1;
                        }
                        break;
                    case 14: // 禁用扩展遥测
                        dshot_extended_telemetry = 0;
                        send_EDT_deinit = 1;
                        break;
                    case 20: // 设置正向旋转（临时）
                        forward = 1 - eepromBuffer.dir_reversed;
                        break;
                    case 21: // 设置反向旋转（临时）
                        forward = eepromBuffer.dir_reversed;
                        break;
                    case 36: // 进入编程模式
                        programming_mode = 1;
                        break;
                    }
                    last_dshot_command = dshotcommand;
                    dshotcommand = 0;
                }
            }
        } else {
            // CRC 校验失败
            dshot_badcounts++;
            programming_mode = 0; // 退出编程模式
        }
    }
}

void make_dshot_package(uint16_t com_time)
{
    uint16_t extended_frame_to_send = 0;

    if (dshot_extended_telemetry) {
        // Only send extended telemetry if last frame wasn't extended. This ensures eRPM interleaving.
        if (telem_scheduler.last_sent_extended) {
            telem_scheduler.last_sent_extended = 0;

        } else {
            telem_scheduler.current_count++;
            telem_scheduler.voltage_count++;
            telem_scheduler.temp_count++;

            if (telem_scheduler.current_count >= CURRENT_EDT_RATE_DIVISOR) {
                extended_frame_to_send = 0b0110 << 8 | (uint8_t)(actual_current / 50);
                telem_scheduler.current_count = 0;
            }
            else if (telem_scheduler.voltage_count >= VOLTAGE_EDT_RATE_DIVISOR) {
                extended_frame_to_send = 0b0100 << 8 | (uint8_t)(battery_voltage / 25);
                telem_scheduler.voltage_count = 0;
            }
            else if (telem_scheduler.temp_count >= TEMP_EDT_RATE_DIVISOR) {
                extended_frame_to_send = 0b0010 << 8 | (uint8_t)degrees_celsius;
                telem_scheduler.temp_count = 0;
            }
        }
    }
      if(send_EDT_init){
        extended_frame_to_send = 0b111000000000;
        send_EDT_init = 0;
      }
      if(send_EDT_deinit){
        extended_frame_to_send = 0b111011111111;
        send_EDT_deinit = 0;
      }
    
    if (extended_frame_to_send > 0) {
        dshot_full_number = extended_frame_to_send;
        telem_scheduler.last_sent_extended = 1;

    } else {
        if (!running) {
            com_time = 65535;
        }
        //	calculate shift amount for data in format eee mmm mmm mmm, first 1 found
        // in first seven bits of data determines shift amount
        // this allows for a range of up to 65408 microseconds which would be
        // shifted 0b111 (eee) or 7 times.
        for (int i = 15; i >= 9; i--) {
            if (com_time >> i == 1) {
                shift_amount = i + 1 - 9;
                break;
            } else {
                shift_amount = 0;
            }
        }
        // shift the commutation time to allow for expanded range and put shift
        // amount in first three bits
        dshot_full_number = ((shift_amount << 9) | (com_time >> shift_amount));
    }
    // calculate checksum
    uint16_t csum = 0;
    uint16_t csum_data = dshot_full_number;
    for (int i = 0; i < 3; i++) {
        csum ^= csum_data; // xor data by nibbles
        csum_data >>= 4;
    }
    csum = ~csum; // invert it
    csum &= 0xf;

    dshot_full_number = (dshot_full_number << 4) | csum; // put checksum at the end of 12 bit dshot number

    // GCR RLL encode 16 to 20 bit

    gcrnumber = gcr_encode_table[(dshot_full_number >> 12)]
            << 15 // first set of four digits
        | gcr_encode_table[(((1 << 4) - 1) & (dshot_full_number >> 8))]
            << 10 // 2nd set of 4 digits
        | gcr_encode_table[(((1 << 4) - 1) & (dshot_full_number >> 4))]
            << 5 // 3rd set of four digits
        | gcr_encode_table[(((1 << 4) - 1) & (dshot_full_number >> 0))]; // last four digits
// GCR RLL encode 20 to 21bit output
#if defined(MCU_F051) || defined(MCU_F031) || defined(MCU_CH32V203)
    gcr[1 + buffer_padding] = 64;
    for (int i = 19; i >= 0; i--) { // each digit in gcrnumber
        gcr[buffer_padding + 20 - i + 1] = ((((gcrnumber & 1 << i)) >> i) ^ (gcr[buffer_padding + 20 - i] >> 6))
            << 6; // exclusive ored with number before it multiplied by 64 to match
                  // output timer.
    }
    gcr[buffer_padding] = 0;
#else
    gcr[1 + buffer_padding] = 128;
    for (int i = 19; i >= 0; i--) { // each digit in gcrnumber
        gcr[buffer_padding + 20 - i + 1] = ((((gcrnumber & 1 << i)) >> i) ^ (gcr[buffer_padding + 20 - i] >> 7))
            << 7; // exclusive ored with number before it multiplied by 64 to match
                  // output timer.
    }
    gcr[buffer_padding] = 0;
#endif
}

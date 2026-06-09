/*
 * peripherals.h
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

#ifndef PERIPHERALS_H_
#define PERIPHERALS_H_

void PB5_TMR3_CH2_Init(void);
void PB5_TMR3_CH2_Resume(void);

#endif /* PERIPHERALS_H_ */

#include "main.h"
#define INTERVAL_TIMER_COUNT (INTERVAL_TIMER->cval)
#define RELOAD_WATCHDOG_COUNTER() (WDT->cmd = WDT_CMD_RELOAD)
// 倒计时换相定时器
#define DISABLE_COM_TIMER_INT() (COM_TIMER->iden &= ~TMR_OVF_INT)
#define ENABLE_COM_TIMER_INT() (COM_TIMER->iden |= TMR_OVF_INT)
// 设置COM定时器中断时间并使能中断
// COM_TIMER->cval = 0 ：重置定时器计数器值为0
// 定时器将在计数达到 time 值时产生溢出中断
// COM_TIMER->ists = 0x00 ：清除中断状态标志
// COM_TIMER->iden |= TMR_OVF_INT ：启用定时器溢出中断
#define SET_AND_ENABLE_COM_INT(time)                                    \
    (COM_TIMER->cval = 0, COM_TIMER->pr = time, COM_TIMER->ists = 0x00, \
        COM_TIMER->iden |= TMR_OVF_INT)
#define SET_INTERVAL_TIMER_COUNT(intertime) (INTERVAL_TIMER->cval = intertime)
#define SET_PRESCALER_PWM(presc) (TMR1->div = presc)
#define SET_AUTO_RELOAD_PWM(relval) (TMR1->pr = relval)
#define SET_DUTY_CYCLE_ALL(newdc) \
    (TMR1->c1dt = newdc, TMR1->c2dt = newdc, TMR1->c3dt = newdc)

// TIM15 PWM输出配置 (用于舵机控制)
#define TIM15_PRESCALER     119
#define TIM15_PERIOD        ((120000000 / (TIM15_PRESCALER + 1)) / 333 - 1)
#define TIM15_MAX_CCR       (TIM15_PERIOD + 1)

void initAfterJump(void);
void initCorePeripherals(void);
// void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
// static void MX_ADC_Init(void);
void AT_COMP_Init(void);
void TIM1_Init(void);
void TIM6_Init(void);
void system_clock_config(void);
void MX_IWDG_Init(void);
void TIM17_Init(void);
void TIM14_Init(void);
void TIM15_Init(void);
void TIM16_Init(void);
// static void MX_USART1_UART_Init(void);
void resetInputCaptureTimer();
void setPWMCompare1(uint16_t compareone);
void setPWMCompare2(uint16_t comparetwo);
void setPWMCompare3(uint16_t comparethree);
void enableCorePeripherals(void);
void reloadWatchDogCounter(void);
void generatePwmTimerEvent(void);
void UN_TIM_Init(void);
void LED_GPIO_init(void);
void setIndividualRGBLed(uint8_t red, uint8_t green, uint8_t blue);

// 按钮和LED函数
void button_led_init(void);
uint8_t button_read(void);       // 读取按键状态（带delay消抖）
void led_set(uint8_t state);     // 设置LED状态
void led_blink_1hz(void);        // LED每隔1秒闪一下（非阻塞式）
void led_blink_fast(uint8_t times, uint16_t interval_ms);  // LED快闪N次
void led_blink_fast_3x(void);    // LED快闪3次（默认）

// Flash 保护函数
flash_status_type flash_fap_high_level_protection_enable(void);
flash_status_type flash_fap_high_level_protection_disable(void);
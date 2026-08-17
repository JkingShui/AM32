/*
 * sounds.c
 *
 *  Created on: May 13, 2020
 *      Author: Alka
 */

#include "sounds.h"
#include "common.h"
#include "eeprom.h"
#include "functions.h"
#include "peripherals.h"
#include "phaseouts.h"
#include "targets.h"

#ifndef ERASED_FLASH_BYTE
#define ERASED_FLASH_BYTE  0xFF
#endif

// 音符定义 (基于公式: frequency = 10000000 / (t3 * 247 + 4000))
#define NOTE_C3  262    // 130.81 Hz
#define NOTE_D3  228    // 146.83 Hz
#define NOTE_E3  201    // 164.81 Hz
#define NOTE_F3  187    // 174.61 Hz
#define NOTE_G3  164    // 196.00 Hz
#define NOTE_A3  145    // 220.00 Hz
#define NOTE_B3  128    // 246.94 Hz
#define NOTE_C4  119    // 261.63 Hz
#define NOTE_D4  104    // 293.66 Hz
#define NOTE_E4  92     // 329.63 Hz
#define NOTE_F4  85     // 349.23 Hz
#define NOTE_G4  75     // 392.00 Hz
#define NOTE_A4  66     // 440.00 Hz
#define NOTE_B4  59     // 493.88 Hz
#define NOTE_C5  54     // 523.25 Hz
#define NOTE_D5  47     // 587.33 Hz
#define NOTE_DS5 44     // 622.25 Hz (D#5)
#define NOTE_E5  45     // 659.25 Hz
#define NOTE_F5  38     // 698.46 Hz
#define NOTE_FS5 36     // 739.99 Hz (F#5)
#define NOTE_G5  35     // 783.99 Hz
#define NOTE_A5  30     // 880.00 Hz
#define NOTE_B5  26     // 987.77 Hz
#define NOTE_C6  24     // 1046.50 Hz
#define NOTE_D6  21     // 1174.66 Hz
#define NOTE_DS6 19     // 1244.51 Hz (D#6)
#define NOTE_E6  17     // 1318.56 Hz

// 节拍定义 (基于 150 BPM，增大值减慢速度)
#define NOTE_SIXTEENTH   40    // 十六分音符 (约160ms)
#define NOTE_EIGHTH      80    // 八分音符
#define NOTE_QUARTER    160    // 四分音符
#define NOTE_HALF       320    // 二分音符
#define NOTE_WHOLE      640    // 全音符

// 旋律结构
typedef struct {
    uint8_t note;     // 音符 (t3 值)
    uint16_t beat;    // 节拍 (uint16_t 支持更大的值)
} MelodyNote;

// 函数声明
void playBlueJayTune(void);

uint8_t beep_volume;

// 休止音符定义
#define REST  0    // 休止符（静音）

// 用户自定义旋律 (b=150, o=5, d=16)
const MelodyNote my_melody[] = {
    {NOTE_G5,  NOTE_QUARTER},  // g5
    {REST,     NOTE_SIXTEENTH},  // p
    {NOTE_A5,  NOTE_QUARTER},  // a5
    {REST,     NOTE_SIXTEENTH},  // p
    {NOTE_C6,  NOTE_QUARTER},  // c6
    {REST,     NOTE_SIXTEENTH},  // p
    {NOTE_D6, NOTE_QUARTER},  // d6
    {REST,     NOTE_SIXTEENTH},  // p
    {NOTE_E6,     NOTE_QUARTER},     // e6
    {REST,     NOTE_SIXTEENTH},  // p
    {0, 0}                       // 结束标记
};

// 将旋律数组转换为 tune 格式
void melody_to_tune(const MelodyNote *melody, uint8_t *tune) {
    int i = 4;  // tune[0-3] 是配置，从索引4开始是旋律数据
    
    tune[0] = 1;  // 启用自定义旋律
    tune[1] = 0;
    tune[2] = 0;
    tune[3] = 0;
    
    while (melody->note != 0 || melody->beat != 0) {
        if (i >= 126) break;  // 防止数组越界
        
        // 如果 beat 超过 255，需要拆分
        if (melody->beat > 255) {
            uint16_t remaining = melody->beat;
            while (remaining > 255) {
                tune[i++] = 255;        // t4 = 255，表示继续计数
                tune[i++] = melody->note; // t3 = 音符
                remaining -= 255;
            }
            tune[i++] = (uint8_t)remaining;  // 剩余部分
            tune[i++] = melody->note;
        } else {
            tune[i++] = (uint8_t)melody->beat;
            tune[i++] = melody->note;
        }
        
        melody++;
    }
    
    // 添加结束标记
    tune[i++] = 0;
    tune[i] = 0;
}

// 设置自定义旋律到 eepromBuffer.tune
void set_custom_melody(const MelodyNote *melody) {
    melody_to_tune(melody, eepromBuffer.tune);
    // saveEEpromSettings() 需要在调用处手动调用
}

// 播放指定的旋律（你的旋律：2-3-5-6十六分音符，1四分音符高八度）
void play_my_melody(void) {
    // 设置旋律到 tune 数组
    melody_to_tune(my_melody, eepromBuffer.tune);
    
    // 播放旋律
    playBlueJayTune();
}


void pause(uint16_t ms)
{
    SET_DUTY_CYCLE_ALL(0);
    delayMillis(ms);
    SET_DUTY_CYCLE_ALL(beep_volume); // volume of the beep, (duty cycle) don't go
                                     // above 25 out of 2000
}

void setVolume(uint8_t volume)
{
    if (volume > 11) {
        volume = 11;
    }
    beep_volume = volume * 3; // volume variable from 0 - 11 equates to CCR value of 0-33
}

void setCaptureCompare()
{
    SET_DUTY_CYCLE_ALL(beep_volume); // volume of the beep, (duty cycle) 
}

void playBJNote(uint16_t freq, uint16_t bduration)
{
    uint16_t timerOne_reload;
    SET_PRESCALER_PWM(9);
    timerOne_reload = (uint16_t)(CPU_FREQUENCY_MHZ * 100000 / freq);
    SET_AUTO_RELOAD_PWM(timerOne_reload);
    SET_DUTY_CYCLE_ALL(beep_volume * timerOne_reload/TIM1_AUTORELOAD);
    delayMillis(bduration);
}

uint16_t getBlueJayNoteFrequency(uint8_t bjarrayfreq)
{
    return (uint16_t)(10000000 / ((uint32_t)bjarrayfreq * 247 + 4000));
}

void playBlueJayTune(void)
{
    uint8_t  full_time_count = 0;
    uint32_t duration;          
    uint16_t frequency;
    uint8_t  t4, t3;
    comStep(3);

    for (int i = 4; i < 128; i += 2) {
        RELOAD_WATCHDOG_COUNTER();
        signaltimeout = 0;
        t4 = eepromBuffer.tune[i];
        t3 = eepromBuffer.tune[i + 1];
        if (t4 == 0 && t3 == 0) {
            break;
        }

        if (t4 == 255 && t3 != 0) {
            full_time_count++;

        } else if (t3 == 0) {
            duration = (uint32_t)full_time_count * 255 + t4;
            SET_DUTY_CYCLE_ALL(0);
            delayMillis((uint16_t)duration);
            full_time_count = 0;

        } else {
            uint32_t total_pulses = (uint32_t)full_time_count * 255 + t4;
            uint32_t t3_period    = (uint32_t)t3 * 247 + 4000;
            duration              = (total_pulses * t3_period) / 11000;

            frequency = getBlueJayNoteFrequency(t3);
            playBJNote(frequency, (uint16_t)duration);
            full_time_count = 0;
        }
        
        if(eepromBuffer.tune[3] > 239 ){
            SET_DUTY_CYCLE_ALL(0);
            delayMillis(10*(255 - eepromBuffer.tune[3]));
        }
    }

    allOff();
    SET_PRESCALER_PWM(0);
    SET_AUTO_RELOAD_PWM(TIMER1_MAX_ARR);
    signaltimeout = 0;
    RELOAD_WATCHDOG_COUNTER();
}


void playStartupTune()
{
    __disable_irq();
    comStep(3);
    play_my_melody();
//   if (eepromBuffer.tune[0] != ERASED_FLASH_BYTE) {
//     playBlueJayTune();
//     } else {
//         SET_AUTO_RELOAD_PWM(TIM1_AUTORELOAD);
//         setCaptureCompare();
//         comStep(3); // activate a pwm channel
//         SET_PRESCALER_PWM(55); // frequency of beep
//         delayMillis(200); // duration of beep

//         comStep(5);
//         SET_PRESCALER_PWM(40); // next beep is higher frequency
//         delayMillis(200);

//         comStep(6);
//         SET_PRESCALER_PWM(25); // higher again..
//         delayMillis(200);

//         allOff(); // turn all channels low again
//         SET_PRESCALER_PWM(0); // set prescaler back to 0.
//         signaltimeout = 0;
//     }

    SET_AUTO_RELOAD_PWM(TIMER1_MAX_ARR);
    __enable_irq();
}

void playBrushedStartupTune()
{
    __disable_irq();
    SET_AUTO_RELOAD_PWM(TIM1_AUTORELOAD);
    setCaptureCompare();
    comStep(1); // activate a pwm channel
    SET_PRESCALER_PWM(40); // frequency of beep
    delayMillis(300); // duration of beep
    comStep(2); // activate a pwm channel
    SET_PRESCALER_PWM(30); // frequency of beep
    delayMillis(300); // duration of beep
    comStep(3); // activate a pwm channel
    SET_PRESCALER_PWM(25); // frequency of beep
    delayMillis(300); // duration of beep
    comStep(4);
    SET_PRESCALER_PWM(20); // higher again..
    delayMillis(300);
    allOff(); // turn all channels low again
    SET_PRESCALER_PWM(0); // set prescaler back to 0.
    signaltimeout = 0;
    SET_AUTO_RELOAD_PWM(TIMER1_MAX_ARR);
    __enable_irq();
}

void playDuskingTune()
{
    setCaptureCompare();
    SET_AUTO_RELOAD_PWM(TIM1_AUTORELOAD);
    comStep(2); // activate a pwm channel
    SET_PRESCALER_PWM(60); // frequency of beep
    delayMillis(200); // duration of beep
    SET_PRESCALER_PWM(55); // next beep is higher frequency
    delayMillis(150);
    SET_PRESCALER_PWM(50); // higher again..
    delayMillis(150);
    SET_PRESCALER_PWM(45); // frequency of beep
    delayMillis(100); // duration of beep
    SET_PRESCALER_PWM(50); // next beep is higher frequency
    delayMillis(100);
    SET_PRESCALER_PWM(55); // higher again..
    delayMillis(100);
    SET_PRESCALER_PWM(25); // higher again..
    delayMillis(200);
    SET_PRESCALER_PWM(55); // higher again..
    delayMillis(150);
    allOff(); // turn all channels low again
    SET_PRESCALER_PWM(0); // set prescaler back to 0.
    SET_AUTO_RELOAD_PWM(TIMER1_MAX_ARR);
}

void playInputTune2()
{
    SET_AUTO_RELOAD_PWM(TIM1_AUTORELOAD);
    __disable_irq();
    RELOAD_WATCHDOG_COUNTER();
    SET_PRESCALER_PWM(60);
    setCaptureCompare();
    comStep(1);
    delayMillis(75);
    SET_PRESCALER_PWM(80);
    delayMillis(75);
    SET_PRESCALER_PWM(90);
    RELOAD_WATCHDOG_COUNTER();
    delayMillis(75);
    allOff();
    SET_PRESCALER_PWM(0);
    signaltimeout = 0;
    SET_AUTO_RELOAD_PWM(TIMER1_MAX_ARR);
    __enable_irq();
}

void playInputTune()
{
    __disable_irq();
    SET_AUTO_RELOAD_PWM(TIM1_AUTORELOAD);
    RELOAD_WATCHDOG_COUNTER();
    setCaptureCompare();
    comStep(3);
    SET_PRESCALER_PWM(100);
    delayMillis(100);
    SET_PRESCALER_PWM(0);
    delayMillis(100);
    SET_PRESCALER_PWM(80);
    delayMillis(100);
    allOff();
    SET_PRESCALER_PWM(0);
    signaltimeout = 0;
    SET_AUTO_RELOAD_PWM(TIMER1_MAX_ARR);
    __enable_irq();
}

// 关机音效
void playShotDownTune()
{
    __disable_irq();
    SET_AUTO_RELOAD_PWM(TIM1_AUTORELOAD);
    RELOAD_WATCHDOG_COUNTER();
    setCaptureCompare();
    comStep(3);
    SET_PRESCALER_PWM(80);
    delayMillis(100);
    SET_PRESCALER_PWM(0);
    delayMillis(100);
    SET_PRESCALER_PWM(100);
    delayMillis(100);
    allOff();
    SET_PRESCALER_PWM(0);
    signaltimeout = 0;
    SET_AUTO_RELOAD_PWM(TIMER1_MAX_ARR);
    __enable_irq();
}

void playDefaultTone()
{
    SET_AUTO_RELOAD_PWM(TIM1_AUTORELOAD);
    SET_PRESCALER_PWM(50);
    setCaptureCompare();
    comStep(2);
    delayMillis(150);
    RELOAD_WATCHDOG_COUNTER();
    SET_PRESCALER_PWM(30);
    delayMillis(150);
    allOff();
    SET_PRESCALER_PWM(0);
    signaltimeout = 0;
    SET_AUTO_RELOAD_PWM(TIMER1_MAX_ARR);
}

void playChangedTone()
{
    SET_AUTO_RELOAD_PWM(TIM1_AUTORELOAD);
    SET_PRESCALER_PWM(40);
    setCaptureCompare();
    comStep(2);
    delayMillis(150);
    RELOAD_WATCHDOG_COUNTER();
    SET_PRESCALER_PWM(80);
    delayMillis(150);
    allOff();
    SET_PRESCALER_PWM(0);
    signaltimeout = 0;
    SET_AUTO_RELOAD_PWM(TIMER1_MAX_ARR);
}

void playBeaconTune3()
{
    SET_AUTO_RELOAD_PWM(TIM1_AUTORELOAD);
    __disable_irq();
    setCaptureCompare();
    for (int i = 119; i > 0; i = i - 2) {
        RELOAD_WATCHDOG_COUNTER();
        comStep(i / 20);
        SET_PRESCALER_PWM(10 + (i / 2));
        delayMillis(10);
    }
    allOff();
    SET_PRESCALER_PWM(0);
    signaltimeout = 0;
    SET_AUTO_RELOAD_PWM(TIMER1_MAX_ARR);
    __enable_irq();
}

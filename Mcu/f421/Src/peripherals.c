/*
 * peripherals.c
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

// PERIPHERAL SETUP

#define KR_KEY_Reload ((uint16_t)0xAAAA)
#define KR_KEY_Enable ((uint16_t)0xCCCC)

#include "peripherals.h"

#include "ADC.h"
#include "common.h"
#include "functions.h"
#include "serial_telemetry.h"
#include "targets.h"
#include "at32f421_flash.h"
#ifdef USE_LED_STRIP
#include "WS2812.h"
#endif
#include "lsm6ds3.h"
#include "sensor_processor.h"



void initCorePeripherals(void)
{
    system_clock_config();

    MX_GPIO_Init();
    MX_DMA_Init();
    // 串口
    uart_print_init(115200);
    uart_print_string("uart initialized\n");
    // ABC三相电机控制
    TIM1_Init();
    // 换相间隔时间定时器，用于控制换相的时间间隔
    TIM6_Init();
    // 10kHz定时器，用于控制10kHz的PWM输出
    TIM14_Init();
    // 舵机输出pwm
    TIM15_Init();
    // 比较器初始化
    AT_COMP_Init();
    lsm6ds3_init();
    // 定时读取陀螺仪（原本17用作rgb）17还作为delay定时器
    TIM17_Init();
    // 换相定时器，用于控制下次换相的时间（通过触发中断PeriodElapsedCallback换相）
    TIM16_Init();
    // 输入捕获（TMR3）
    UN_TIM_Init();
    
    // 陀螺仪传感器处理器
    sensor_processor_init();
    
    // 按钮和LED初始化
    button_led_init();
    
    // 启用 Flash 高级别读保护
    // 注意：启用后将无法通过调试器连接，需要解除保护（会擦除Flash）
    // 建议通过条件编译或特定条件控制是否启用
#ifdef ENABLE_FLASH_PROTECTION
    flash_status_type status = flash_fap_high_level_protection_enable();
    if (status == FLASH_OPERATE_DONE) {
        uart_print_string("Flash protection enabled\n");
    } else {
        uart_print_string("Flash protection failed\n");
    }
#endif

}

void initAfterJump(void) { __enable_irq(); }

void system_clock_config(void)
{
    flash_psr_set(FLASH_WAIT_CYCLE_3);
    crm_reset();
    crm_clock_source_enable(CRM_CLOCK_SOURCE_HICK, TRUE);
    while (crm_flag_get(CRM_HICK_STABLE_FLAG) != SET) {
    }
    crm_pll_config(CRM_PLL_SOURCE_HICK, CRM_PLL_MULT_30);
    crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, TRUE);
    while (crm_flag_get(CRM_PLL_STABLE_FLAG) != SET) {
    }
    crm_ahb_div_set(CRM_AHB_DIV_1);
    crm_apb2_div_set(CRM_APB2_DIV_1);
    crm_apb1_div_set(CRM_APB1_DIV_1);
    crm_auto_step_mode_enable(TRUE);
    crm_sysclk_switch(CRM_SCLK_PLL);
    while (crm_sysclk_switch_status_get() != CRM_SCLK_PLL) {
    }
    crm_auto_step_mode_enable(FALSE);
    system_core_clock_update();
}

void AT_COMP_Init(void)
{
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_CMP_PERIPH_CLOCK, TRUE);
    gpio_mode_QUICK(GPIOA, GPIO_MODE_ANALOG, GPIO_PULL_NONE, GPIO_PINS_0);  // PA0 - 反电动势检测
    gpio_mode_QUICK(GPIOA, GPIO_MODE_ANALOG, GPIO_PULL_NONE, GPIO_PINS_1);  // PA1 - 比较器输入
    gpio_mode_QUICK(GPIOA, GPIO_MODE_ANALOG, GPIO_PULL_NONE, GPIO_PINS_4);  // PA4 - 反电动势检测
    gpio_mode_QUICK(GPIOA, GPIO_MODE_ANALOG, GPIO_PULL_NONE, GPIO_PINS_5);  // PA5 - 反电动势检测
    
    // 配置 EXTI_LINE_21（连接到比较器输出）
    EXINT->inten |= EXINT_LINE_21;  // 使能 EXTI_LINE_21 中断
    EXINT->polcfg1 |= EXINT_LINE_21;  // 使能上升沿触发
    EXINT->polcfg2 |= EXINT_LINE_21;  // 使能下降沿触发（双边沿）
    
    NVIC_SetPriority(ADC1_CMP_IRQn, 0);  // TODO 比较器应该高优先，测试不同优先级下的效果
    NVIC_EnableIRQ(ADC1_CMP_IRQn);
    
    CMP->ctrlsts = 0x400000E1;  // PA0 反电动势检测，PA1 比较器输入
    
    cmp_enable(CMP1_SELECTION, TRUE);
}

void MX_IWDG_Init(void)
{
    WDT->cmd = WDT_CMD_UNLOCK;
    WDT->cmd = WDT_CMD_ENABLE;
    WDT->div = WDT_CLK_DIV_32;
    WDT->rld = 4000;
    WDT->cmd = WDT_CMD_RELOAD;
}

void TIM1_Init(void)
{
    crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);
    TMR1->pr = TIM1_AUTORELOAD;
    TMR1->div = 0;

    TMR1->cm1 = 0x6868; // Channel 1 and 2 in PWM output mode
    TMR1->cm2 = 0x68; // channel 3 in PWM output mode
#ifdef USE_INVERTED_HIGH
    tmr_output_channel_polarity_set(TMR1, TMR_SELECT_CHANNEL_1,
        TMR_POLARITY_ACTIVE_LOW);
    tmr_output_channel_polarity_set(TMR1, TMR_SELECT_CHANNEL_2,
        TMR_POLARITY_ACTIVE_LOW);
    tmr_output_channel_polarity_set(TMR1, TMR_SELECT_CHANNEL_3,
        TMR_POLARITY_ACTIVE_LOW);
#endif

    tmr_output_channel_buffer_enable(TMR1, TMR_SELECT_CHANNEL_1, TRUE);
    tmr_output_channel_buffer_enable(TMR1, TMR_SELECT_CHANNEL_2, TRUE);
    tmr_output_channel_buffer_enable(TMR1, TMR_SELECT_CHANNEL_3, TRUE);

    tmr_period_buffer_enable(TMR1, TRUE);
    TMR1->brk_bit.dtc = DEAD_TIME;
    // TMR1->brk_bit.dtc = 0x7f;//   TODO 测试互补 pwm多一点死区
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);

    /*configure PA8/PA9/PA10(TIMER0/CH0/CH1/CH2) as alternate function*/
    gpio_mode_QUICK(PHASE_A_GPIO_PORT_LOW, GPIO_MODE_MUX, GPIO_PULL_NONE,
        PHASE_A_GPIO_LOW);

    gpio_mode_QUICK(PHASE_B_GPIO_PORT_LOW, GPIO_MODE_MUX, GPIO_PULL_NONE,
        PHASE_B_GPIO_LOW);

    gpio_mode_QUICK(PHASE_C_GPIO_PORT_LOW, GPIO_MODE_MUX, GPIO_PULL_NONE,
        PHASE_C_GPIO_LOW);

    gpio_pin_mux_config(PHASE_A_GPIO_PORT_LOW, PHASE_A_PIN_SOURCE_LOW,
        GPIO_MUX_2);
    gpio_pin_mux_config(PHASE_B_GPIO_PORT_LOW, PHASE_B_PIN_SOURCE_LOW,
        GPIO_MUX_2);
    gpio_pin_mux_config(PHASE_C_GPIO_PORT_LOW, PHASE_C_PIN_SOURCE_LOW,
        GPIO_MUX_2);

    /*configure PB13/PB14/PB15(TIMER0/CH0N/CH1N/CH2N) as alternate function*/
    gpio_mode_QUICK(PHASE_A_GPIO_PORT_HIGH, GPIO_MODE_MUX, GPIO_PULL_NONE,
        PHASE_A_GPIO_HIGH);
    //  gpio_output_options_set(GPIOB, GPIO_OTYPE_PP,
    //  GPIO_OSPEED_50MHZ,PHASE_A_GPIO_HIGH);

    gpio_mode_QUICK(PHASE_B_GPIO_PORT_HIGH, GPIO_MODE_MUX, GPIO_PULL_NONE,
        PHASE_B_GPIO_HIGH);
    //   gpio_output_options_set(GPIOB, GPIO_OTYPE_PP,
    //   GPIO_OSPEED_50MHZ,PHASE_B_GPIO_HIGH);

    gpio_mode_QUICK(PHASE_C_GPIO_PORT_HIGH, GPIO_MODE_MUX, GPIO_PULL_NONE,
        PHASE_C_GPIO_HIGH);
    //    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP,
    //    GPIO_OSPEED_50MHZ,PHASE_C_GPIO_HIGH);

    gpio_pin_mux_config(PHASE_A_GPIO_PORT_HIGH, PHASE_A_PIN_SOURCE_HIGH,
        GPIO_MUX_2);
    gpio_pin_mux_config(PHASE_B_GPIO_PORT_HIGH, PHASE_B_PIN_SOURCE_HIGH,
        GPIO_MUX_2);
    gpio_pin_mux_config(PHASE_C_GPIO_PORT_HIGH, PHASE_C_PIN_SOURCE_HIGH,
        GPIO_MUX_2);
}

void TIM6_Init(void)
{
    crm_periph_clock_enable(CRM_TMR6_PERIPH_CLOCK, TRUE);
    TMR6->pr = 0xFFFF;
    TMR6->div = 59;
}

void TIM14_Init(void)
{
    crm_periph_clock_enable(CRM_TMR14_PERIPH_CLOCK, TRUE);
    TMR14->pr = 1000000 / LOOP_FREQUENCY_HZ;
    TMR14->div = 119;

    NVIC_SetPriority(TMR14_GLOBAL_IRQn, 0);
    NVIC_EnableIRQ(TMR14_GLOBAL_IRQn);

    // TMR_Cmd(TMR14, ENABLE);
}

void TIM15_Init(void) 
{
    gpio_init_type gpio_init_struct;
    tmr_output_config_type tmr_output_struct;
    tmr_brkdt_config_type tmr_brkdt_struct;
    uint32_t ccr_value;

    crm_periph_clock_enable(CRM_TMR15_PERIPH_CLOCK, TRUE);
    gpio_default_para_init(&gpio_init_struct);

    /* configure the tmr15 CH1 pin (PA2) */
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE2, GPIO_MUX_0);
    gpio_init_struct.gpio_pins = GPIO_PINS_2;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
    gpio_init(GPIOA, &gpio_init_struct);

    /* configure counter settings */
    tmr_cnt_dir_set(TMR15, TMR_COUNT_UP);
    tmr_clock_source_div_set(TMR15, TMR_CLOCK_DIV1);
    tmr_repetition_counter_set(TMR15, 0);
    tmr_period_buffer_enable(TMR15, FALSE);
    
    tmr_base_init(TMR15, TIM15_PERIOD, TIM15_PRESCALER);

    /* configure primary mode settings */
    tmr_sub_sync_mode_set(TMR15, FALSE);
    tmr_primary_mode_select(TMR15, TMR_PRIMARY_SEL_RESET);

    /* configure channel 1 output settings for PWM mode */
    tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
    tmr_output_struct.oc_output_state = TRUE;
    tmr_output_struct.occ_output_state = FALSE;
    tmr_output_struct.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    tmr_output_struct.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    tmr_output_struct.oc_idle_state = FALSE;
    tmr_output_struct.occ_idle_state = FALSE;
    tmr_output_channel_config(TMR15, TMR_SELECT_CHANNEL_1, &tmr_output_struct);
    
    /* set duty cycle */
    ccr_value = TIM15_MAX_CCR * 20 / 100;
    tmr_channel_value_set(TMR15, TMR_SELECT_CHANNEL_1, ccr_value);
    
    tmr_output_channel_buffer_enable(TMR15, TMR_SELECT_CHANNEL_1, FALSE);

    /* configure break and dead-time settings */
    tmr_brkdt_struct.brk_enable = FALSE;
    tmr_brkdt_struct.auto_output_enable = FALSE;
    tmr_brkdt_struct.brk_polarity = TMR_BRK_INPUT_ACTIVE_LOW;
    tmr_brkdt_struct.fcsoen_state = FALSE;
    tmr_brkdt_struct.fcsodis_state = FALSE;
    tmr_brkdt_struct.wp_level = TMR_WP_OFF;
    tmr_brkdt_struct.deadtime = 0;
    tmr_brkdt_config(TMR15, &tmr_brkdt_struct);
    tmr_output_enable(TMR15, TRUE);
    tmr_counter_enable(TMR15, TRUE);
}

void TIM16_Init(void)
{
    crm_periph_clock_enable(CRM_TMR16_PERIPH_CLOCK, TRUE);
    TMR16->pr = 500;
    TMR16->div = 59;
    NVIC_SetPriority(TMR16_GLOBAL_IRQn, 0);
    NVIC_EnableIRQ(TMR16_GLOBAL_IRQn);
}

void TIM17_Init(void)
{
    crm_periph_clock_enable(CRM_TMR17_PERIPH_CLOCK, TRUE);
    TMR17->pr = 0xFFFF;
    TMR17->div = 119;
    TMR17->ctrl1_bit.prben = TRUE;

    // TMR_Cmd(TMR15, ENABLE);
}
/**
 * @brief 初始化DMA通道1
 * @note DMA通道1用于将数据从内存传输到USART1
 */
void MX_DMA_Init(void)
{
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);// 使能DMA1时钟，确保DMA通道1正常工作
    NVIC_SetPriority(DMA1_Channel5_4_IRQn, 1);
    NVIC_EnableIRQ(DMA1_Channel5_4_IRQn);// 使能DMA通道1的中断，用于接收DMA传输完成的中断信号
}

void MX_GPIO_Init(void) { }

// 初始化定时器1和定时器3

void UN_TIM_Init(void)
{
    // PB4 油门输入
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_TMR3_PERIPH_CLOCK, TRUE);
    gpio_mode_QUICK(INPUT_PIN_PORT, GPIO_MODE_MUX, GPIO_PULL_NONE, INPUT_PIN);
    gpio_pin_mux_config(INPUT_PIN_PORT, INPUT_PIN_SOURCE, GPIO_MUX_1);

    // DMA1时钟使能
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);

    // PB4 DMA配置
    INPUT_DMA_CHANNEL->paddr = (uint32_t)&IC_TIMER_REGISTER->c1dt;
    INPUT_DMA_CHANNEL->maddr = (uint32_t)&dma_buffer;
    INPUT_DMA_CHANNEL->dtcnt = buffersize;
    INPUT_DMA_CHANNEL->ctrl = 0X98a;// TODO 这里应该开始就要使能
    NVIC_SetPriority(IC_DMA_IRQ_NAME, 1);
    NVIC_EnableIRQ(IC_DMA_IRQ_NAME);

    IC_TIMER_REGISTER->pr = 0xFFFF;
    IC_TIMER_REGISTER->ctrl1_bit.prben = TRUE;

    // CH1输入捕获配置（PB4油门）
    IC_TIMER_REGISTER->cm1_input_bit.c1c = 0x1;         // CH1直接映射
    IC_TIMER_REGISTER->cm1_input_bit.c1idiv = 0x0;      // CH1不分频
    IC_TIMER_REGISTER->cm1_input_bit.c1df = 0x0A;       // CH1输入滤波：8个采样周期
    IC_TIMER_REGISTER->cctrl = 0xB;                      // CH1输入捕获使能+双边沿触发

    IC_TIMER_REGISTER->ctrl1_bit.tmren = TRUE;
    IC_TIMER_REGISTER->iden |= TMR_C1_DMA_REQUEST;// 使能定时器 1 通道 1 的 DMA 请求
    IC_TIMER_REGISTER->swevt_bit.ovfswtr = TRUE;


    // PB5转向输入配置 (TIM3_CH2 + 中断方式)
    gpio_init_type gpio_init_struct;
    tmr_input_config_type tmr_input_struct;

    gpio_default_para_init(&gpio_init_struct);

    /* configure the tmr3 CH2 pin (PB5) */
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE5, GPIO_MUX_1);
    gpio_init_struct.gpio_pins = GPIO_PINS_5;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
    gpio_init(GPIOB, &gpio_init_struct);

    /* configure channel 2 input settings for capture mode */
    tmr_input_struct.input_channel_select = TMR_SELECT_CHANNEL_2;
    tmr_input_struct.input_mapped_select = TMR_CC_CHANNEL_MAPPED_DIRECT;
    tmr_input_struct.input_polarity_select = TMR_INPUT_RISING_EDGE;
    tmr_input_struct.input_filter_value = 0x05;
    tmr_input_channel_init(TMR3, &tmr_input_struct, TMR_CHANNEL_INPUT_DIV_1);

    /* enable capture interrupt */
    tmr_interrupt_enable(TMR3, TMR_C2_INT, TRUE);
    NVIC_SetPriority(TMR3_GLOBAL_IRQn, 1);
    NVIC_EnableIRQ(TMR3_GLOBAL_IRQn);
    tmr_counter_enable(TMR3, TRUE);
}

void PB5_TMR3_CH2_Init(void)
{
    gpio_init_type gpio_init_struct;
    tmr_input_config_type tmr_input_struct;

    gpio_default_para_init(&gpio_init_struct);

    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE5, GPIO_MUX_1);
    gpio_init_struct.gpio_pins = GPIO_PINS_5;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pull = GPIO_PULL_UP;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
    gpio_init(GPIOB, &gpio_init_struct);

    tmr_input_struct.input_channel_select = TMR_SELECT_CHANNEL_2;
    tmr_input_struct.input_mapped_select = TMR_CC_CHANNEL_MAPPED_DIRECT;
    tmr_input_struct.input_polarity_select = TMR_INPUT_RISING_EDGE;
    tmr_input_struct.input_filter_value = 0x0A;
    tmr_input_channel_init(TMR3, &tmr_input_struct, TMR_CHANNEL_INPUT_DIV_1);

    tmr_interrupt_enable(TMR3, TMR_C2_INT, TRUE);
    NVIC_SetPriority(TMR3_GLOBAL_IRQn, 2);
    NVIC_EnableIRQ(TMR3_GLOBAL_IRQn);
    tmr_counter_enable(TMR3, TRUE);
}

void PB5_TMR3_CH2_Resume(void)
{
    tmr_input_config_type tmr_input_struct;

    tmr_input_struct.input_channel_select = TMR_SELECT_CHANNEL_2;
    tmr_input_struct.input_mapped_select = TMR_CC_CHANNEL_MAPPED_DIRECT;
    tmr_input_struct.input_polarity_select = TMR_INPUT_RISING_EDGE;
    tmr_input_struct.input_filter_value = 0x0A;
    tmr_input_channel_init(TMR3, &tmr_input_struct, TMR_CHANNEL_INPUT_DIV_1);

    tmr_interrupt_enable(TMR3, TMR_C2_INT, TRUE);
    tmr_counter_enable(TMR3, TRUE);
}

void reloadWatchDogCounter()
{
    WDT->cmd = WDT_CMD_RELOAD;
}

void setPWMCompare1(uint16_t compareone) { TMR1->c1dt = compareone; }
void setPWMCompare2(uint16_t comparetwo) { TMR1->c2dt = comparetwo; }
void setPWMCompare3(uint16_t comparethree) { TMR1->c3dt = comparethree; }

void generatePwmTimerEvent()
{
    TMR1->swevt |= TMR_OVERFLOW_SWTRIG;
    ;
}

void resetInputCaptureTimer()
{
    IC_TIMER_REGISTER->pr = 0;
    IC_TIMER_REGISTER->cval = 0;
}

#ifdef USE_RGB_LED // has 3 color led
void LED_GPIO_init()
{
    /* GPIO Ports Clock Enable */
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);

    gpio_mode_QUICK(RED_PORT, GPIO_MODE_OUTPUT, GPIO_PULL_NONE, RED_PIN);

    gpio_mode_QUICK(GREEN_PORT, GPIO_MODE_OUTPUT, GPIO_PULL_NONE, GREEN_PIN);

    gpio_mode_QUICK(BLUE_PORT, GPIO_MODE_OUTPUT, GPIO_PULL_NONE, BLUE_PIN);    
}

void setIndividualRGBLed(uint8_t red, uint8_t green, uint8_t blue){

  if(red > 0){   
    RED_PORT->clr = RED_PIN;
  }else{
    RED_PORT->scr = RED_PIN;
  }
  if(green > 0){
    GREEN_PORT->clr = GREEN_PIN;
  }else{
    GREEN_PORT->scr = GREEN_PIN;
  }
  if(blue > 0){
    BLUE_PORT->clr = BLUE_PIN;
  }else{
    BLUE_PORT->scr = BLUE_PIN;;
  }
}

#endif


void enableCorePeripherals()
{
    tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_1, TRUE);
    tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_2, TRUE);
    tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_3, TRUE);
    tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_1C, TRUE);
    tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_2C, TRUE);
    tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_3C, TRUE);

    // 油门输入dma使能
    INPUT_DMA_CHANNEL->ctrl = 0X98B;

    TMR1->ctrl1_bit.tmren = TRUE;
    TMR1->brk_bit.oen = TRUE;
    TMR1->swevt |= TMR_OVERFLOW_SWTRIG;

#ifndef BRUSHED_MODE
    COM_TIMER->ctrl1_bit.tmren = TRUE;
    COM_TIMER->swevt |= TMR_OVERFLOW_SWTRIG;
    COM_TIMER->iden &= ~TMR_OVF_INT;
#endif
    UTILITY_TIMER->ctrl1_bit.tmren = TRUE;
    INTERVAL_TIMER->ctrl1_bit.tmren = TRUE;
    INTERVAL_TIMER->swevt |= TMR_OVERFLOW_SWTRIG;
    TEN_KHZ_TIMER->ctrl1_bit.tmren = TRUE;
    TEN_KHZ_TIMER->swevt |= TMR_OVERFLOW_SWTRIG;
    TEN_KHZ_TIMER->iden |= TMR_OVF_INT;

#ifdef USE_ADC
    ADC_Init();
#endif

#ifdef USE_ADC_INPUT

#else
    tmr_channel_enable(IC_TIMER_REGISTER, IC_TIMER_CHANNEL, TRUE);
    IC_TIMER_REGISTER->ctrl1_bit.tmren = TRUE;
#endif

    // 油门信号中断
    NVIC_SetPriority(EXINT15_4_IRQn, 2);
    NVIC_EnableIRQ(EXINT15_4_IRQn);
    EXINT->inten |= EXINT_LINE_15;
		
#ifdef USE_PULSE_OUT
 gpio_mode_QUICK(GPIOB, GPIO_MODE_OUTPUT, GPIO_PULL_NONE, GPIO_PINS_8);
#endif
}

/**
 * @brief 按钮和LED初始化
 * @note PB3 - 按钮输入（上拉）
 *       PB2 - LED输出
 *       PA15 - mos开关电路输出
 */
void button_led_init(void)
{
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    
    // PB3 按钮输入，上拉
    gpio_mode_QUICK(GPIOB, GPIO_MODE_INPUT, GPIO_PULL_UP, GPIO_PINS_3);
    
    // PB2 LED输出
    gpio_mode_QUICK(GPIOB, GPIO_MODE_OUTPUT, GPIO_PULL_NONE, GPIO_PINS_2);

    // PA15 mos开关电路输出
    gpio_mode_QUICK(GPIOA, GPIO_MODE_OUTPUT, GPIO_PULL_NONE, GPIO_PINS_15);
    
    // 默认LED关闭
    led_set(0);
}

/**
 * @brief 读取按键状态（带消抖）
 * @note  硬件接法：PB3内部上拉（GPIO_PULL_UP），按键一端接PB3，另一端接GND
 *        按键按下 → PB3被拉到GND（低电平/RESET）→ 本函数返回1
 *        按键释放 → PB3被上拉到VDD（高电平/SET）→ 本函数返回0
 * @return 1 - 按键按下
 *         0 - 按键释放
 */
uint8_t button_read(void)
{
    // 内部上拉 + 按下接GND → RESET对应"按下"状态
    uint8_t first = (gpio_input_data_bit_read(GPIOB, GPIO_PINS_3) == RESET) ? 1 : 0;
    
    // 延时约10ms消抖
    delayMillis(10);
    
    uint8_t second = (gpio_input_data_bit_read(GPIOB, GPIO_PINS_3) == RESET) ? 1 : 0;
    
    // 两次读取一致则认为状态稳定，不一致则视为未按（释放）
    return (first == second) ? first : 0;
}


/**
 * @brief 设置LED状态
 * @param state: 1 - LED亮
 *               0 - LED灭
 */
void led_set(uint8_t state)
{
    if (state) {
        GPIOB->clr = GPIO_PINS_2;  // 低电平点亮
    } else {
        GPIOB->scr = GPIO_PINS_2;  // 高电平熄灭
    }
}

void mos_button_set(uint8_t state)
{
    if (state) {
        GPIOA->scr = GPIO_PINS_15;
    } else {
        GPIOA->clr = GPIO_PINS_15;
    }
}

/**
 * @brief LED 每隔 1 秒闪一下（非阻塞式）
 * 
 * 调用后检查时间，每隔 500ms 切换一次 LED 状态
 * 需要在主循环中持续调用，不会阻塞其他代码执行
 * 闪烁模式：亮 500ms，灭 500ms，周期 1 秒
 */
void led_blink_1hz(void)
{
    static uint32_t last_time = 0;  // 上次切换时间
    static uint8_t led_state = 0;   // 当前 LED 状态
    static uint16_t counter = 0;    // 计数器，计满 100 次为 500ms
    
    uint32_t current_time = UTILITY_TIMER->cval;  // 获取当前时间（微秒）
    
    // 检查是否超过 5ms（5000 微秒）
    if (current_time - last_time >= 5000 || 
        (current_time < last_time && 0xFFFF - last_time + current_time >= 5000)) {
        counter++;
        last_time = current_time;
        
        // 计满 100 次（500ms）切换 LED 状态
        if (counter >= 100) {
            led_set(led_state);
            led_state = !led_state;
            counter = 0;
        }
    }
}

/**
 * @brief LED快闪N次（阻塞式）
 * @param times 闪烁次数
 * @param interval_ms 每次闪烁间隔（毫秒）
 */
void led_blink_fast(uint8_t times, uint16_t interval_ms)
{
    for (uint8_t i = 0; i < times; i++) {
        led_set(1);   // LED亮
        delayMillis(interval_ms);
        
        led_set(0);   // LED灭
        delayMillis(interval_ms);
    }
}

/**
 * @brief LED快闪3次（默认）
 * 
 * 快捷函数，LED快速闪烁3次（亮100ms，灭100ms）
 */
void led_blink_fast_3x(void)
{
    led_blink_fast(3, 200);
}

/**
 * @brief 启用 Flash 高级别读保护（FAP High Level）
 * @note 启用后将禁止调试器访问和读取 Flash 内容
 *       解除保护会擦除整个 Flash
 * @retval FLASH_OPERATE_DONE - 成功
 *         FLASH_OPERATE_TIMEOUT - 超时
 *         FLASH_PROGRAM_ERROR - 编程错误
 *         FLASH_EPP_ERROR - 保护错误
 */
flash_status_type flash_fap_high_level_protection_enable(void)
{
    flash_status_type status;
    
    // 解锁 FLASH
    flash_unlock();
    
    // 启用 FAP 高级别保护
    status = flash_fap_high_level_enable(TRUE);
    
    // 锁定 FLASH
    flash_lock();
    
    return status;
}

/**
 * @brief 禁用 Flash 高级别读保护（FAP High Level）
 * @note 禁用保护将擦除整个 Flash 内容
 * @retval FLASH_OPERATE_DONE - 成功
 *         FLASH_OPERATE_TIMEOUT - 超时
 *         FLASH_PROGRAM_ERROR - 编程错误
 *         FLASH_EPP_ERROR - 保护错误
 */
flash_status_type flash_fap_high_level_protection_disable(void)
{
    flash_status_type status;
    
    // 解锁 FLASH
    flash_unlock();
    
    // 禁用 FAP 高级别保护
    status = flash_fap_high_level_enable(FALSE);
    
    // 锁定 FLASH
    flash_lock();
    
    return status;
}

/* Includes
 * ------------------------------------------------------------------*/
#include "at32f421_it.h"

#include "ADC.h"
#include "main.h"
#include "targets.h"
#include "common.h"
#include "comparator.h"
#include "sensor_processor.h"


extern void transfercomplete();
extern void PeriodElapsedCallback();
extern void interruptRoutine();
extern void doPWMChanges();
extern void tenKhzRoutine();
extern void sendDshotDma();
extern void receiveDshotDma();
extern void signalEdgeRoutine();
extern void processDshot();

extern char send_telemetry;
extern char telemetry_done;
extern char servoPwm;
extern char dshot;
int exti_int = 0;

void HardFault_Handler(void)
{
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1) {
    }
}

/**
 * @brief  This function handles Memory Manage exception.
 * @param  None
 * @retval None
 */
void MemManage_Handler(void)
{
    /* Go to infinite loop when Memory Manage exception occurs */
    while (1) {
    }
}

/**
 * @brief  This function handles Bus Fault exception.
 * @param  None
 * @retval None
 */
void BusFault_Handler(void)
{
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1) {
    }
}

/**
 * @brief  This function handles Usage Fault exception.
 * @param  None
 * @retval None
 */
void UsageFault_Handler(void)
{
    /* Go to infinite loop when Usage Fault exception occurs */
    while (1) {
    }
}

/**
 * @brief  This function handles SVCall exception.
 * @param  None
 * @retval None
 */
void SVC_Handler(void) { }

/**
 * @brief  This function handles Debug Monitor exception.
 * @param  None
 * @retval None
 */
void DebugMon_Handler(void) { }

/**
 * @brief  This function handles PendSV_Handler exception.
 * @param  None
 * @retval None
 */
void PendSV_Handler(void) { }

/**
 * @brief  This function handles SysTick Handler.
 * @param  None
 * @retval None
 */
void SysTick_Handler(void) { }

void DMA1_Channel1_IRQHandler(void)
{
    if (dma_flag_get(DMA1_FDT1_FLAG) == SET) {
        DMA1->clr = DMA1_GL1_FLAG;
#ifdef USE_ADC
        ADC_DMA_Callback();
#endif
        if (dma_flag_get(DMA1_DTERR1_FLAG) == SET) {
            DMA1->clr = DMA1_GL1_FLAG;
        }
    }
}


void DMA1_Channel5_4_IRQHandler(void)
{
#ifdef USE_TIMER_3_CHANNEL_1
    if (dshot) {
        DMA1->clr = DMA1_GL4_FLAG;
        INPUT_DMA_CHANNEL->ctrl_bit.chen = FALSE;
        transfercomplete();
        EXINT->swtrg = EXINT_LINE_15;
        return;
    }
    if (dma_flag_get(DMA1_HDT4_FLAG) == SET) {
        if (servoPwm) {
            IC_TIMER_REGISTER->cctrl_bit.c1p = TMR_INPUT_FALLING_EDGE;
            DMA1->clr = DMA1_HDT4_FLAG;
        }
    }
    if (dma_flag_get(DMA1_FDT4_FLAG) == SET) {
        DMA1->clr = DMA1_GL4_FLAG;
        INPUT_DMA_CHANNEL->ctrl_bit.chen = FALSE;
        transfercomplete();
        EXINT->swtrg = EXINT_LINE_15;
    }
    if (dma_flag_get(DMA1_DTERR4_FLAG) == SET) {
        DMA1->clr = DMA1_GL4_FLAG;
    }
#endif
}

/**
 * @brief This function handles ADC and COMP interrupts (COMP interrupts
 * through EXTI lines 21 and 22).
 */
void ADC1_CMP_IRQHandler(void)
{
  // INTERVAL_TIMER->cval 间隔定时器的当前计数值
  // average_interval>>1 平均间隔的一半（相当于除以2）
  // 如果当前时间已经超过换相间隔的一半，确保过零点检测发生在换相周期的后半段（避免误检测）
  if((INTERVAL_TIMER->cval) > ((average_interval>>1))){
        // 清除外部中断标志
       EXINT->intsts = EXTI_LINE;
       interruptRoutine();
    }else{ 
      // 如果还未到换相时间，只有检测到预期的沿方向才清除中断标志
      if (getCompOutputLevel() == rising){
        EXINT->intsts = EXTI_LINE;
    }
  }
}

/**
 * #define INTERVAL_TIMER TMR6
#define TEN_KHZ_TIMER TMR14
#define UTILITY_TIMER TMR17
#define COM_TIMER TMR16
 */

/**
 * @brief This function handles TIM6 global and DAC underrun error interrupts.
 */
void TMR14_GLOBAL_IRQHandler(void)
{
    TMR14->ists = (uint16_t)~TMR_OVF_FLAG;
    tenKhzRoutine();
}

/**
 * @brief This function handles TIM14 global interrupt.
 */
void TMR16_GLOBAL_IRQHandler(void)
{ 
    TMR16->ists = 0x00;
    PeriodElapsedCallback();
}

void TMR15_GLOBAL_IRQHandler(void)
{
    TMR15->ists = (uint16_t)~TMR_OVF_FLAG;
    TMR15->ists = (uint16_t)~TMR_C1_FLAG;
}

extern uint32_t pwm2_capture_high_time;
extern volatile uint8_t pwm2_data_ready;

void TMR3_GLOBAL_IRQHandler(void)
{
    static uint32_t capture_value_prev = 0;
    static uint8_t capture_state = 0;
    tmr_input_config_type tmr_input_struct;

    if ((TMR3->ists & TMR_OVF_FLAG) != (uint16_t)RESET) {
        TMR3->ists = (uint16_t)~TMR_OVF_FLAG;
    }

    if ((TMR3->ists & TMR_C2_FLAG) != (uint16_t)RESET) {
        uint32_t capture_value = tmr_channel_value_get(TMR3, TMR_SELECT_CHANNEL_2);

        if (capture_state == 0) {
            capture_value_prev = capture_value;
            capture_state = 1;

            tmr_input_struct.input_channel_select = TMR_SELECT_CHANNEL_2;
            tmr_input_struct.input_mapped_select = TMR_CC_CHANNEL_MAPPED_DIRECT;
            tmr_input_struct.input_polarity_select = TMR_INPUT_FALLING_EDGE;
            tmr_input_struct.input_filter_value = 0x0A;
            tmr_input_channel_init(TMR3, &tmr_input_struct, TMR_CHANNEL_INPUT_DIV_1);
        } else {
            uint32_t high_time;
            if (capture_value >= capture_value_prev) {
                high_time = capture_value - capture_value_prev;
            } else {
                high_time = (65536 - capture_value_prev) + capture_value;
            }

            if (high_time > 500 && high_time < 3000) {
                pwm2_capture_high_time = high_time;
                pwm2_data_ready = 1;
            }

            capture_state = 0;

            tmr_input_struct.input_channel_select = TMR_SELECT_CHANNEL_2;
            tmr_input_struct.input_mapped_select = TMR_CC_CHANNEL_MAPPED_DIRECT;
            tmr_input_struct.input_polarity_select = TMR_INPUT_RISING_EDGE;
            tmr_input_struct.input_filter_value = 0x0A;
            tmr_input_channel_init(TMR3, &tmr_input_struct, TMR_CHANNEL_INPUT_DIV_1);
        }

        TMR3->ists = (uint16_t)~TMR_C2_FLAG;
    }
}


void TMR17_GLOBAL_IRQHandler(void) {
    if (TMR17->ists & TMR_OVF_FLAG) {
        TMR17->ists = ~TMR_OVF_FLAG;
        sensor_processor_update_gyro();
    }
}

// void DMA_Channel0_IRQHandler(void)         // ADC
//{
//	  if(LL_DMA_IsActiveFlag_TC1(DMA1) == 1)
//	  {
//	    /* Clear flag DMA global interrupt */
//	    /* (global interrupt flag: half transfer and transfer complete
// flags) */ 	    LL_DMA_ClearFlag_GI1(DMA1); 	    ADC_DMA_Callback();
//	    /* Call interruption treatment function */
//	 //   AdcDmaTransferComplete_Callback();
//	  }

//	  /* Check whether DMA transfer error caused the DMA interruption */
//	  if(LL_DMA_IsActiveFlag_TE1(DMA1) == 1)
//	  {
//	    /* Clear flag DMA transfer error */
//	    LL_DMA_ClearFlag_TE1(DMA1);

//	    /* Call interruption treatment function */
//	  }
//}

void EXINT15_4_IRQHandler(void)
{
    exti_int++;
    if ((EXINT->intsts & EXINT_LINE_15) != (uint32_t)RESET) {
        EXINT->intsts |= EXINT_LINE_15;
        processDshot();
    }
}

/******************************************************************************/
/*                 AT32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the */
/*  available peripheral interrupt handler's name please refer to the startup
 */
/*  file (startup_at32f413_xx.s).                                            */
/******************************************************************************/

/**
 * @brief  This function handles PPP interrupt request.
 * @param  None
 * @retval None
 */
/*void PPP_IRQHandler(void)
{
}*/

/**
 * @}
 */

/**
 * @}
 */

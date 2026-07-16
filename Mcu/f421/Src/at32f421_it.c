/* Includes
 * ------------------------------------------------------------------*/
#include "at32f421_it.h"

#include "ADC.h"
#include "main.h"
#include "targets.h"
#include "common.h"
#include "comparator.h"
#include "sensor_processor.h"
#include "i2c_application.h"

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
extern uint16_t gyro_buf;
extern uint8_t buf_arry[2];
int exti_int = 0;
extern i2c_handle_type hi2cx;

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
    if (dshot) {
        DMA1->clr = DMA1_GL4_FLAG;
        INPUT_DMA_CHANNEL->ctrl_bit.chen = FALSE;
        transfercomplete();
        EXINT->swtrg = EXINT_LINE_15;
        return;
    }
    // 通道4半传输完成
    if (dma_flag_get(DMA1_HDT4_FLAG) == SET) {
        if (servoPwm) {
            IC_TIMER_REGISTER->cctrl_bit.c1p = TMR_INPUT_FALLING_EDGE;
            DMA1->clr = DMA1_HDT4_FLAG;// 清除通道 4 的半数据传输标志
        }
    }
    // 通道4完整传输完成
    if (dma_flag_get(DMA1_FDT4_FLAG) == SET) {
        DMA1->clr = DMA1_GL4_FLAG;// 清除通道 4 的完整数据传输标志
        INPUT_DMA_CHANNEL->ctrl_bit.chen = FALSE;// 禁用输入通道 4 的DMA传输
        // IC_TIMER_REGISTER->cctrl_bit.c1p = TMR_INPUT_RISING_EDGE;
        transfercomplete();
        EXINT->swtrg = EXINT_LINE_15;// 触发外部中断 15
    }

    if (dma_flag_get(DMA1_DTERR4_FLAG) == SET) {
        DMA1->clr = DMA1_GL4_FLAG;
    }

    // 处理I2C接收中断，陀螺仪用
    i2c_dma_rx_irq_handler(&hi2cx);
}

void I2C2_EVT_IRQHandler(void)
{
  i2c_evt_irq_handler(&hi2cx);
}

void I2C2_ERR_IRQHandler(void)
{
  i2c_err_irq_handler(&hi2cx);
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
    static uint8_t  capture_state = 0;     // 0=等上升沿(起点), 1=等下降沿(算高电平)

    /* ---- 溢出清标志（16位无符号减法算差值已自动处理溢出，这里只是防止反复进中断） ---- */
    if (TMR3->ists & TMR_OVF_FLAG) {
        TMR3->ists = (uint16_t)~TMR_OVF_FLAG;
    }

    /* ---- CH2(PB5转向) 捕获：先清标志再处理！（关键！把"清标志"放最前面） ---- */
    if (TMR3->ists & TMR_C2_FLAG) {
        TMR3->ists = (uint16_t)~TMR_C2_FLAG;   // ★ 第一时间清标志，最大限度避免丢下一边沿
        const uint32_t capture_value = TMR3->c2dt;  // ★ 直接读CH2捕获寄存器，不调用库函数
        const uint32_t prescaler     = TMR3->div + 1;

        if (capture_state == 0) {
            /* ============= 上升沿：记录起点 + 切"下降沿触发"(只写c2p一个位,极快) ============= */
            capture_value_prev = capture_value;
            capture_state = 1;
            TMR3->cctrl_bit.c2p = TMR_INPUT_FALLING_EDGE;  // AT32位域写法，等价于SDK宏切极性
        } else {
            /* ============= 下降沿：算高电平 + 切回"上升沿触发" ============= */
            const uint32_t high_time = (uint16_t)(capture_value - capture_value_prev);   // ★ 无符号16位减法,自动处理0xFFFF溢出(不用if/else分支)
            const uint32_t min_count = ( 500u * 120u) / prescaler;   // 合法范围  500us
            const uint32_t max_count = (3000u * 120u) / prescaler;   // 合法范围 3000us

            if (high_time > min_count && high_time < max_count) {
                pwm2_capture_high_time = (high_time * prescaler) / 120u;
                pwm2_data_ready = 1;
            }

            capture_state = 0;
            TMR3->cctrl_bit.c2p = TMR_INPUT_RISING_EDGE;   // 切回上升沿，等下一周期起点
        }
    }
}

// 陀螺仪定时器中断处理函数，没有使用了，因为17被delay占用
// void TMR17_GLOBAL_IRQHandler(void) {
//     if (TMR17->ists & TMR_OVF_FLAG) {
//         TMR17->ists = ~TMR_OVF_FLAG;
//         sensor_processor_update_gyro();
//     }
// }

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

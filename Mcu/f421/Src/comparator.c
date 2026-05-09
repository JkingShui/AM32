/*
 * comparator.c
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

#include "comparator.h"
#include "targets.h"
#include "common.h"

uint8_t getCompOutputLevel() { return CMP->ctrlsts_bit.cmpvalue; }

/**
 * @brief 禁用 指定中断线的中断功能，防止换相时触发中断
 * 
 * 该函数用于禁用比较器中断，防止在比较器输出电平变化时触发中断。
 */
void maskPhaseInterrupts()
{
    // 禁用 指定中断线的中断功能
    EXINT->inten &= ~EXTI_LINE;
    EXINT->intsts = EXTI_LINE;
}

void enableCompInterrupts() { EXINT->inten |= EXTI_LINE; }

void changeCompInput()
{
//    if (step == 1 || step == 4) { // c floating
//        CMP->ctrlsts = PHASE_C_COMP;
//    }
//    if (step == 2 || step == 5) { // a floating
//        CMP->ctrlsts = PHASE_A_COMP;
//    }
//    if (step == 3 || step == 6) { // b floating
//        CMP->ctrlsts = PHASE_B_COMP;
//    }
//    if (rising) {

//        EXINT->polcfg1 = 0;
//        EXINT->polcfg2 |= (uint32_t)EXTI_LINE;
//    } else {
//        // falling bemf
//        EXINT->polcfg1 |= (uint32_t)EXTI_LINE;
//        EXINT->polcfg2 = 0;
//    }
    /**
     * @brief 比较器速度选择
     * 
     * 该比较器速度选择根据平均换相时间间隔来确定，用于调整比较器的输出电平。
     */
    if((average_interval < 400)){ 
        //set comp to high speed mode
        CMP->ctrlsts = CMP->ctrlsts & ~(1<<2);
    }
    if((average_interval > 600)){
        //set comp to medium speed mode
        CMP->ctrlsts  = CMP->ctrlsts | 1<<2;
    }
	EXINT->polcfg1 = !rising << 21;
    EXINT->polcfg2 = rising << 21;
}

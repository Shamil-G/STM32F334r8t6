/*
 * buttons.c
 *
 *  Created on: Jan 14, 2024
 *      Author: Shamil Gusseynov
 */
#include "gpio.h"
#include "buttons.h"

extern int16_t  CHANNEL_DUTY;
void set_duty(int16_t duty);

volatile uint32_t led_delay=2000;

// Button IRQ
void EXTI15_10_IRQHandler() {
  EXTI->PR |= EXTI_PR_PR13;
//  led_delay=(led_delay==2000)?1000:500;
//  uint16_t curr_duty;
//  curr_duty=(CHANNEL_DUTY!=12000)?12000:6000;
//  set_duty(curr_duty);
}

void init_user_button(){
	InitGPio(GPIOC, 13, input, 0, low, noPull, af0);
	SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;
//	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR4_EXTI13_PC;
//	SYSCFG->EXTICR[1] |= SYSCFG_EXTICR4_EXTI13_PC;
//	SYSCFG->EXTICR[2] |= SYSCFG_EXTICR4_EXTI13_PC;

	EXTI->RTSR |= EXTI_RTSR_TR13;
	EXTI->FTSR &= ~EXTI_FTSR_TR13;
	EXTI->PR |= EXTI_PR_PR13;
	EXTI->IMR |= EXTI_IMR_MR13;
	NVIC_EnableIRQ(EXTI15_10_IRQn);
}


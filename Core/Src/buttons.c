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

volatile uint32_t led_delay=1000;

// Button IRQ
void EXTI15_10_IRQHandler() {
	if(EXTI->PR & EXTI_PR_PR13)
		EXTI->PR = EXTI_PR_PR13; //Сбрасываем флаг прерывания

    led_delay=(led_delay==1000)?250:1000;
}

void init_user_button(){
	InitGPio(GPIOC, 13, input, 0, 0, 0, 0);
	// Включаем тактирование SYSCFG!!!
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;

	EXTI->RTSR |= EXTI_RTSR_TR13;
//	EXTI->FTSR |= EXTI_FTSR_TR13;
	// Сброс флага прерывания
	EXTI->PR |= EXTI_PR_PR13;
	// Включаем прерывание на 13 канал
	EXTI->IMR |= EXTI_IMR_MR13;
	NVIC_EnableIRQ(EXTI15_10_IRQn);
//	__enable_irq();
}


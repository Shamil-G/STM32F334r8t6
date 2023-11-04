/*
 * SystemInit.c
 *
 *  Created on: Oct 31, 2023
 *      Author: Gusseynov-G
 */

#include "main.h"

// CFGR
// PLLNODIV    MCOPRE   RES  MCO  |  RES  PLLMUL    PLLXTPRE PLLSRC  |  RES   PPRE2   PPRE1  |  HPRE    SWS    SW */
//      0                   000            0       000  |   00       0000                0               0         |   00         000        000     |   0000       00        00
// [27:24]	PLLMUL = 9 = 0x1001

// Original function SystemInit(void) is in file system__stm32fxx.c
// This function using for replace original function SystemInit(void) in file system__stm32fxx.c
void SystemInit_F334(void)
{
  /* FPU settings ------------------------------------------------------------*/
  #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
  #endif

	// 1. Включаем HSI

	// 2. WAIT HSI READY как источник тактирования

	// 3. Сбрасываем все HCLK, APB предделители

	// 4. Сбрасываем HSE PLL CSS

	// 5. Сбрасыаем HSEBYP

	// 6. Сбрасываем все предделители и умножители

	// 7. Устанавливаем в начальное состояние источники сигналов для  USART, TIM, HRTIM ...

	// 8. Блокируем все прерывания


//  1
  /* Reset the RCC clock configuration to the default reset state ------------*/
  /* Set HSION bit */
    RCC->CR |= 0x00000001U;
//  2
// WAIT HSI clock enable
    while(READ_BIT(RCC->CFGR, RCC_CFGR_SWS_HSI));

//  3
  /* Reset CFGR register */
    RCC->CFGR &= 0xF87FC00CU;

// 1111 1000 0111 1111 1100 0000 0000 1100
// SW:     00 - HSI selected as system clock
// SWS:  11 - not applicable
// HPRE: 0000 - SYSCLK not divided
// PPRE1: 000 - HCLK not divided
// PPRE2: 000 - HCLK not divided
// RES: 11 - Хм!?
// PLLSRC: 1 -  HSE/PREDIV selected as PLL
// PLLXTRE: 1 - HSE input divided by 2
// PLLMUL: 1111 - PLL input clock x 16
// RES: 01 - Хм!?
// MCO: 000 - MCO output disabled
// RES: 1 - - Хм!?
// MCOPRE: 111 - MCO divided by 128
// PLLNODIV: 1 - PLL is not divided before MCO

// 4.
  /* Reset HSEON, CSSON and PLLON bits */
    RCC->CR &= 0xFEF6FFFF;
// 1111 1110 1111 0110 1111 1111 1111 1111
// HSEON: 0 - disable
// CSSON: 0 - Clock detector OFF
// PLLON: 0 - disable


// 5.
  /* Reset HSEBYP bit */
  RCC->CR &= 0xFFFBFFFFU;
// 1111 1111 1111 1011 1111 1111 1111 1111
// HSEBYP: 0 - HSE crystal oscillator not bypassed


// 6.
  /* Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE bits */
  RCC->CFGR &= 0xFF80FFFFU;
// 1111 1111 1000 0000 1111 1111 1111 1111
// PLLSRC:   0 -  HSI/2 selected as PLL
// PLLXTRE: 0 - HSE input to PLL not divided
// PLLMUL: 0000 - PLL input clock x 2


// 6-1.
  /* Reset PREDIV1[3:0] bits */
  RCC->CFGR2 &= 0xFFFFFFF0U;
  /* Reset PREDIV1[3:0] bits */
//  RCC->CFGR2 &= (uint32_t)0xFFFFFFF0;
// PREDIV: 0000 - HSE input to PLL not divided

// 7
  /* Reset USARTSW[1:0], I2CSW and TIMs bits */
  RCC->CFGR3 &= 0xFF00FCCCU;
// 1111 1111 0000 0000 1111 1100 1100 1100
// USART1SW: 00
// I2C1SW:	0
// TIM1SW:	0
// HRTIM1SW: 0


// 8
  /* Disable all interrupts */
  RCC->CIR = 0x00000000;


#if defined(USER_VECT_TAB_ADDRESS)
  SCB->VTOR = VECT_TAB_BASE_ADDRESS | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM */
#endif /* USER_VECT_TAB_ADDRESS */


//#ifdef VECT_TAB_SRAM
//  SCB->VTOR = SRAM_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM */
//#else
//  SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH */
//#endif

}


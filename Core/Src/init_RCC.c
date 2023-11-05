#include "main.h"


void PllOn_F334(void)
{
  __IO uint32_t StartUpCounter = 0, HSEStatus = 0;

#ifdef PLL_SOURCE_HSI
   /* At this stage the HSI is already enabled */
	// CFGR
	/* PLLNODIV    MCOPRE   RES  MCO  |  RES  PLLMUL    PLLXTPRE PLLSRC  |  RES   PPRE2   PPRE1  |  HPRE    SWS    SW */
	//    01        000      0   000  |  00    0000        0       0     |  00     000     000   |  0000    00     00
	// [27:24]	PLLMUL = 9 = 0x1001

     /*  PLL configuration: PLLCLK = HSI/2 *16  = 64 MHz Max frequency on PLL HSI mode*/
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL));
    RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSI_Div2 | RCC_CFGR_PLLMULL16);

#else /* PLL_SOURCE_HSE_BYPASS or PLL_SOURCE_HSE */

  /* Enable HSE */
  RCC->CR |= ((uint32_t)RCC_CR_HSEON);
#ifdef PLL_SOURCE_HSE_BYPASS
  RCC->CR |= ((uint32_t)RCC_CR_HSEBYP);
#endif   /* PLL_SOURCE_HSE_BYPASS */

  /* Wait till HSE is ready and if Time out is reached exit */
  do
  {
    HSEStatus = RCC->CR & RCC_CR_HSERDY;
    StartUpCounter++;
  } while((HSEStatus == 0) && (StartUpCounter <100));

  if ((RCC->CR & RCC_CR_HSERDY) != RESET)
  {
    HSEStatus = (uint32_t)0x01;
  }
  else
  {
    HSEStatus = (uint32_t)0x00;
  }

  if (HSEStatus == (uint32_t)0x01)
  {
         /*  PLL configuration: PLLCLK = HSE * 9 = 72 MHz */
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMUL));
    RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSE_PREDIV | RCC_CFGR_PLLMUL9);
  }

  else
  { /* If HSE fails to start-up, the application will have wrong clock
         configuration. User can add here some code to deal with this error */
  }

#endif /*PLL_SOURCE_HSI*/

    /* Enable Prefetch Buffer and set Flash Latency */
    FLASH->ACR = FLASH_ACR_PRFTBE | (HSEStatus>0)?FLASH_ACR_LATENCY_2:FLASH_ACR_LATENCY_1;
    /* HCLK = SYSCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;

    /* PCLK1 = HCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE1_DIV2;

    /* PCLK2 = HCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE2_DIV1;

      /* Enable PLL */
    RCC->CR |= RCC_CR_PLLON;

    /* Wait till PLL is ready */
    while((RCC->CR & RCC_CR_PLLRDY) == 0)
    {
    }

    /* Select PLL as system clock source */
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
    RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;

    /* Wait till PLL is used as system clock source */
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)RCC_CFGR_SWS_PLL)
    {
    }
    SystemCoreClockUpdate();

}


void SystemUp_F334(void){
	uint32_t tick=0, Ready;

	// Turn on HSE
	SET_BIT(RCC->CR, RCC_CR_HSEON);

	// Wait Ready HSE
	tick=0;
	do{
		Ready = READ_BIT(RCC->CR,RCC_CR_HSERDY);
	}while(++tick<255 && !Ready);


	if(Ready){
		/*--------------------- FLASH code must be here ------------------*/
		// So HCLK will be 72MHz - then Latency must be 2 for stm32f334
		MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_2);
		SET_BIT(FLASH->ACR, FLASH_ACR_PRFTBE);

		//	AHB
		MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_CFGR_HPRE_DIV1);
		//  APB1 Low Speed prescaler
		MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_CFGR_PPRE1_DIV2);
		//  APB2 High Speed prescaler
		MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, RCC_CFGR_PPRE2_DIV1);


		RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMUL));
	    	RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSE_PREDIV | RCC_CFGR_PLLMUL9);

		// CFGR
		/* PLLNODIV    MCOPRE   RES  MCO  |  RES  PLLMUL    PLLXTPRE PLLSRC  |  RES   PPRE2   PPRE1  |  HPRE    SWS    SW */
		//    01        000      0   000  |  00    0000        0       0     |  00     000     000   |  0000    00     00
		// [27:24]	PLLMUL = 9 = 0x1001
	    RCC->CR |= RCC_CR_PLLON;
	    /* Wait till PLL is ready */
	    while((RCC->CR & RCC_CR_PLLRDY) == 0) {}
	    /* Select PLL as system clock source */
	    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
	    RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;

	    /* Wait till PLL is used as system clock source */
	    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)RCC_CFGR_SWS_PLL){}

	}
	else{
	    // showSOS();
	    // HSE don't working
	}
	SystemCoreClockUpdate();
}

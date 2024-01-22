/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "SysTick.h"
#include "led.h"
#include "hrtim.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
extern volatile uint32_t led_delay;
extern int16_t  CHANNEL_DUTY;
extern uint16_t CHANNEL_PERIOD;
extern uint16_t HRTIM_FULL_PERIOD;
extern int16_t  CHANNEL_DUTY;
extern uint16_t repetition_rate;

uint16_t curr_duty;
uint32_t curr_freq;
uint8_t  ticks;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void PllOn_F334(void);
void enable_led1();
void initHRTIM(void);
void startHRTIM(void);
void freq_up(uint16_t delta_period);
void freq_down(uint16_t delta_period);
void duty_down(uint16_t duty);
void set_duty(uint16_t duty);
void set_freq(uint32_t freq);
void init_user_button(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//  PllOn_F334();
	SystemUp_F334();
	init_SysTick();
	enable_led1();
	initHRTIM();
	init_user_button();

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
//  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
//  MX_GPIO_Init();
//  MX_HRTIM1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  ticks_delay=0;
  ticks=0;

  startHRTIM();
  while (1)
  {
	if(ticks_delay>=led_delay){
			LED1_TOGGLE;
			ticks_delay=0;
//			ticks++;

			curr_duty=(CHANNEL_DUTY!=CHANNEL_PERIOD)?CHANNEL_PERIOD:CHANNEL_PERIOD/2;
			set_duty(curr_duty);
	}
//	if(ticks_delay>=led_delay && ticks>3){
//		ticks_delay=0;
//		ticks++;
//
//		curr_freq=(FHRCK/HRTIM_FULL_PERIOD==75000)?100000:75000;
//		set_freq(curr_freq);
//		if(ticks>7)
//			ticks=0;
//    }
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}


/**
  * @brief HRTIM1 Initialization Function
  * @param None
  * @retval None
  */

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

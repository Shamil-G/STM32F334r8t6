/*
 * hrtim.h
 *
 *  Created on: Nov 11, 2023
 *      Author: sguss
 */

#ifndef INC_HRTIM_H_
#define INC_HRTIM_H_

#define MAX_PHASE 3

#define FHRCK  4608000000

//#define PeriodTimer 	((uint16_t)45000) // f = 102,4 kHz
//#define PeriodTimer 	((uint16_t)65000) // f = 70,892 kHz
//#define PeriodTimer 	((uint16_t)65100) // f = 70,783 kHz
//#define PeriodTimer 	((uint16_t)57000) // f = 80,842 kHz
#define PeriodTimer 	((uint16_t)61440) // f = 75,000 kHz, phase_shift(4) = 18750

#define MAX_PERIOD 61440
#define MIN_PERIOD 46080



#define DLL_CALIBRATIONTIMEOUT ((uint32_t)   10)        /* Timeout in ms */
#define ADC_CONVERSION_TIME     ((uint16_t)0x480)
#define REPETITON_RATE ((uint32_t)   31) /* Define the interrupt rate vs switching frequency */

#define HRTIM_INDEX_TIMER_A 0x0U
#define HRTIM_INDEX_TIMER_B 0x1U
#define HRTIM_INDEX_TIMER_C 0x2U
#define HRTIM_INDEX_TIMER_D 0x3U
#define HRTIM_INDEX_TIMER_E 0x4U


void initHRTIM();
void startHRTIM();


#endif /* INC_HRTIM_H_ */

/*
 * init_hrtim.c
 *
 *  Created on: Oct 29, 2023
 *      Author: sguss
 */
#include "main.h"
#include "gpio.h"

// FHRCK = 4 608 000 000
// f = 102,4 kHz
#define PeriodTimer 	((uint16_t)45000)
// f = 70,3 kHz
//#define PeriodTimer 	((uint16_t)65536)
// f = 80,4 kHz
//#define PeriodTimer 	((uint16_t)57000)


uint16_t DutyHRTIM = 1000;

void InitGpio_HRTIM(void){

    /*
    * Setting I/O for output high resolution PWM:
    *
    * PA8  - HRPWM channel A output 1
    * PA9  - HRPWM channel A output 2
    * PA10 - HRPWM channel B output 1
    * PA11 - HRPWM channel B output 2
    * PC8  - HRPWM channel E output 1
    * PC9  - HRPWM channel E output 2
    *
    */
//    Gpio::Init<8,9>(GPIOA, Gpio::Mode::outputAF, Gpio::Type::PP, Gpio::Speed::veryHigh, Gpio::Pupd::pullDown, Gpio::AF::af13);
//    Gpio::Init<10,11>(GPIOA, Gpio::Mode::outputAF, Gpio::Type::PP, Gpio::Speed::veryHigh, Gpio::Pupd::pullDown, Gpio::AF::af13);
//    Gpio::Init<8,9>(GPIOC, Gpio::Mode::outputAF, Gpio::Type::PP, Gpio::Speed::veryHigh, Gpio::Pupd::pullDown, Gpio::AF::af3);

	RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;
	RCC->AHBENR  |= RCC_AHBENR_GPIOCEN;

//	void InitGPio( GPIO_TypeDef *port, unsigned int NumPin, Moder v_moder, OTyper v_type, Speed v_speed, Pupdr v_pupdr, AF v_af);
	InitGPio(GPIOA, 8, alternateF, push_pull, veryHigh, pullDown, af13);
	InitGPio(GPIOA, 9, alternateF, push_pull, veryHigh, pullDown, af13);

	InitGPio(GPIOA, 10, alternateF, push_pull, veryHigh, pullDown, af13);
	InitGPio(GPIOA, 11, alternateF, push_pull, veryHigh, pullDown, af13);

	InitGPio(GPIOC, 8, alternateF, push_pull, veryHigh, pullDown, af3);
	InitGPio(GPIOC, 9, alternateF, push_pull, veryHigh, pullDown, af3);
}

void InitHRPWM_buck_3phase (void) {
	uint32_t Ready=0;
	uint8_t  tick=0;

	InitGpio_HRTIM();
	RCC->CFGR3   |= RCC_CFGR3_HRTIM1SW_PLL;
	RCC->APB2ENR |= RCC_APB2ENR_HRTIM1EN;

    HRTIM1->sCommonRegs.DLLCR |= HRTIM_DLLCR_CAL | HRTIM_DLLCR_CALEN;   // Start timer's calibration
    do{
    	Ready = HRTIM1->sCommonRegs.ISR & HRTIM_ISR_DLLRDY;
    } while (!Ready && tick++< 100);      // Waiting for the end of calibration
    if(Ready){
    	DutyHRTIM = PeriodTimer/5;
		/************************************************
		*                          Setting Period
		***********************************************/
		HRTIM1->sTimerxRegs[0].PERxR  = PeriodTimer;   // Set period for timer A
		HRTIM1->sTimerxRegs[0].CMP1xR = DutyHRTIM;  // Set starting duty A
		HRTIM1->sTimerxRegs[1].PERxR  = PeriodTimer;   // Set period for timer B
		HRTIM1->sTimerxRegs[1].CMP1xR = DutyHRTIM;  // Set starting duty B
		HRTIM1->sTimerxRegs[4].PERxR  = PeriodTimer;   // Set period for timer E
		HRTIM1->sTimerxRegs[4].CMP1xR = DutyHRTIM;  // Set starting duty E
		/*
		* Setting to dead time for complementary output
		*/
		// Enable dead-time
		HRTIM1->sTimerxRegs[0].OUTxR |= HRTIM_OUTR_DTEN;
		// Tdtg = (2exp((DTPRSC[2:0])))) x (Thrtim / 8)
		// For DTPRSC[2:0] = 3 => Tdtg = (2*2*2)/(144Mhz * 8) = 6.94 ns (RefManual 0364, 650-651 page)
		HRTIM1->sTimerxRegs[0].DTxR  |= HRTIM_DTR_DTPRSC_0 | HRTIM_DTR_DTPRSC_1;
		// Set dead-time rising and falling Tdtr = DTR[8:0] * Tdtg = 15 * 6.94 = 104 ns
		HRTIM1->sTimerxRegs[0].DTxR  |= (15 << HRTIM_DTR_DTR_Pos) | (15 << HRTIM_DTR_DTF_Pos);
		HRTIM1->sTimerxRegs[0].DTxR  |= HRTIM_DTR_DTFSLK | HRTIM_DTR_DTRSLK;                    // Lock value dead-time

		HRTIM1->sTimerxRegs[1].OUTxR |= HRTIM_OUTR_DTEN;
		HRTIM1->sTimerxRegs[1].DTxR  |= (3 << HRTIM_DTR_DTPRSC_Pos);
		HRTIM1->sTimerxRegs[1].DTxR  |= (15 << HRTIM_DTR_DTR_Pos) | (15 << HRTIM_DTR_DTF_Pos);
		HRTIM1->sTimerxRegs[1].DTxR  |= HRTIM_DTR_DTFSLK | HRTIM_DTR_DTRSLK;

		HRTIM1->sTimerxRegs[4].OUTxR |= HRTIM_OUTR_DTEN;
		HRTIM1->sTimerxRegs[4].DTxR  |= (3 << HRTIM_DTR_DTPRSC_Pos);
		HRTIM1->sTimerxRegs[4].DTxR  |= (15 << HRTIM_DTR_DTR_Pos) | (15 << HRTIM_DTR_DTF_Pos);
		HRTIM1->sTimerxRegs[4].DTxR  |= HRTIM_DTR_DTFSLK | HRTIM_DTR_DTRSLK;
		/*
		* Start and stop event/comparator
		*/
		HRTIM1->sTimerxRegs[0].RSTxR  |= HRTIM_RSTR_MSTPER;     // Event reload timer for channel A
		HRTIM1->sTimerxRegs[0].SETx1R |= HRTIM_SET1R_MSTPER;    // Event forces the output to active state for channel A
		HRTIM1->sTimerxRegs[0].RSTx1R |= HRTIM_RST1R_CMP1;      // Event forces the output to inactive state for channel A

		HRTIM1->sTimerxRegs[1].RSTxR  |= HRTIM_RSTR_MSTCMP1;    // Event reload timer for channel B
		HRTIM1->sTimerxRegs[1].SETx1R |= HRTIM_SET1R_MSTCMP1;   // Event forces the output to active state for channel B
		HRTIM1->sTimerxRegs[1].RSTx1R |= HRTIM_RST1R_CMP1;      // Event forces the output to inactive state for channel B

		HRTIM1->sTimerxRegs[4].RSTxR  |= HRTIM_RSTR_MSTCMP2;    // Event reload timer for channel E
		HRTIM1->sTimerxRegs[4].SETx1R |= HRTIM_SET1R_MSTCMP2;   // Event forces the output to active state for channel E
		HRTIM1->sTimerxRegs[4].RSTx1R |= HRTIM_RST1R_CMP1;      // Event forces the output to inactive state for channel E

		/*
		* Select to Continuous mode + update Master timer
		*/
		HRTIM1->sTimerxRegs[0].TIMxCR |= HRTIM_TIMCR_CONT | HRTIM_TIMCR_MSTU;
		HRTIM1->sTimerxRegs[1].TIMxCR |= HRTIM_TIMCR_CONT | HRTIM_TIMCR_MSTU;
		HRTIM1->sTimerxRegs[4].TIMxCR |= HRTIM_TIMCR_CONT | HRTIM_TIMCR_MSTU;

		/*
		* Enable output HRPWM channel
		*/
		HRTIM1->sCommonRegs.OENR |= HRTIM_OENR_TA1OEN | HRTIM_OENR_TA2OEN;  // Enable output PWM channel A
		HRTIM1->sCommonRegs.OENR |= HRTIM_OENR_TB1OEN | HRTIM_OENR_TB2OEN;  // Enable output PWM channel B
		HRTIM1->sCommonRegs.OENR |= HRTIM_OENR_TE1OEN | HRTIM_OENR_TE2OEN;  // Enable output PWM channel E

		/*
		* Setting Master timer for period (frequency) and comparator for phase shift
		*/
		HRTIM1->sMasterRegs.MPER   = PeriodTimer;           // Period for master timer
		HRTIM1->sMasterRegs.MCMP1R = PeriodTimer/3;         // Phase shift 1/3 period (120 deg) for channel B
		HRTIM1->sMasterRegs.MCMP2R = PeriodTimer*2/3;         // Phase shift 2/3 period (240 deg) for channel E

		HRTIM1->sMasterRegs.MCR |= HRTIM_MCR_CONT | HRTIM_MCR_PREEN | HRTIM_MCR_MREPU;

		/*
		* Start Master timer and PWM signal to channel A,B,E
		*/
		HRTIM1->sMasterRegs.MCR |= HRTIM_MCR_MCEN | HRTIM_MCR_TACEN | HRTIM_MCR_TBCEN | HRTIM_MCR_TECEN;
    }
}


void SetDutyTimerA (uint16_t duty) {
	if(duty>=0 && duty<(PeriodTimer/3)){
		DutyHRTIM = duty;
		HRTIM1->sTimerxRegs[0].CMP1xR = duty;
		HRTIM1->sTimerxRegs[1].CMP1xR = duty;
		HRTIM1->sTimerxRegs[4].CMP1xR = duty;
	}
}

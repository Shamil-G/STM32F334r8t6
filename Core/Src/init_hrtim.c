/*
 * init_hrtim.c
 *
 *  Created on: Oct 29, 2023
 *      Author: sguss
 */
#include "main.h"
#include "gpio.h"
#include "hrtim.h"
#include "SysTick.h"

int8_t   activeTimer[] = {	HRTIM_INDEX_TIMER_A,
							HRTIM_INDEX_TIMER_B,
							HRTIM_INDEX_TIMER_C,
//							HRTIM_INDEX_TIMER_D,
							HRTIM_INDEX_TIMER_E};

volatile uint8_t  COUNT_HRTIM_CHANNEL = sizeof(activeTimer);

volatile uint16_t HRTIM_FULL_PERIOD = PeriodTimer;
volatile uint16_t CHANNEL_PERIOD = 0;
volatile int16_t  CHANNEL_DUTY = 8000;
volatile uint16_t repetition_rate = 0;

void InitGpio_HRTIM(void){

    /*
    * Setting I/O for output high resolution PWM:
    *
    * PA8  - HRPWM channel A output 1
    * PA9  - HRPWM channel A output 2
      Gpio::Init<8,9>(GPIOA, Gpio::Mode::outputAF, Gpio::Type::PP, Gpio::Speed::veryHigh, Gpio::Pupd::pullDown, Gpio::AF::af13);

    * PA10 - HRPWM channel B output 1
    * PA11 - HRPWM channel B output 2
      Gpio::Init<10,11>(GPIOA, Gpio::Mode::outputAF, Gpio::Type::PP, Gpio::Speed::veryHigh, Gpio::Pupd::pullDown, Gpio::AF::af13);

    * PB12 - HRPWM channel C output 1
    * PB13 - HRPWM channel C output 2
      Gpio::Init<12,13>(GPIOB, Gpio::Mode::outputAF, Gpio::Type::PP, Gpio::Speed::veryHigh, Gpio::Pupd::pullDown, Gpio::AF::af13);

    * PB14 - HRPWM channel D output 1
    * PB15 - HRPWM channel D output 2
      Gpio::Init<14,15>(GPIOB, Gpio::Mode::outputAF, Gpio::Type::PP, Gpio::Speed::veryHigh, Gpio::Pupd::pullDown, Gpio::AF::af13);

    * PC8  - HRPWM channel E output 1
    * PC9  - HRPWM channel E output 2
      Gpio::Init<8,9>(GPIOC, Gpio::Mode::outputAF, Gpio::Type::PP, Gpio::Speed::veryHigh, Gpio::Pupd::pullDown, Gpio::AF::af3);

    *
    */

//	void InitGPio( GPIO_TypeDef *port, unsigned int NumPin, Moder v_moder, OTyper v_type, Speed v_speed, Pupdr v_pupdr, AF v_af);
	InitGPio(GPIOA, 8, alternateF, push_pull, veryHigh, pullDown, af13);
	InitGPio(GPIOA, 9, alternateF, push_pull, veryHigh, pullDown, af13);

	InitGPio(GPIOA, 10, alternateF, push_pull, veryHigh, pullDown, af13);
	InitGPio(GPIOA, 11, alternateF, push_pull, veryHigh, pullDown, af13);

	InitGPio(GPIOB, 12, alternateF, push_pull, veryHigh, pullDown, af13);
	InitGPio(GPIOB, 13, alternateF, push_pull, veryHigh, pullDown, af13);

//	InitGPio(GPIOB, 14, alternateF, push_pull, veryHigh, pullDown, af13);
//	InitGPio(GPIOB, 15, alternateF, push_pull, veryHigh, pullDown, af13);

	InitGPio(GPIOC, 8, alternateF, push_pull, veryHigh, pullDown, af3);
	InitGPio(GPIOC, 9, alternateF, push_pull, veryHigh, pullDown, af3);
}

// STEP 2
void set_frequence(){
	HRTIM1->sMasterRegs.MPER = HRTIM_FULL_PERIOD;
	HRTIM1->sMasterRegs.MREP = REPETITON_RATE; /* 1 ISR every REPETITON_RATE PWM periods */

	HRTIM1->sMasterRegs.MPER   = HRTIM_FULL_PERIOD;           // Period for master timer
	HRTIM1->sMasterRegs.MCMP1R = (COUNT_HRTIM_CHANNEL>=1)?CHANNEL_PERIOD:0;    // When using 1 phase with 180 deg
	HRTIM1->sMasterRegs.MCMP2R = (COUNT_HRTIM_CHANNEL>=2)?CHANNEL_PERIOD*2:0;; // When using 2 phase with 120 deg for channel
	HRTIM1->sMasterRegs.MCMP3R = (COUNT_HRTIM_CHANNEL>=3)?CHANNEL_PERIOD*3:0;  // When using 3 phase with 90  deg for channel
	HRTIM1->sMasterRegs.MCMP4R = (COUNT_HRTIM_CHANNEL>=4)?CHANNEL_PERIOD*4:0;  // When using 4 phase with 72  deg for channel

	for(int8_t i=0, index_timer=0; i<COUNT_HRTIM_CHANNEL; i++){
		// Set period for timer A..E
		index_timer = activeTimer[i];
		if(index_timer!=-1){
			HRTIM1->sTimerxRegs[index_timer].PERxR  = HRTIM_FULL_PERIOD - 1 - ADC_CONVERSION_TIME;
			HRTIM1->sTimerxRegs[index_timer].CMP1xR = CHANNEL_DUTY;  	// Set starting duty
			HRTIM1->sTimerxRegs[index_timer].RSTx1R = HRTIM_RST1R_CMP1;	// Событие Таймера COMPARE 1 переводит выход в неактивное состояние для канала [i].

			HRTIM1->sTimerxRegs[index_timer].OUTxR = HRTIM_OUTR_FAULT2_1 + HRTIM_OUTR_IDLM2;
			HRTIM1->sTimerxRegs[index_timer].FLTxR = HRTIM_FLTR_FLT1EN;


			HRTIM1->sTimerxRegs[index_timer].REPxR = repetition_rate;
			HRTIM1->sTimerxRegs[index_timer].TIMxCR = HRTIM_TIMCR_MSTU 	// Обновление регистра инииируется Master таймером
													+ HRTIM_TIMCR_CONT
//														+ HRTIM_TIMCR_RETRIG 	// Таймер имеет возможность повторного запуска: сброс счетчика выполняется независимо от его состояния.
									    			+ HRTIM_TIMCR_PREEN 	// Enable preload register
										;
		}

	}
	/* ------------------------------------------- */
	/* ADC trigger intialization (with CMP2 event) */
	/* ------------------------------------------- */
	for(int8_t i=0, index_timer=0; i<COUNT_HRTIM_CHANNEL; i++){
		index_timer = activeTimer[i];
		if(index_timer!=-1){
			HRTIM1->sTimerxRegs[index_timer].CMP2xR = CHANNEL_DUTY/2; /* Выборка будет проводится на середине периода (50% of Ton time) */
		}
	}
}

void set_timing(){
	HRTIM1->sMasterRegs.MCR = HRTIM_MCR_CONT + HRTIM_MCR_PREEN + HRTIM_MCR_MREPU;
	// Master Repetition Interrupt Enable
	HRTIM1->sMasterRegs.MDIER = HRTIM_MICR_MREP;

	set_frequence();
}

// STEP 1
void initHRTIM(void) {
	uint32_t Ready=0;
	uint8_t  tick=0, index_timer;

	InitGpio_HRTIM();
	RCC->CFGR3   |= RCC_CFGR3_HRTIM1SW_PLL;
	RCC->APB2ENR |= RCC_APB2ENR_HRTIM1EN;

    HRTIM1->sCommonRegs.DLLCR |= 	HRTIM_DLLCR_CAL |
//    								HRTIM_CALIBRATIONRATE_14 | // Each 14 us, default 7.3 ms
    								HRTIM_DLLCR_CALEN;   // Start timer's calibration
    do{
    	Delay(1);
    	Ready = HRTIM1->sCommonRegs.ISR & HRTIM_ISR_DLLRDY;
    } while (!Ready && tick++< DLL_CALIBRATIONTIMEOUT);      // Waiting for the end of calibration
    if(Ready){
    	/************************************************
		* Setting Master timer for period (frequency) and comparator for phase shift
		***********************************************/
    	CHANNEL_PERIOD = HRTIM_FULL_PERIOD / COUNT_HRTIM_CHANNEL;
    	// STEP 2
    	set_timing();

//    	HRTIM1->sMasterRegs.MPER = HRTIM_FULL_PERIOD;
//		HRTIM1->sMasterRegs.MREP = REPETITON_RATE; /* 1 ISR every REPETITON_RATE PWM periods */
//		HRTIM1->sMasterRegs.MCR = HRTIM_MCR_CONT + HRTIM_MCR_PREEN + HRTIM_MCR_MREPU;
//		// Master Repetition Interrupt Enable
//		HRTIM1->sMasterRegs.MDIER = HRTIM_MICR_MREP;
//
//		/* Set compare registers for phase-shifts in master timer */
//		/* Each compare is coding for the phase-shift of one phase */
//		HRTIM1->sMasterRegs.MPER   = HRTIM_FULL_PERIOD;           // Period for master timer
//		HRTIM1->sMasterRegs.MCMP1R = (COUNT_HRTIM_CHANNEL>=1)?phase_shift:0;    // When using 1 phase with 180 deg
//		HRTIM1->sMasterRegs.MCMP2R = (COUNT_HRTIM_CHANNEL>=2)?phase_shift*2:0;; // When using 2 phase with 120 deg for channel
//		HRTIM1->sMasterRegs.MCMP3R = (COUNT_HRTIM_CHANNEL>=3)?phase_shift*3:0;  // When using 3 phase with 90  deg for channel
//		HRTIM1->sMasterRegs.MCMP4R = (COUNT_HRTIM_CHANNEL==4)?phase_shift*4:0;  // When using 4 phase with 72  deg for channel
//
//    	/************************************************
//		*                          Setting Period For Timer A..E
//		***********************************************/
//		for(int8_t i=0; i<COUNT_HRTIM_CHANNEL; i++){
//			// Set period for timer A..E
//			index_timer = activeTimer[i];
//			if(index_timer!=-1){
//				HRTIM1->sTimerxRegs[index_timer].PERxR  = HRTIM_FULL_PERIOD - 1 - ADC_CONVERSION_TIME;
//				HRTIM1->sTimerxRegs[index_timer].CMP1xR = CHANNEL_DUTY;  	// Set starting duty
//				HRTIM1->sTimerxRegs[index_timer].RSTx1R = HRTIM_RST1R_CMP1;	// Событие Таймера COMPARE 1 переводит выход в неактивное состояние для канала [i].
//
//				HRTIM1->sTimerxRegs[index_timer].REPxR = repetition_rate;
//				HRTIM1->sTimerxRegs[index_timer].TIMxCR = HRTIM_TIMCR_MSTU 	// Обновление регистра инииируется Master таймером
//														+ HRTIM_TIMCR_CONT
////														+ HRTIM_TIMCR_RETRIG 	// Таймер имеет возможность повторного запуска: сброс счетчика выполняется независимо от его состояния.
//										    			+ HRTIM_TIMCR_PREEN 	// Enable preload register
//											;
//			}
//
//		}
		HRTIM1->sTimerxRegs[activeTimer[0]].RSTxR  |= HRTIM_RSTR_MSTPER;     // Счетчик таймера сбрасывается при событии периода главного таймера.
		HRTIM1->sTimerxRegs[activeTimer[0]].SETx1R |= HRTIM_SET1R_MSTPER;    // Event forces the output to active state for channel A
		/*
		* Start and stop event/comparator
		*/
		if(COUNT_HRTIM_CHANNEL>1){
			HRTIM1->sTimerxRegs[activeTimer[1]].RSTxR  |= HRTIM_RSTR_MSTCMP1;    // Счетчик таймера сбрасывается при событии периода CMP 1.
			HRTIM1->sTimerxRegs[activeTimer[1]].SETx1R |= HRTIM_SET1R_MSTCMP1;   // Событие таймера Compare 1 переводит выход в активное состояние.
//			HRTIM1->sTimerxRegs[1].RSTx1R |= HRTIM_RST1R_CMP1;      // Event forces the output to inactive state for channel B
			if(COUNT_HRTIM_CHANNEL>2){
				HRTIM1->sTimerxRegs[activeTimer[2]].RSTxR  |= HRTIM_RSTR_MSTCMP2;    // Счетчик таймера сбрасывается при событии периода CMP 2
				HRTIM1->sTimerxRegs[activeTimer[2]].SETx1R |= HRTIM_SET1R_MSTCMP2;   // Событие таймера Compare 2 переводит выход в активное состояние.
//				HRTIM1->sTimerxRegs[4].RSTx1R |= HRTIM_RST1R_CMP1;      // Event forces the output to inactive state for channel E
				if(COUNT_HRTIM_CHANNEL>3){
					HRTIM1->sTimerxRegs[activeTimer[3]].RSTxR  |= HRTIM_RSTR_MSTCMP3;    // Счетчик таймера сбрасывается при событии периода CMP 3
					HRTIM1->sTimerxRegs[activeTimer[3]].SETx1R |= HRTIM_SET1R_MSTCMP3;   // Событие таймера Compare 3 переводит выход в активное состояние.
	//				HRTIM1->sTimerxRegs[4].RSTx1R |= HRTIM_RST1R_CMP1;      // Event forces the output to inactive state for channel E
					if(COUNT_HRTIM_CHANNEL>4){
						HRTIM1->sTimerxRegs[activeTimer[4]].RSTxR  |= HRTIM_RSTR_MSTCMP4;    // Счетчик таймера сбрасывается при событии периода CMP 4
						HRTIM1->sTimerxRegs[activeTimer[4]].SETx1R |= HRTIM_SET1R_MSTCMP4;   // Событие таймера Compare 4 переводит выход в активное состояние.
		//				HRTIM1->sTimerxRegs[4].RSTx1R |= HRTIM_RST1R_CMP1;      // Event forces the output to inactive state for channel E
					}
				}
			}
		}

		//		/*
		//		* Select to Continuous mode + update Master timer
		//		*/
		//		HRTIM1->sTimerxRegs[0].TIMxCR |= HRTIM_TIMCR_CONT | HRTIM_TIMCR_MSTU;
		//		HRTIM1->sTimerxRegs[1].TIMxCR |= HRTIM_TIMCR_CONT | HRTIM_TIMCR_MSTU;
		//		HRTIM1->sTimerxRegs[4].TIMxCR |= HRTIM_TIMCR_CONT | HRTIM_TIMCR_MSTU;
		//

//		HRTIM1->sTimerxRegs[4].RSTxR  |= HRTIM_RSTR_MSTCMP2;    // Event reload timer for channel E
//		HRTIM1->sTimerxRegs[4].SETx1R |= HRTIM_SET1R_MSTCMP2;   // Event forces the output to active state for channel E
//		HRTIM1->sTimerxRegs[4].RSTx1R |= HRTIM_RST1R_CMP1;      // Event forces the output to inactive state for channel E
		/*
		* Setting to dead time for complementary output
		*/
		for(int8_t i=0; i<COUNT_HRTIM_CHANNEL; i++){
			// Enable dead-time
			index_timer = activeTimer[i];
			if(index_timer!=-1){
				HRTIM1->sTimerxRegs[index_timer].OUTxR |= HRTIM_OUTR_DTEN;
				// Tdtg = (2exp((DTPRSC[2:0])))) x (Thrtim / 8)
				// For DTPRSC[2:0] = 3 => Tdtg = (2*2*2)/(144Mhz * 8) = 6.94 ns (RefManual 0364, 650-651 page)
				HRTIM1->sTimerxRegs[index_timer].DTxR  |= HRTIM_DTR_DTPRSC_0 | HRTIM_DTR_DTPRSC_1;
				// Set dead-time rising and falling Tdtr = DTR[8:0] * Tdtg = 15 * 6.94 = 104 ns
				HRTIM1->sTimerxRegs[index_timer].DTxR  |= (15 << HRTIM_DTR_DTR_Pos) | (15 << HRTIM_DTR_DTF_Pos);
				HRTIM1->sTimerxRegs[index_timer].DTxR  |= HRTIM_DTR_DTFSLK | HRTIM_DTR_DTRSLK;                    // Lock value dead-time
			}
		}
		/* ------------------------------------------- */
		/* ADC trigger intialization (with CMP2 event) */
		/* ------------------------------------------- */
//		for(int8_t i=0; i<COUNT_HRTIM_CHANNEL; i++){
//			index_timer = activeTimer[i];
//			if(index_timer!=-1){
//				HRTIM1->sTimerxRegs[i].CMP2xR = CHANNEL_DUTY/2; /* Samples at 50% of Ton time */
//			}
//		}
		HRTIM1->sCommonRegs.CR1 = 0; /* ADC trigger 1 update source: Master - тогда один ADC для всех каналов, сдвинутых по фазе*/

		/* Use 5 trigger sources, one per phase */
		HRTIM1->sCommonRegs.ADC1R =   HRTIM_ADC1R_AD1TAC2  /* ADC trigger event: Timer A compare 2 */
									+ HRTIM_ADC1R_AD1TBC2  /* ADC trigger event: Timer B compare 2 */
									+ HRTIM_ADC1R_AD1TCC2  /* ADC trigger event: Timer C compare 2 */
									+ HRTIM_ADC1R_AD1TDC2  /* ADC trigger event: Timer D compare 2 */
									+ HRTIM_ADC1R_AD1TEC2; /* ADC trigger event: Timer E compare 2 */

		/* ---------------------------------------------------------- */
		/* FAULT1 global init: no filter, low polarity, Fault1 enable */
		/* ---------------------------------------------------------- */
		// SET Fflts=Fhrtim/2 = 144 / 2
		HRTIM1->sCommonRegs.FLTINR2 = HRTIM_FLTINR2_FLTSD_0;
		// SET Fflt1f = Fflts/4 & N=8 : 0111 -> 222ns
		HRTIM1->sCommonRegs.FLTINR1 = HRTIM_FLTINR1_FLT1F_2 | HRTIM_FLTINR1_FLT1F_1 | HRTIM_FLTINR1_FLT1F_0;
		// SELECT FAULT SRC in HRTIM_FLTINR1
		// FAULT1 = PA12 (FLTxSRC=0): COMP2 (FLTxSRC=1),
		// FAULT2 = PA15            : COMP4
		// FAULT3 = PB10			: COMP6
		// FAULT4 = PB11
		// FAULT5 = PC7
		HRTIM1->sCommonRegs.FLTINR1 = HRTIM_FLTINR1_FLT1E;

		/* Force register update before starting */
		HRTIM1->sCommonRegs.CR2 |= HRTIM_CR2_TASWU
							   + HRTIM_CR2_TBSWU
							   + HRTIM_CR2_TCSWU
//							   + HRTIM_CR2_TDSWU
							   + HRTIM_CR2_TESWU
							   ;
    }
}

// STEP 3
/* HRTIM start-up */
void startHRTIM(){
//	HRTIM1->sCommonRegs.OENR |= HRTIM_OENR_TA1OEN | HRTIM_OENR_TA2OEN;  // Enable output PWM channel A
//	HRTIM1->sCommonRegs.OENR |= HRTIM_OENR_TB1OEN | HRTIM_OENR_TB2OEN;  // Enable output PWM channel B
//	HRTIM1->sCommonRegs.OENR |= HRTIM_OENR_TC1OEN | HRTIM_OENR_TC2OEN;  // Enable output PWM channel C
//	HRTIM1->sCommonRegs.OENR |= HRTIM_OENR_TD1OEN | HRTIM_OENR_TD2OEN;  // Enable output PWM channel D
//	HRTIM1->sCommonRegs.OENR |= HRTIM_OENR_TE1OEN | HRTIM_OENR_TE2OEN;  // Enable output PWM channel E
	uint8_t  index_timer;
	for(int8_t i=0; i<COUNT_HRTIM_CHANNEL; i++){
		// Enable dead-time
		index_timer = activeTimer[i];
		HRTIM1->sCommonRegs.OENR |= 0b11 << index_timer*2;
	}
	/*
	* Start Master timer and PWM signal to channel [i]
	*/
	HRTIM1->sMasterRegs.MCR |= HRTIM_MCR_MCEN |
			HRTIM_MCR_TACEN |
			HRTIM_MCR_TBCEN |
			HRTIM_MCR_TCCEN |
//			HRTIM_MCR_TDCEN |
			HRTIM_MCR_TECEN; // | HRTIM_MCR_TECEN;
}


void set_freq(uint32_t freq){
	if(FHRCK/freq>=MIN_PERIOD && FHRCK/freq<=MAX_PERIOD){
		HRTIM_FULL_PERIOD=FHRCK/freq;
    	CHANNEL_PERIOD = HRTIM_FULL_PERIOD / COUNT_HRTIM_CHANNEL;
    	set_frequence();
	}
}


void set_duty(uint16_t duty){
	if(duty>0 && duty <= CHANNEL_PERIOD){
		CHANNEL_DUTY=duty;
		for(int8_t i=0, index_timer=0; i<COUNT_HRTIM_CHANNEL; i++){
			// Set period for timer A..E
			index_timer = activeTimer[i];
			if(index_timer!=-1){
				HRTIM1->sTimerxRegs[index_timer].CMP1xR = CHANNEL_DUTY;
			}
		}
	}
}


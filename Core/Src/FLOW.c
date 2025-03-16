#include "stm32f0xx.h"
#include "FLOW.h"
#include <stdint.h>

void flow_init(void) {
	flow_rcc_en();
	timer_config();
}

void flow_rcc_en(void) {

	/* (1) Enable the peripheral clock of Timer 1 */
	/* (2) Enable the peripheral clock of GPIOA */
	/* (3) Select Alternate function mode (2) on GPIOA pin 8*/
	/* (4) Select TIM1_CH1 on PA10 by enabling AF2 for pin 8 in GPIOA AFRH
	 register */
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; /* (1) */
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; /* (2) */
	GPIOA->MODER &= ~(1 << 16); /* (3) */
	GPIOA->MODER |= (1 << 17);
	GPIOA->AFR[1] &= ~(1 << 0); //SETTING PA8 ALTERNATE FUNCTION MODE TO TIM1_CH1
	GPIOA->AFR[1] |= (1 << 1);
	GPIOA->AFR[1] &= ~(1 << 2);
	GPIOA->AFR[1] &= ~(1 << 3); /* (4) */

}

void timer_config(void) {

	//modified for channel 1
	/* (1) Configure channel 2 to detect rising edges on the TI2 input by
	 writing CC2S = ‘01’, and configure the input filter duration by
	 writing the IC2F[3:0] bits in the TIMx_CCMR1 register (if no filter
	 is needed, keep IC2F=0000).*/
	/* (2) Select rising edge polarity by writing CC2P=0 in the TIMx_CCER
	 register (reset value). */
	/* (3) Configure the timer in external clock mode 1 by writing SMS=111
	 Select TI2 as the trigger input source by writing TS=110
	 in the TIMx_SMCR register.*/
	/* (4) Enable the counter by writing CEN=1 in the TIMx_CR1 register. */
	TIM1->CCMR1 |= TIM_CCMR1_IC1F_0 | TIM_CCMR1_IC1F_1 | TIM_CCMR1_CC1S_0; /* (1) */
	TIM1->CCER &= (uint16_t) (~TIM_CCER_CC1P); /* (2) */
	TIM1->SMCR |= TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0; // SMS = 111 (3)
	TIM1->SMCR |= TIM_SMCR_TS_2 | TIM_SMCR_TS_0;            // TS = 101 → TI1FP1
	TIM1->SMCR &= ~(TIM_SMCR_TS_1);                                // Clear TS_1
	TIM1->CR1 |= TIM_CR1_CEN; /* (4) */

}

int measurement_function(int x) {
	uint16_t counter_value = 0;
	counter_value = TIM1->CNT;
	TIM1->CNT = 0;
	if (x==0)
	{
	return counter_value /450;
	}
	else if (x ==1)
	{
		return counter_value;
	}
	else
	{
		return 0;
	}
	//delay_ms(1000);

}

int four_sec(void) {
	delay_ms(4000);
	return 1;

}

void delay1_ms(int delay) {
	for (volatile int i = 0; i < delay * 8000; i++)
		;
}

void delay_ms(uint32_t ms) {
	// Assuming SystemCoreClock is set to the CPU clock frequency
	SysTick->LOAD = (SystemCoreClock / 8000) * ms - 1; // Load the number of clock cycles for 1 ms
	SysTick->VAL = 0; // Clear the current value register
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; // Enable the SysTick timer

	// Wait until the COUNTFLAG is set
	while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0)
		;

	// Disable the SysTick timer
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

void delay_seconds(uint32_t sec) {
	for (uint32_t i = 0; i < sec; ++i) {
		delay_ms(1000);
	}
}

void delay_minutes(uint32_t min) {
	for (uint32_t i = 0; i < min; ++i) {
		delay_seconds(60);
	}
}

#include "stm32f0xx.h"
#include "FLOW.h"
#include <stdint.h>

void flow_init(void)
{
	flow_rcc_en();
	timer_config();
}

void flow_rcc_en(void) {

	/* (1) Enable the peripheral clock of Timer 1 */
	/* (2) Enable the peripheral clock of GPIOA */
	/* (3) Select Alternate function mode (10) on GPIOA pin 9 */
	/* (4) Select TIM1_CH2 on PA9 by enabling AF2 for pin 9 in GPIOA AFRH
	 register */
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; /* (1) */
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; /* (2) */
	GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER9))
			| (GPIO_MODER_MODER9_1); /* (3) */
	GPIOA->AFR[1] |= 0x2 << ((9 - 8) * 4); /* (4) */

}

void timer_config(void) {

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
	 TIM1->CCMR1 |= TIM_CCMR1_IC2F_0 | TIM_CCMR1_IC2F_1| TIM_CCMR1_CC2S_0; /* (1) */
	 TIM1->CCER &= (uint16_t)(~TIM_CCER_CC2P); /* (2) */
	 TIM1->SMCR |= TIM_SMCR_SMS | TIM_SMCR_TS_2 | TIM_SMCR_TS_1; /* (3) */
	 TIM1->CR1 |= TIM_CR1_CEN; /* (4) */

}

int measurement_function(void)
{
	uint16_t counter_value= 0;
	counter_value= TIM1->CNT;
	TIM1->CNT = 0;
	return counter_value/7.5;
	//delay1_ms(1000);


}

void delay1_ms(int delay)
{
    for (volatile int i = 0; i < delay * 8000; i++);
}

void delay_ms(uint32_t  ms) {
    // Assuming SystemCoreClock is set to the CPU clock frequency
    SysTick->LOAD = (SystemCoreClock / 8000) * ms - 1; // Load the number of clock cycles for 1 ms
    SysTick->VAL = 0; // Clear the current value register
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; // Enable the SysTick timer

    // Wait until the COUNTFLAG is set
    while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);

    // Disable the SysTick timer
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

void delay_seconds(uint32_t sec) {
    for (uint32_t i = 0; i < sec; ++i) {
        delay_ms(1000); // Delay for 1 second
    }
}

void delay_minutes(uint32_t min) {
    for (uint32_t i = 0; i < min; ++i) {
        delay_seconds(60);
    }
}

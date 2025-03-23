/*
 * qrt_8rc.h
 *
 *  Created on: Sep 8, 2024
 *      Author: danba
 */

#ifndef SRC_QTR_8A_H_
#define SRC_QTR_8A_H_

// RCC Register Definitions for GPIO and ADC
#define RCC_AHBENR_GPIOAEN      (1U << 17) // Enable GPIOA clock
#define RCC_AHBENR_GPIOBEN      (1U << 18) // Enable GPIOB clock
#define RCC_AHBENR_GPIOCEN      (1U << 19) // Enable GPIOC clock
#define RCC_APB2ENR_ADC1EN      (1U << 9)  // Enable ADC1 clock

void clk_enable(void);
void gpio_config(void);
void adc_enable(void);
void adc_disable(void);
void adc_config(void);
void adc_init(void);
long unsigned int conv_start1(void);
long unsigned int conv_start2(void);
long unsigned int  conv_start3(void);
long unsigned int  conv_start4(void);
long unsigned int  conv_start5(void);
long unsigned int  conv_start6(void);



#endif /* SRC_QTR_8A_H_ */

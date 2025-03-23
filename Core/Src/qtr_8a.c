#include "qtr_8a.h"
#include "stm32f030x8.h"
#include <stdio.h>
#include <stdint.h>
#include "usart.h"

void clk_enable() {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; 
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN; 
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN; 
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; 
}

void gpio_config() {
    // PA0, PA1, PA4
    GPIOA->MODER |= (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 8) | (1 << 9);
    GPIOA->PUPDR &= ~((1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 8) | (1 << 9));

    // PB0
    GPIOB->MODER |= (1 << 0) | (1 << 1);
    GPIOB->PUPDR &= ~((1 << 0) | (1 << 1));

    // PC0, PC1
    GPIOC->MODER |= (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3);
    GPIOC->PUPDR &= ~((1 << 0) | (1 << 1) | (1 << 2) | (1 << 3));
}

void adc_enable(void) {
    ADC1->CR |= (1 << 0); 
}

void adc_disable(void) {
    ADC1->CR &= ~(1 << 0); 
}

void adc_config(void) {
    ADC1->SMPR |= ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_2; // Configure sampling time
}

void adc_init(void)
{
		clk_enable();
		gpio_config();
		adc_config();
}

long unsigned int conv_start1(void) {
    ADC1->CHSELR = ADC_CHSELR_CHSEL0;
    long unsigned int ADC_Result1;
    ADC1->CR |= ADC_CR_ADSTART;
    while ((ADC1->ISR & ADC_ISR_EOC) == 0) {
        // Wait for conversion to complete
    }
    ADC_Result1 = ADC1->DR;
  //  printf("Sensor %d, result: %lu\n", 1, ADC_Result1);

    return ADC_Result1;
}

long unsigned int conv_start2(void) {
    ADC1->CHSELR = ADC_CHSELR_CHSEL1;
    long unsigned int ADC_Result2;
    ADC1->CR |= ADC_CR_ADSTART;
    while ((ADC1->ISR & ADC_ISR_EOC) == 0) {
        
    }
    ADC_Result2 = ADC1->DR;
    //printf("Sensor %d, result: %lu\n", 2, ADC_Result2);

    return ADC_Result2;
}

long unsigned int conv_start3(void) {
    ADC1->CHSELR = ADC_CHSELR_CHSEL4;
    long unsigned int ADC_Result3;
    ADC1->CR |= ADC_CR_ADSTART;
    while ((ADC1->ISR & ADC_ISR_EOC) == 0) {
        
    }
    ADC_Result3 = ADC1->DR;
   // printf("Sensor %d, result: %lu\n", 3, ADC_Result3);

    return ADC_Result3;
}

long unsigned int conv_start4(void) {
    ADC1->CHSELR = ADC_CHSELR_CHSEL8;
    long unsigned int ADC_Result4;
    ADC1->CR |= ADC_CR_ADSTART;
    while ((ADC1->ISR & ADC_ISR_EOC) == 0) {
        
    }
    ADC_Result4 = ADC1->DR;
    //printf("Sensor %d, result: %lu\n", 4, ADC_Result4);

    return ADC_Result4;
}

long unsigned int conv_start5(void) {
    ADC1->CHSELR = ADC_CHSELR_CHSEL10;
    long unsigned int ADC_Result5;
    ADC1->CR |= ADC_CR_ADSTART;
    while ((ADC1->ISR & ADC_ISR_EOC) == 0) {
        
    }
    ADC_Result5 = ADC1->DR;
    //printf("Sensor %d, result: %lu\n", 5, ADC_Result5);

    return ADC_Result5;
}

long unsigned int conv_start6(void) {
    ADC1->CHSELR = ADC_CHSELR_CHSEL11;
    long unsigned int ADC_Result6;
    ADC1->CR |= ADC_CR_ADSTART;
    while ((ADC1->ISR & ADC_ISR_EOC) == 0) {
    }
    ADC_Result6 = ADC1->DR;
    //printf("Sensor %d, result: %lu\n", 6, ADC_Result6);

    return ADC_Result6;
}

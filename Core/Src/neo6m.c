/*
 * neo6m.c
 *
 *  Created on: Mar 6, 2025
 *      Author: danba
 */

#include "stm32f030x8.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "usart.h"

#define BUFFER 128;

#define GPS_BUFFER_SIZE 100
char gps_buffer[GPS_BUFFER_SIZE];
volatile uint8_t in = 0;


void neo6m_gpio_config(void) {
	RCC->AHBENR |= (1 << 17);  //ENABLING CLOCK ACCESS TO GPIOA
	RCC->APB2ENR |= (1 << 14);  //ENABLING CLOCk ACCESS TO USART1
	GPIOA->MODER &= ~(1 << 20); //SETTING PA10 MODE TO ALTERNATE FUNCTION MODE
	GPIOA->MODER |= (1 << 21);

	GPIOA->AFR[1] |= (1 << 8); //SETTING PA10 ALTERNATE FUNCTION MODE TO UART_RX
	GPIOA->AFR[1] &= ~(1 << 9);
	GPIOA->AFR[1] &= ~(1 << 10);
	GPIOA->AFR[1] &= ~(1 << 11);
}

void neo6m_config(void) {

	 /* (1) oversampling by 16, 9600 baud */
	 /* (2) 8 data bit, 1 start bit, 1 stop bit, no parity, reception mode */
	 USART1->BRR = 833; /* (1) */
	 USART1->CR1 = USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_UE; /* (2) */

}

void neo6m_init(void) {
	neo6m_gpio_config();
	neo6m_config();
}

void neo6m_read(void) {
    static uint8_t in_sentence = 0;
    if (USART1->ISR & USART_ISR_RXNE) {
        char chartoreceive = USART1->RDR;

        if (chartoreceive == '$') {
            printf("\n");
            in_sentence = 1;
        }

        if (in_sentence) {
            printf("%c", chartoreceive);
        }

        if (chartoreceive == '\n' ) {
            in_sentence = 0;
        }

        USART1->ICR |= USART_ICR_ORECF;
    }
}









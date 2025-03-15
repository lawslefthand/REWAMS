/*
 * hc12.c
 *
 *  Created on: Mar 10, 2025
 *      Author: danba
 */

#include <stdint.h>
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <qtr_8a.h>
#include "stm32f030x8.h"
#include "usart.h"
#include "FLOW.h"
#include "neo6m.h"
#include <inttypes.h>


void hc12_en(void) {
	RCC->AHBENR |= (1 << 17); //enabled port and and uart 1
	RCC->APB2ENR |= (1 << 14);

	GPIOA->MODER &=~ (1<<18); //moder to afr mode
	GPIOA->MODER |= (1<<19);

	GPIOA->AFR[1] |= (1<<4);  //pa9 set to afr 1 uart 1 tx
	GPIOA->AFR[1] &=~ (1<<5);
	GPIOA->AFR[1] &=~ (1<<6);
	GPIOA->AFR[1] &=~ (1<<7);

}

void hc12_uart_config(void)
{
	 /*  8 data bit, 1 start bit, 1 stop bit, no parity */
	 USART1->BRR = 833; /* (1) */
	 USART1->CR1 = USART_CR1_TE | USART_CR1_UE;
}

void hc12_init(void)
{
	hc12_en();
	hc12_uart_config();
}

void hc12_send(char char_to_send)
{


			while(!(USART1->ISR & (1<<7))){}

			printf("sent\n");
			USART1->TDR	=  char_to_send;

}


void hc12_write_char(unsigned char ch)
{
	/*Make sure the transmit data register is empty*/
	while(!(USART1->ISR & (1<<7))){}

	/*Write to transmit data register*/
	USART1->TDR  =  (ch & 0xFF);
}

int __io_putchar(int ch)
{
	 hc12_write_char(ch);
	return ch;
}

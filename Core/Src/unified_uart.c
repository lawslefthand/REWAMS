#include "stm32f030x8.h"
#include <stdio.h>
#include "unified_uart.h"


void usart1_gpio_config(void) {
    RCC->AHBENR |= (1 << 17);
    RCC->APB2ENR |= (1 << 14);

    // --- PA9 TX (HC-12) ---
    GPIOA->MODER &= ~(1 << 18);
    GPIOA->MODER |= (1 << 19);    // Alternate function mode
    GPIOA->AFR[1] |= (1 << 4);    // AFRH4 = 1
    GPIOA->AFR[1] &= ~(1 << 5);   // AFRH5 = 0
    GPIOA->AFR[1] &= ~(1 << 6);   // AFRH6 = 0
    GPIOA->AFR[1] &= ~(1 << 7);   // AFRH7 = 0

    // --- PA10 RX (Neo-6M) ---
    GPIOA->MODER &= ~(1 << 20);
    GPIOA->MODER |= (1 << 21);    // Alternate function mode
    GPIOA->AFR[1] |= (1 << 8);    // AFRH8 = 1
    GPIOA->AFR[1] &= ~(1 << 9);   // AFRH9 = 0
    GPIOA->AFR[1] &= ~(1 << 10);  // AFRH10 = 0
    GPIOA->AFR[1] &= ~(1 << 11);  // AFRH11 = 0
}


void usart1_config(void) {
    USART1->BRR = 833; // 9600 baud rate @ 8MHz
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;
}


void usart1_init(void) {
    usart1_gpio_config();
    usart1_config();
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


void hc12_write_char(unsigned char ch) {
    while (!(USART1->ISR & USART_ISR_TXE));
    USART1->TDR = (ch & 0xFF);
}


int __io_putchar(int ch) {
    hc12_write_char(ch);
    return ch;
}

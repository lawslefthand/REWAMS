#include "stm32f4xx.h"
#include <stdio.h>
#include "unified_uart.h"

void usart1_gpio_config(void) {
    // GPIOA and USART1 clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    // PA9 and PA10 to alternate function
    GPIOA->MODER &= ~((3 << (9 * 2)) | (3 << (10 * 2)));
    GPIOA->MODER |= (2 << (9 * 2)) | (2 << (10 * 2)); // Alternate function mode

    // alternate function to AF7 (USART1)
    GPIOA->AFR[1] &= ~((0xF << (1 * 4)) | (0xF << (2 * 4))); //
    GPIOA->AFR[1] |= (7 << (1 * 4)) | (7 << (2 * 4));        // AF7
}

void usart1_config(void) {
    USART1->BRR = 1666; // Baud rate = 9600, PCLK2 = 16MHz
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

void usart1_init(void) {
    usart1_gpio_config();
    usart1_config();
}

void neo6m_read(void) {
    static uint8_t in_sentence = 0;
    if (USART1->SR & USART_SR_RXNE) {
        char chartoreceive = USART1->DR;

        if (chartoreceive == '$') {
            printf("\n");
            in_sentence = 1;
        }

        if (in_sentence) {
            printf("%c", chartoreceive);
        }

        if (chartoreceive == '\n') {
            in_sentence = 0;
        }
    }
}

void hc12_write_char(unsigned char ch) {
    while (!(USART1->SR & USART_SR_TXE));
    USART1->DR = (ch & 0xFF);
}



int __io_putchar(int ch) {
    hc12_write_char(ch);
    return ch;
}



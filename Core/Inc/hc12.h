/*
 * hc12.h
 *
 *  Created on: Mar 10, 2025
 *      Author: danba
 */

#ifndef INC_HC12_H_
#define INC_HC12_H_


void hc12_en(void);
void hc12_uart_config(void);
void hc12_init(void);
void hc12_write_char(unsigned char ch);
void hc12_send(char char_to_send);




#endif /* INC_HC12_H_ */

/*
 * FLOW.h
 *
 *  Created on: Mar 5, 2025
 *      Author: hp
 */

#ifndef FLOW_H_
#define FLOW_H_

void flow_rcc_en(void);
void timer_config(void);
int measurement_function(void);
void delay1_ms(int delay);
void flow_init(void);
void delay_ms(uint32_t  ms);
void delay_seconds(uint32_t sec);
void delay_minutes(uint32_t min);
int four_sec(void);



#endif /* FLOW_H_ */

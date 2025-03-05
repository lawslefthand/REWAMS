/*
 * sensor_core.c
 *
 *  Created on: Mar 5, 2025
 *      Author: danba
 */

#include <stdint.h>
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <qtr_8a.h>
#include "stm32f030x8.h"
#include "sensor_core.h"

int water_detection(void) {
	int water_voltage = 0;
	int threshold = WATER_THRESHOLD;

	water_voltage = conv_start1();

	if (water_voltage > threshold) {
		return 1;
	} else {
		return 0;
	}
}

int water_turbidity(void) {
	int turbidity_reading = 0;
	int dngr_thres = dangerous_threshold;
	int caut_thres = caution_threshold;
	int safe_thres = safe_threshold;

	turbidity_reading = conv_start2();

	if (turbidity_reading >= dngr_thres) {
		return 1;
	} else if ((turbidity_reading <= dngr_thres)
			&& (turbidity_reading >= caut_thres)) {
		return 2;
	} else if ((turbidity_reading <= caut_thres)
			&& (turbidity_reading >= safe_thres)) {
		return 3;
	} else {
		return 0;
	}

}

void  gpio_a_config(void){
	RCC->AHBENR |= (1<<17);

	//pa3 set to input
	GPIOA->MODER &=~(1<<6);
	GPIOA->MODER &=~ (1<<7);
}

void gpio_b_config(void) {
	RCC->AHBENR |= (1 << 18);  //GPIO B Clk Access

	//all set to output

	GPIOB->MODER |= (1 << 2);  //PB1  ENA
	GPIOB->MODER &= ~(1 << 3);

	GPIOB->MODER |= (1 << 12); // PB6 IN1
	GPIOB->MODER &= ~(1 << 13);

	GPIOB->MODER |= (1 << 4);  //PB2  IN2
	GPIOB->MODER &= ~(1 << 5);

	GPIOB->MODER |= (1 << 6);  //PB3  ENB
	GPIOB->MODER &= ~(1 << 7);

	GPIOB->MODER |= (1 << 8);  //PB4  IN3
	GPIOB->MODER &= ~(1 << 9);

	GPIOB->MODER |= (1 << 10); //PB5 IN4
	GPIOB->MODER &= ~(1 << 11);
}

void full_speed_left(void) {
	GPIOB->ODR &= ~ GPIO_ODR_6; // IN1 high
	GPIOB->ODR |= GPIO_ODR_2;  // IN2 low

}

void full_speed_right(void) {
	GPIOB->ODR &= ~ GPIO_ODR_4;  // IN3 HIGH
	GPIOB->ODR |= GPIO_ODR_5; // IN4 LOW

}

void reverse_left(void) {
	GPIOB->ODR |= GPIO_ODR_6;  // IN1 low
	GPIOB->ODR &= ~ GPIO_ODR_2; // IN2 high

}

void reverse_right(void) {
	GPIOB->ODR |= GPIO_ODR_4; // IN3 HIGH
	GPIOB->ODR &= ~ GPIO_ODR_5;  // IN4 LOW

}

void power_reverse_right(void) {
	GPIOB->ODR |= GPIO_ODR_4; // IN3 HIGH
	GPIOB->ODR &= ~ GPIO_ODR_5;  // IN4 LOW

}

void stop_left(void) {
	GPIOB->ODR &= ~ GPIO_ODR_6; // IN1 high
	GPIOB->ODR &= ~ GPIO_ODR_2;  // IN2 low
}

void stop_right(void) {
	GPIOB->ODR &= ~GPIO_ODR_4;  // IN3 HIGH
	GPIOB->ODR &= ~GPIO_ODR_5; // IN4 LOW

}

void stop(void) {
	GPIOB->ODR &= ~GPIO_ODR_4;  // IN3 HIGH
	GPIOB->ODR &= ~GPIO_ODR_5; // IN4 LOW
	GPIOB->ODR &= ~ GPIO_ODR_6; // IN1 high
	GPIOB->ODR &= ~ GPIO_ODR_2;  // IN2 low
}


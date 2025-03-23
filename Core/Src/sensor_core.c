/*
 * sensor_core.c
 *
 *  Created on: Mar 5, 2025
 *      Author: danba   KA  BAP
 */

#include <stdint.h>
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <math.h>
#include <qtr_8a.h>
#include "stm32f030x8.h"
#include "sensor_core.h"

int water_detection(void) {

	int water_voltage = 0;
	//int threshold = WATER_THRESHOLD;

	adc_enable();
	water_voltage = conv_start1();
	adc_disable();
	printf("water voltage reading %d\n", water_voltage); //for debugging

	if (water_voltage > 0) {
		return 1;
	} else {
		return 0;
	}
}

void water_turbidity(void) {
	int turbidity_reading = 0;
	adc_enable();
	turbidity_reading = conv_start2();
	printf("Turbidity level is %d\n", turbidity_reading);
	adc_disable();
}

void mq7_gas_sensor(void) {
    int mq2_reading = 0;
    float voltage = 0.0;
    adc_enable();
    mq2_reading = conv_start3();
    adc_disable();

    voltage = (mq2_reading / 4095.0) * 3.3;

    printf("MQ7 Gas Sensor Reading: %d (Voltage: %.2f V)\n", mq2_reading, voltage);

    if (voltage < 1.0) {
        printf("Air Quality: Clean air\n");
    } else if (voltage >= 1.0 && voltage < 2.0) {
        printf("Air Quality: Moderate CO concentration\n");
    } else {
        printf("Air Quality: High  concentration detected!\n");
    }
}

void mq2_gas_sensor(void) {
    int mq9_reading = 0;
    float voltage = 0.0;
    adc_enable();
    mq9_reading = conv_start4();
    adc_disable();

    voltage = (mq9_reading / 4095.0) * 3.3;

    printf("MQ-2 Gas Sensor Reading: %d (Voltage: %.2f V)\n", mq9_reading, voltage);

    if (voltage < 0.8) {
        printf("CH4 Concentration: Low\n");
    } else if (voltage >= 0.8 && voltage < 1.5) {
        printf("CH4 Concentration: Moderate\n");
    } else {
        printf("CH4 Concentration: High - Dangerous Levels!\n");
    }
}

void mq9_gas_sensor(void) {
    int mq8_reading = 0;
    float voltage = 0.0;
    adc_enable();
    mq8_reading = conv_start5();
    adc_disable();

    voltage = (mq9_reading / 4095.0) * 3.3;

    printf("MQ-9 Gas Sensor Reading: %d (Voltage: %.2f V)\n", mq8_reading, voltage);

    if (voltage < 0.8) {
        printf("CO Concentration: Low\n");
    } else if (voltage >= 0.8 && voltage < 1.5) {
        printf("CO Concentration: Moderate\n");
    } else {
        printf("CO Concentration: High - Dangerous Levels!\n");
    }
}




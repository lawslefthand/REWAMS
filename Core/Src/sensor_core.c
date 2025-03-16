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



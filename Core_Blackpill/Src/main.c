#include <stdint.h>
#include <stdio.h>
#include <inttypes.h>
#include <qtr8a.h>
#include "stm32f4xx.h"
#include "usart.h"
#include "flow.h"
#include "sensor_core.h"
#include "bmp280_i2c.h"
#include "unified_uart.h"
#include "main.h"
#include "ISOLATION.h"
#include "welford.h"


// Welford window buffers and counters
double WELFORD_temp[WINDOW_SIZE];
double WELFORD_pressure[WINDOW_SIZE];
double WELFORD_flowrate[WINDOW_SIZE];

int index_temp = 0, count_temp = 0;
int index_press = 0, count_press = 0;
int index_flow = 0, count_flow = 0;

void update_window(double *arr, double value, int *idx, int *cnt) {
	arr[*idx] = value;
	*idx = (*idx + 1) % WINDOW_SIZE;
	if (*cnt < WINDOW_SIZE)
		(*cnt)++;
}

void run_welford_on_array(Welford *w, double *arr, int count) {
	welford_init(w);
	for (int i = 0; i < count; ++i) {
		welford_update(w, arr[i]);
	}
}

int main() {
	int j = 0, k = 0, l = 0;
	int p = 0, o = 0, s = 0, t = 0;

	double temp_array[2] = { 0 };
	double press_array[2] = { 0 };
	double flow_array[2] = { 0 };

	double temp = 0, press = 0, alt = 0;
	double temp_roc = 0, press_roc = 0;
	int flow_roc = 0;
	double temp_roc_2 = 0, press_roc_2 = 0;
	int flow_roc_2 = 0;

	int turbine_rot = 0, vol = 0, flow = 0;

	init_usart_tx();
	usart1_init();
	adc_init();
	clk_en();
	gpio_config();
	pwm_upcounter_config();
	bmp_i2c_setup();

	printf("initialization complete\n");

	while (1) {
		bmp_i2c_write(0xF5, 0xFF);
		bmp_i2c_write(0xF4, 0xFF);

		printf("GPS GEO-TAG NMEA data\n");
		for (int i = 0; i <= 800000; i++) {
			neo6m_read();
		}

		printf("Logging sensor data\n");

		delay_ms(300);
		flow = flow_rate();
		delay_ms(300);
		turbine_rot = turbine_rpm();
		delay_ms(300);
		vol = volume();

		temp = temperature(0);
		press = pressure();
		alt = altitude();

		if (j == 2) {
			j = 0;
			p = 0;
		}
		temp_array[p++] = temp;
		j++;
		temp_roc = (temp_array[1] - temp_array[0]) / (0.009520119 + 3);

		if (k == 2) {
			k = 0;
			o = 0;
		}
		press_array[o++] = press;
		k++;
		press_roc = (press_array[1] - press_array[0]) / (0.009520119 + 3);

		if (l == 2) {
			l = 0;
			s = 0;
		}
		flow_array[s++] = flow;
		l++;
		flow_roc = (flow_array[1] - flow_array[0]) / (0.009520119 + 3);

		if (water_detection() == 1)
			printf("Water detected!\n");
		else
			printf("Waiting for water\n");

		printf("Flow rate is %d Litres per Minute\n", flow);
		printf("Turbine RPM is %d\n", turbine_rot);
		printf("Volume is %d\n", vol);
		printf("temp %f\n", temp);
		printf("press %f\n", press);
		printf("Altitude %f\n", alt);

		if (t >= 1) {
			if (temp_roc > 1500 || temp_roc < -1500)
				printf("Temperature ROC %f\n", temp_roc_2);
			else {
				temp_roc_2 = temp_roc;
				printf("Temperature ROC %f\n", temp_roc_2);
			}

			if (press_roc > 1500 || press_roc < -1500)
				printf("Pressure ROC %f\n", press_roc_2);
			else {
				press_roc_2 = press_roc;
				printf("Pressure ROC %f\n", press_roc_2);
			}

			if (flow_roc > 1500 || flow_roc < -1500)
				printf("Flow rate ROC %d\n", flow_roc_2);
			else {
				flow_roc_2 = flow_roc;
				printf("Flow rate ROC %d\n", flow_roc_2);
			}

			update_window(WELFORD_temp, temp_roc_2, &index_temp, &count_temp);
			update_window(WELFORD_pressure, press_roc_2, &index_press,
					&count_press);
			update_window(WELFORD_flowrate, (double) flow_roc_2, &index_flow,
					&count_flow);

			Welford temp_stats, press_stats, flow_stats;

			run_welford_on_array(&temp_stats, WELFORD_temp, count_temp);
			run_welford_on_array(&press_stats, WELFORD_pressure, count_press);
			run_welford_on_array(&flow_stats, WELFORD_flowrate, count_flow);

			if (is_welford_anomaly(&flow_stats, flow_roc_2, 1.5))
				printf("Warning: Dangerous Anomaly in flow rate!\n");
			if (is_welford_anomaly(&temp_stats, temp_roc_2, 1.5))
				printf("Warning:Dangerous Anomaly in temperature!\n");
			if (is_welford_anomaly(&press_stats, press_roc_2, 1.5))
				printf("Warning:Dangerous Anomaly in pressure!\n");

		//	Sample sample = { .flow_roc = flow_roc_2 };

			// if (is_anomaly_isoforest(&sample, 2.03f))
			//  printf("Isolation Forest Anomaly\n");

			// if (
			//      is_anomaly_isoforest(&sample, 1.3f) &&
			//      is_welford_anomaly(&flow_stats, flow_roc_2, 1.5)
			//  ) {
			//      printf("Double-confirmed anomaly!\n");
			//  }
		}

		delay_ms(3000);
		t++;
	}
}

#include <stdint.h>
#include <stdio.h>
#include <inttypes.h>
#include <qtr8a.h>
#include "stm32f4xx.h"
#include "usart.h"
#include "flow.h"
#include "sensor_core.h"
#include "bmp280_i2c.h"
#include <inttypes.h>
#include "unified_uart.h"
#include "main.h"


//Insert extra functions here

int main() {

	//if you want new variables put here


	//dont touch established variables
	int j = 0;
	int k = 0;
	int l = 0;
	int p = 0;
	int o = 0;
	int s = 0;
	int t = 0;

	//always declare outside superloop or array forgor prev vals
	double temp_array[2];
	double press_array[2];
	double flow_array[2];

	double temp = 0;
	double press = 0;
	double alt = 0;

	double temp_roc = 0;
	double press_roc = 0;
	int flow_roc = 0;

	double temp_roc_2 = 0;
	double press_roc_2 = 0;
	int flow_roc_2 = 0;

	int turbine_rot = 0;
	int vol = 0;
	int flow = 0;

	//NO TOUCH TO INIT FUNCTION
	init_usart_tx();
	usart1_init();
	adc_init();
	clk_en();
	gpio_config();
	pwm_upcounter_config();
	bmp_i2c_setup();
	printf("initialization complete\n");
	//

	while (1) {

		int i = 0;

		//NO TOUCH TO I2C WRITE
		//bmp280 pre-init writes
		bmp_i2c_write(0xF5, 0xFF); //Recommended to apply init every loop if power loss is to be expected.
		bmp_i2c_write(0xF4, 0xFF);

		printf("GPS GEO-TAG NMEA data\n");
		while (i <= 800000) {

			neo6m_read();
			i++;
		}

		printf("Logging sensor data\n");

		delay_ms(1000);
		flow = flow_rate();
		delay_ms(1000);
		turbine_rot = turbine_rpm();
		delay_ms(1000);
		vol = volume();


		temp = temperature(0);
		press = pressure();
		alt = altitude();

		if (j == 2) {
			j = 0;
			p = 0;
		}
		temp_array[p] = temp;
		p++;
		j++;

		temp_roc = (temp_array[1] - temp_array[0]) / (0.009520119 + 3);

		if (k == 2) {
			k = 0;

			o = 0;
		}
		press_array[o] = press;
		o++;
		k++;
		press_roc = (press_array[1] - press_array[0]) / (0.009520119 + 3);

		if (l == 2) {
			l = 0;
			s = 0;
		}
		flow_array[s] = flow;
		s++;
		l++;
		flow_roc = (flow_array[1] - flow_array[0]) / (0.009520119 + 3);

		if (water_detection() == 1)
		{
			printf("Water detected!\n");
		}
		else
		{
			printf("Waiting for water\n");
		}
		printf("Flow rate is %dLitres per Minute\n", flow);
		printf("Turbine RPM is %d\n", turbine_rot);
		printf("Volume is %d\n", vol);
		printf("temp %f\n", temp);
		printf("press %f\n", press);
		printf("Altitude %f\n", alt);

		if (t >= 1) {

			//ONLY USE ROC_2 FUNCTIONS

			if (temp_roc > 1500 || temp_roc < -1500) {
				printf("Temperature ROC %f\n", temp_roc_2);
			} else {
				temp_roc_2 = temp_roc;
				printf("Temperature ROC %f\n", temp_roc_2);
			}

			if (press_roc > 1500 || press_roc < -1500) {
				printf("Pressure ROC %f\n", press_roc_2);

			} else {
				press_roc_2 = press_roc;
				printf("Pressure ROC %f\n", press_roc_2);
			}

			if (flow_roc > 1500 || flow_roc < -1500) {
				printf("Flow rate ROC %d\n", flow_roc_2);
			} else {
				flow_roc_2 = flow_roc;
				printf("Flow rate ROC %d\n", flow_roc_2);
			}

			//PUT EXTRA FUNCTION HERE

			}

		delay_ms(3000);


		//DO NOT TOUCH INCREMENT OPERATOR
		t++;

	}
}


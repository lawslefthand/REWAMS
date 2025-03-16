#include <stdint.h>
#include <stdio.h>
#include <inttypes.h>
#include <qtr_8a.h>
#include "stm32f030x8.h"
#include "usart.h"
#include "FLOW.h"
#include "neo6m.h"
#include "hc12.h"
#include "sensor_core.h"
#include "bmp280_i2c.h"
#include <inttypes.h>

int main() {

	//printf uart init (tx)
	//(uncomment for serial debugging)
	/*
	usart_gpio_init();
	usart_init();
	*/
	printf("debug initialization first\n");
	//all single use init other than main uart comes here
	//bmp_i2c_setup(); //bmp280 sensor init
	flow_init(); //yfs-201 init
	hc12_init(); //hc12 init
	neo6m_init(); //neo6m init
	adc_init(); //all adcs init

	uint16_t total_cnt = 0;
	uint16_t volume = 0;

	printf("debug initialization second\n");

	while (1) {

		int i = 0;
		

		printf("Gps NMEA Data:\n");
		while (i <= 800000) {

			neo6m_read();
			i++;
		}
		printf("Sensory data\n");
		printf("progress\n");
		water_turbidity();
		water_detection();
		printf("Flow rate is %d Litre/Min\n", measurement_function(0));
		total_cnt= measurement_function(1);
		volume = total_cnt/450;
		printf("Volume of water passed is %d\n",volume);

		calculate_ppm(3.3, 10000, 10000);

		//  bmp_i2c_write(0xF5, 0xFF); //Recommended to apply init every loop if power loss is to be expected.
		//  bmp_i2c_write(0xF4, 0xFF);
		//  printf("Temperature: %fC\n", temperature(0));
		//   pressure();
		delay_ms(2000);

	}
}


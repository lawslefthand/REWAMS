#include <stdint.h>
#include <stdio.h>
#include <inttypes.h>
#include <qtr_8a.h>
#include "stm32f030x8.h"
#include "usart.h"
#include "FLOW.h"
#include "sensor_core.h"
#include "bmp280_i2c.h"
#include <inttypes.h>
#include "unified_uart.h"

int main() {

	//printf uart init (tx)
	//usart_gpio_init();
	//usart_init();
	//printf("debug initialization first\n");
	//all single use init other than main uart comes here
	bmp_i2c_setup(); //bmp280 sensor init
	flow_init(); //yfs-201 init
	usart1_init();
	adc_init(); //all adcs init
	uint16_t volume = 0;
//
	printf("initialization complete\n");
	//delay_ms(2000);

	while (1) {

		double temp = 0;
		double press = 0;

		bmp_i2c_write(0xF5, 0xFF); //Recommended to apply init every loop if power loss is to be expected.
		bmp_i2c_write(0xF4, 0xFF);

		int i = 0;

		printf("GPS data\n");
		//printf("Gps NMEA Data:\n");
		while (i <= 800000) {

			neo6m_read();
			i++;
		}
		printf("Sensory data\n");
		//printf("progress\n");
		water_turbidity();
		water_detection();
		printf("Flow rate is %d Litre/Min\n", measurement_function());
		volume = measurement_function() / 450;
		printf("Volume of water passed is %dL\n", volume);
		mq2_gas_sensor();
		mq7_gas_sensor();

		temp = temperature(0);
		press = pressure();

		printf("temp %fC\n", temp);
		printf("press %fPa\n", press);

	}
}


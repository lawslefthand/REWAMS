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
	usart_gpio_init();
	usart_init();
	printf("debug initialization first\n");
	//all single use init other than main uart comes here
	//bmp_i2c_setup(); //bmp280 sensor init
	flow_init(); //yfs-201 init
	hc12_init(); //hc12 init
	//neo6m_init(); //neo6m init
	adc_init(); //all adcs init

	printf("debug initialization second\n");


	while (1) {

		//neo6m_read();

	//	hc12_write_char("hello");



        printf("progress\n");
        water_turbidity();
        water_detection();
        printf("Flow rate is %d Litre/Min\n", measurement_function());

      //  bmp_i2c_write(0xF5, 0xFF); //Recommended to apply init every loop if power loss is to be expected.
      //  bmp_i2c_write(0xF4, 0xFF);

      //  printf("Temperature: %fC\n", temperature(0));
     //   pressure();


		/*
		int flag1 = 0;
		int flag2 = 0;
		//all looping functions come here

		x
		printf("lol\n");

		printf("debug super loop\n");

		//if(water_detection() == 1)
		//{
		while (flag1 <= 80000000) {
			flag1++;
			//printf("debug while first\n");
			neo6m_read();

		}

		while (flag2 <= 10) {
			flag2++;
			printf("debug while second\n");






		}
		*/
		//}
		//else
		//{
		//printf("No water detected, waiting for water");
		//add delay here 2-3 seconds
		delay_ms(1000);
		//}

	}
}


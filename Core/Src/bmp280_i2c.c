#include <stdint.h>
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include "bmp280_i2c.h"
#include "stm32f030x8.h"
#include "usart.h"

void delay_1s() {
	for (uint32_t i = 0; i < 80000; i++) {
		__asm__("nop");
	}
}

void bmp_rcc_config() {
	RCC->AHBENR |= (1 << 18); //gpiob en
	RCC->APB1ENR |= (1 << 21); // i2c 1 en

}

void bmp_gpio_config() {
	GPIOB->MODER &= ~(1 << 16); //pb 8 alt mode
	GPIOB->MODER |= (1 << 17);

	GPIOB->MODER &= ~(1 << 18); //pb 9 alt mode
	GPIOB->MODER |= (1 << 19);

	GPIOB->OTYPER |= (1 << 8);   // pb 8 open drain
	GPIOB->OTYPER |= (1 << 9); // pb 9 open drain

	GPIOB->PUPDR |= (1 << 16);  //pb 8 pu
	GPIOB->PUPDR &= ~(1 << 17);

	GPIOB->PUPDR |= (1 << 18);  //pb 9 pu
	GPIOB->PUPDR &= ~(1 << 19);

	GPIOB->AFR[1] |= (1 << 0);  //pb 8 afr1 i2c scl
	GPIOB->AFR[1] &= ~(1 << 1);
	GPIOB->AFR[1] &= ~(1 << 2);
	GPIOB->AFR[1] &= ~(1 << 3);

	GPIOB->AFR[1] |= (1 << 4);  //pb9 afr1 i2c sda
	GPIOB->AFR[1] &= ~(1 << 5);
	GPIOB->AFR[1] &= ~(1 << 6);
	GPIOB->AFR[1] &= ~(1 << 7);

}



void bmp_i2c_config() {
	I2C1->CR1 &= ~ I2C_CR1_PE;

	I2C1->CR1 &= ~(1 << 12); //noise filter enabled

	I2C1->CR1 &= ~(1 << 8); // digital filter disabled
	I2C1->CR1 &= ~(1 << 9);
	I2C1->CR1 &= ~(1 << 10);
	I2C1->CR1 &= ~(1 << 11);

	I2C1->TIMINGR |= (1 << 28);   //presc
	I2C1->TIMINGR &= ~(1 << 29);
	I2C1->TIMINGR &= ~(1 << 30);
	I2C1->TIMINGR &= ~(1 << 31);

	I2C1->TIMINGR |= (1 << 0);   //scll
	I2C1->TIMINGR |= (1 << 1);
	I2C1->TIMINGR &= ~(1 << 2);
	I2C1->TIMINGR &= ~(1 << 3);
	I2C1->TIMINGR |= (1 << 4);
	I2C1->TIMINGR &= ~(1 << 5);
	I2C1->TIMINGR &= ~(1 << 6);
	I2C1->TIMINGR &= ~(1 << 7);

	I2C1->TIMINGR |= (1 << 8);   //sclh
	I2C1->TIMINGR |= (1 << 9);
	I2C1->TIMINGR |= (1 << 10);
	I2C1->TIMINGR |= (1 << 11);
	I2C1->TIMINGR &= ~(1 << 12);
	I2C1->TIMINGR &= ~(1 << 13);
	I2C1->TIMINGR &= ~(1 << 14);
	I2C1->TIMINGR &= ~(1 << 15);

	I2C1->TIMINGR &= ~(1 << 16);  //sdadel
	I2C1->TIMINGR |= (1 << 17);
	I2C1->TIMINGR &= ~(1 << 18);
	I2C1->TIMINGR &= ~(1 << 19);

	I2C1->TIMINGR &= ~(1 << 20); //scadel
	I2C1->TIMINGR &= ~(1 << 21);
	I2C1->TIMINGR |= (1 << 22);
	I2C1->TIMINGR &= ~(1 << 23);

	I2C1->CR1 &= ~(1 << 17); //clk streching enabled

	I2C1->CR1 = I2C_CR1_PE; //periph enable

}

void bmp_i2c_write(uint8_t addr, uint8_t value) {

    while (I2C1->ISR & I2C_ISR_BUSY);


    I2C1->CR2 = (2 << 16) | (0x76 << 1) | (0 << 10); // Write mode

    // Start condition
    I2C1->CR2 |= I2C_CR2_START;


    while (!(I2C1->ISR & I2C_ISR_TXIS));


    I2C1->TXDR = addr;


    while (!(I2C1->ISR & I2C_ISR_TXIS));


    I2C1->TXDR = value;


    while (!(I2C1->ISR & I2C_ISR_TC));


    I2C1->CR2 |= I2C_CR2_STOP;
}


uint8_t bmp_i2c_read(uint8_t tx_addr) {

	uint8_t rx_value = 0;

	I2C1->CR2 |= (1 << 16); //NBYTES
	I2C1->CR2 &= ~(1 << 17);
	I2C1->CR2 &= ~(1 << 18);
	I2C1->CR2 &= ~(1 << 19);
	I2C1->CR2 &= ~(1 << 20);
	I2C1->CR2 &= ~(1 << 21);
	I2C1->CR2 &= ~(1 << 22);
	I2C1->CR2 &= ~(1 << 23);

	I2C1->CR2 &= ~(1 << 25); //autoend sw RESTART mode

	I2C1->CR2 &= ~(1 << 11); //7 bit addr mode

	I2C1->CR2 &= ~(1 << 10); //write transfer

	I2C1->CR2 &= ~(1 << 1);     //0x76 SADDR
	I2C1->CR2 |= (1 << 2);
	I2C1->CR2 |= (1 << 3);
	I2C1->CR2 &= ~(1 << 4);
	I2C1->CR2 |= (1 << 5);
	I2C1->CR2 |= (1 << 6);
	I2C1->CR2 |= (1 << 7);

	//while (I2C1->ISR & (1 << 15)) {

	//	printf("busy");
	//}

//	printf("not busy\n");

	I2C1->CR2 |= (1 << 13); //start gen

//	printf("write start\n");

//	printf("txis waiting\n");
	while (!(I2C1->ISR & (1 << 1))) {

	}

//	printf("txis pass\n");

	I2C1->TXDR = tx_addr;

	while (!(I2C1->ISR & (1 << 6))) {
	}

//	printf("tx compelete write end\n");

	/////////////////////////////read start////////////////////////////////////////////////////

	I2C1->CR2 |= (1 << 16);  // NBYTES
	I2C1->CR2 &= ~(1 << 17);
	I2C1->CR2 &= ~(1 << 18);
	I2C1->CR2 &= ~(1 << 19);
	I2C1->CR2 &= ~(1 << 20);
	I2C1->CR2 &= ~(1 << 21);
	I2C1->CR2 &= ~(1 << 22);
	I2C1->CR2 &= ~(1 << 23);

	I2C1->CR2 |= (1 << 25); //autoend STOP mode

	I2C1->CR2 &= ~(1 << 11); //7 bit addr mode

	I2C1->CR2 |= (1 << 10); //read transfer

	I2C1->CR2 &= ~(1 << 1);   //0x76 SADDR
	I2C1->CR2 |= (1 << 2);
	I2C1->CR2 |= (1 << 3);
	I2C1->CR2 &= ~(1 << 4);
	I2C1->CR2 |= (1 << 5);
	I2C1->CR2 |= (1 << 6);
	I2C1->CR2 |= (1 << 7);

	I2C1->CR2 |= (1 << 13); //start gen

	while (!(I2C1->ISR & (1 << 2))) {
	}

//	printf("rxne pass\n");

	rx_value = I2C1->RXDR;

//	printf("reading");

//	printf("rxdr value is %u", rx_value);

	//while (!(I2C1->ISR & (1 << 6))) {
	//	}

//	printf("read compelete\n");

//	printf("rxdr value is %u\n", rx_value);

	return rx_value;
}

double temperature(int x) {

	unsigned short calib_T1 = 0;
	signed short calib_T2 = 0;
	signed short calib_T3 = 0;
	signed long raw_temperature = 0;
	double var1 = 0;
	double var2 = 0;
	uint32_t t_fine = 0;
	double final_temp = 0;

	//calibT1 lsb r/w
	calib_T1 = bmp_i2c_read(0x88);
	//calibT1 msb r/w
	calib_T1 |= (bmp_i2c_read(0x89) << 8);

	//printf("calib t1: %hu\n", calib_T1);

	//calibT2 lsb r/w
	calib_T2 = bmp_i2c_read(0x8A);
	//calibT2 msb r/w
	calib_T2 |= (bmp_i2c_read(0x8B) << 8);

	//printf("calib t2: %hd\n", calib_T2);

	//calibT3 lsb r/w
	calib_T3 = bmp_i2c_read(0x8C);
	//calibT3 msb r/w
	calib_T3 |= (bmp_i2c_read(0x8D) << 8);

	//printf("calib t3: %hd\n", calib_T3);

	delay_1s();
	raw_temperature = bmp_i2c_read(0xFA) << 12;   // msb
	delay_1s();
	raw_temperature |= bmp_i2c_read(0xFB) << 4;   // lsb
	delay_1s();
	raw_temperature |= bmp_i2c_read(0xFC) >> 4;   // xlsb (upper 4 bits only)

	//printf("raw temp: %ld\n", raw_temperature);

	//calc start

	var1 = (((raw_temperature / 16384.0) - (calib_T1 / 1024.0)) * calib_T2);
	var2 = (((raw_temperature / 131072.0) - (calib_T1 / 8192.0))
			* ((raw_temperature / 131072.0) - (calib_T1 / 8192.0)) * calib_T3);
	t_fine = var1 + var2;
	final_temp = t_fine / 5120.0;

//printf("final temp: %f\n", final_temp);

	if (x == 1) {
		return t_fine;
	} else if (x == 0) {
		return final_temp;
	} else {
		return 0;
	}
}

double pressure() {

	double var1 = 0;
	double var2 = 0;
	unsigned short dig_P1 = 0;
	short dig_P2 = 0;
	short dig_P3 = 0;
	short dig_P4 = 0;
	short dig_P5 = 0;
	short dig_P6 = 0;
	short dig_P7 = 0;
	short dig_P8 = 0;
	short dig_P9 = 0;
	uint32_t t_fine = 0;
	signed long raw_pressure = 0;
	double p = 0;

//   temp = temperature(0);
	t_fine = temperature(1);

//	dig_P1 lsb r/w
	dig_P1 = bmp_i2c_read(0x8E);
//dig_p1 msb r/w
	dig_P1 |= (bmp_i2c_read(0x8F) << 8);

	//dig_P2 lsb r/w
	dig_P2 = bmp_i2c_read(0x90);
	//dig_P2 msb r/w
	dig_P2 |= (bmp_i2c_read(0x91) << 8);

	// dig_P3 (0x92 / 0x93)
	dig_P3 = bmp_i2c_read(0x92);
	dig_P3 |= (bmp_i2c_read(0x93) << 8);

	// dig_P4 (0x94 / 0x95)
	dig_P4 = bmp_i2c_read(0x94);
	dig_P4 |= (bmp_i2c_read(0x95) << 8);

	// dig_P5 (0x96 / 0x97)
	dig_P5 = bmp_i2c_read(0x96);
	dig_P5 |= (bmp_i2c_read(0x97) << 8);

	// dig_P6 (0x98 / 0x99)
	dig_P6 = bmp_i2c_read(0x98);
	dig_P6 |= (bmp_i2c_read(0x99) << 8);

	// dig_P7 (0x9A / 0x9B)
	dig_P7 = bmp_i2c_read(0x9A);
	dig_P7 |= (bmp_i2c_read(0x9B) << 8);

	// dig_P8 (0x9C / 0x9D)
	dig_P8 = bmp_i2c_read(0x9C);
	dig_P8 |= (bmp_i2c_read(0x9D) << 8);

	// dig_P9 (0x9E / 0x9F)
	dig_P9 = bmp_i2c_read(0x9E);
	dig_P9 |= (bmp_i2c_read(0x9F) << 8);

	raw_pressure = bmp_i2c_read(0xF7) << 12;   // msb
	raw_pressure |= bmp_i2c_read(0xF8) << 4;   // lsb
	raw_pressure |= bmp_i2c_read(0xF9) >> 4;   // xlsb

	var1 = ((double) t_fine / 2.0) - 64000.0;
	var2 = var1 * var1 * ((double) dig_P6) / 32768.0;
	var2 = var2 * var1 * ((double) dig_P5) * 2.0;
	var2 = (var2 / 4.0) + (((double) dig_P4) * 65536.0);
	var1 = (((double) dig_P3) * var1 * var1 / 524288.0
			+ ((double) dig_P2) * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0) * ((double) dig_P1);
	p = 1048576.0 - (double) raw_pressure;
	p = (p - (var2 / 4096.0)) * 6250.0 / var1;
	var1 = ((double) dig_P9) * p * p / 2147483648.0;
	var2 = p * ((double) dig_P8) / 32768.0;
	p = p + (var1 + var2 + ((double) dig_P7)) / 16.0;

	//printf("final pressure : %f Pa\n",p);

	return p;

}

void bmp_i2c_setup() {

	bmp_rcc_config();
	bmp_gpio_config();
	bmp_i2c_config();


}

double altitude() {

	double temp_rature = 0;
	double pres_sure = 0;
	const double P0 = 101325.0;      // Sea-level standard pressure in Pa
	const double T0 = 288.15;       // Sea-level standard temperature in K
	const double L = 0.0065;        // Temperature lapse rate in K/m
	const double R = 8.3144598;     // Universal gas constant in J/(mol·K)
	const double g = 9.80665;       // Gravity in m/s^2
	const double M = 0.0289644;     // Molar mass of air in kg/mol

	temp_rature = temperature(0);
	pres_sure = pressure();

	double altitude = (temp_rature / L)
			* (1 - pow((pres_sure / P0), (R * L) / (g * M)));
	return altitude;
}

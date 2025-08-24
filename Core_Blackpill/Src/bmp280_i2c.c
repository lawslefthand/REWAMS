#include <stdint.h>
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <math.h>
#include "stm32f4xx.h"
#include "usart.h"

#define BMP280_I2C_ADDR 0x76

void delay_1s(void) {
    for (uint32_t i = 0; i < 1000000; i++) {
        __NOP();
    }
}

void bmp_rcc_config(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
}

void bmp_gpio_config(void) {
    GPIOB->MODER &= ~(GPIO_MODER_MODER8_0);
    GPIOB->MODER |= GPIO_MODER_MODER8_1;
    GPIOB->MODER &= ~(GPIO_MODER_MODER9_0);
    GPIOB->MODER |= GPIO_MODER_MODER9_1;
    GPIOB->OTYPER |= GPIO_OTYPER_OT_8;
    GPIOB->OTYPER |= GPIO_OTYPER_OT_9;
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR8_1);
    GPIOB->PUPDR |= GPIO_PUPDR_PUPDR8_0;
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR9_1);
    GPIOB->PUPDR |= GPIO_PUPDR_PUPDR9_0;
    GPIOB->AFR[1] &= ~(0xF << 0);
    GPIOB->AFR[1] |= (4 << 0);
    GPIOB->AFR[1] &= ~(0xF << 4);
    GPIOB->AFR[1] |= (4 << 4);
}

void bmp_i2c_config(void) {
    I2C1->CR1 &= ~I2C_CR1_PE;
    while(I2C1->CR1 & I2C_CR1_PE);
    I2C1->CR2 = (42 & 0x3F);
    I2C1->CCR = 210;
    I2C1->TRISE = 43;
    I2C1->CR1 |= I2C_CR1_PE;
}

void bmp_i2c_write(uint8_t addr, uint8_t value) {
    while(I2C1->SR2 & I2C_SR2_BUSY);
    I2C1->CR1 |= I2C_CR1_START;
    while(!(I2C1->SR1 & I2C_SR1_SB));
    I2C1->DR = BMP280_I2C_ADDR << 1;
    while(!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2;
    while(!(I2C1->SR1 & I2C_SR1_TXE));
    I2C1->DR = addr;
    while(!(I2C1->SR1 & I2C_SR1_TXE));
    I2C1->DR = value;
    while(!(I2C1->SR1 & I2C_SR1_BTF));
    I2C1->CR1 |= I2C_CR1_STOP;
}

uint8_t bmp_i2c_read(uint8_t reg_addr) {
    uint8_t value;
    while(I2C1->SR2 & I2C_SR2_BUSY);
    I2C1->CR1 |= I2C_CR1_START;
    while(!(I2C1->SR1 & I2C_SR1_SB));
    I2C1->DR = BMP280_I2C_ADDR << 1;
    while(!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2;
    while(!(I2C1->SR1 & I2C_SR1_TXE));
    I2C1->DR = reg_addr;
    while(!(I2C1->SR1 & I2C_SR1_BTF));
    I2C1->CR1 |= I2C_CR1_START;
    while(!(I2C1->SR1 & I2C_SR1_SB));
    I2C1->DR = (BMP280_I2C_ADDR << 1) | 0x01;
    while(!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2;
    I2C1->CR1 &= ~I2C_CR1_ACK;
    I2C1->CR1 |= I2C_CR1_STOP;
    while(!(I2C1->SR1 & I2C_SR1_RXNE));
    value = I2C1->DR;
    while(I2C1->CR1 & I2C_CR1_STOP);
    I2C1->CR1 |= I2C_CR1_ACK;
    return value;
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

    calib_T1 = bmp_i2c_read(0x88);
    calib_T1 |= (bmp_i2c_read(0x89) << 8);

    calib_T2 = bmp_i2c_read(0x8A);
    calib_T2 |= (bmp_i2c_read(0x8B) << 8);

    calib_T3 = bmp_i2c_read(0x8C);
    calib_T3 |= (bmp_i2c_read(0x8D) << 8);

    raw_temperature = bmp_i2c_read(0xFA) << 12;
    raw_temperature |= bmp_i2c_read(0xFB) << 4;
    raw_temperature |= bmp_i2c_read(0xFC) >> 4;

    var1 = (((raw_temperature / 16384.0) - (calib_T1 / 1024.0)) * calib_T2);
    var2 = (((raw_temperature / 131072.0) - (calib_T1 / 8192.0))
            * ((raw_temperature / 131072.0) - (calib_T1 / 8192.0)) * calib_T3);

    t_fine = var1 + var2;
    final_temp = t_fine / 5120.0;

    if (x == 1) {
        return t_fine;
    } else if (x == 0) {
        return final_temp;
    } else {
        return 0;
    }
}

double pressure(void) {
    double var1 = 0;
    double var2 = 0;
    unsigned short dig_P1 = 0;
    signed short dig_P2 = 0;
    signed short dig_P3 = 0;
    signed short dig_P4 = 0;
    signed short dig_P5 = 0;
    signed short dig_P6 = 0;
    signed short dig_P7 = 0;
    signed short dig_P8 = 0;
    signed short dig_P9 = 0;
    uint32_t t_fine = 0;
    signed long raw_pressure = 0;
    double p = 0;

    t_fine = temperature(1);

    dig_P1 = bmp_i2c_read(0x8E);
    dig_P1 |= (bmp_i2c_read(0x8F) << 8);

    dig_P2 = bmp_i2c_read(0x90);
    dig_P2 |= (bmp_i2c_read(0x91) << 8);

    dig_P3 = bmp_i2c_read(0x92);
    dig_P3 |= (bmp_i2c_read(0x93) << 8);

    dig_P4 = bmp_i2c_read(0x94);
    dig_P4 |= (bmp_i2c_read(0x95) << 8);

    dig_P5 = bmp_i2c_read(0x96);
    dig_P5 |= (bmp_i2c_read(0x97) << 8);

    dig_P6 = bmp_i2c_read(0x98);
    dig_P6 |= (bmp_i2c_read(0x99) << 8);

    dig_P7 = bmp_i2c_read(0x9A);
    dig_P7 |= (bmp_i2c_read(0x9B) << 8);

    dig_P8 = bmp_i2c_read(0x9C);
    dig_P8 |= (bmp_i2c_read(0x9D) << 8);

    dig_P9 = bmp_i2c_read(0x9E);
    dig_P9 |= (bmp_i2c_read(0x9F) << 8);

    raw_pressure = bmp_i2c_read(0xF7) << 12;
    raw_pressure |= bmp_i2c_read(0xF8) << 4;
    raw_pressure |= bmp_i2c_read(0xF9) >> 4;

    var1 = ((double)t_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * ((double)dig_P6) / 32768.0;
    var2 = var2 + (var1 * ((double)dig_P5) * 2.0);
    var2 = (var2 / 4.0) + (((double)dig_P4) * 65536.0);
    var1 = (((double)dig_P3) * var1 * var1 / 524288.0 + ((double)dig_P2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double)dig_P1);

    if (var1 == 0.0) {
        return 0;
    }

    p = 1048576.0 - (double)raw_pressure;
    p = (p - (var2 / 4096.0)) * 6250.0 / var1;
    var1 = ((double)dig_P9) * p * p / 2147483648.0;
    var2 = p * ((double)dig_P8) / 32768.0;
    p = p + (var1 + var2 + ((double)dig_P7)) / 16.0;

    return p;
}

void bmp_i2c_setup(void) {
    bmp_rcc_config();
    bmp_gpio_config();
    bmp_i2c_config();
    bmp_i2c_write(0xF4, 0x27);
    bmp_i2c_write(0xF5, 0x0C);
}

double altitude(void) {
    double temp_rature = 0;
    double pres_sure = 0;
    const double P0 = 101325.0;
    const double T0 = 288.15;
    const double L = 0.0065;
    const double R = 8.3144598;
    const double g = 9.80665;
    const double M = 0.0289644;

    temp_rature = temperature(0);
    pres_sure = pressure();

    double altitude = (temp_rature / L) * (1 - pow((pres_sure / P0), (R * L) / (g * M)));
    printf("Altitude: %.2f meters\n", altitude);
    return altitude;
}

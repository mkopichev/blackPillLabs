#include "../Inc/i2c.h"

void i2cInit(void) {

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN; // GPIOA and GPIOB clocking ON
	RCC->APB1ENR |= RCC_APB1ENR_I2C3EN; // I2C clocking ON

	GPIOB->MODER |= GPIO_MODER_MODE4_1; // alternate function for PB4 SDA
	GPIOB->OTYPER |= GPIO_OTYPER_OT4; // open-drain
	GPIOB->AFR[0] |= (9 << GPIO_AFRL_AFSEL4_Pos); // AF09 - SDA

	GPIOA->MODER |= GPIO_MODER_MODE8_1; // alternate function for PA8 SCL
	GPIOA->OTYPER |= GPIO_OTYPER_OT8; // open-drain
	GPIOA->AFR[1] |= (4 << GPIO_AFRH_AFSEL8_Pos); // AF04 - SCL

	I2C3->CR1 |= I2C_CR1_SWRST; // program reset
	I2C3->CR1 &= ~I2C_CR1_SWRST;

	I2C3->CR1 &= ~I2C_CR1_PE; // turn off I2C3
	I2C3->CR2 = (16 << I2C_CR2_FREQ_Pos); // APB1 HSI frequency 16 MHz
	I2C3->CCR = 80; // CCR = f_PCLK / (2 * f_SCL) = 80 if f_SCL = 100 kHz
	I2C3->TRISE = 17; // FREQ + 1
	I2C3->CR1 |= I2C_CR1_PE; // turn on I2C3
}

/**
 * Write data to the device
 * dev_addr: 7-bit device address
 * reg_addr: register address (control byte: 0x00 — command, 0x40 — data)
 * data:     write data array pointer
 * len:      number of data bytes
 * return 0: success, else - error code
 */
uint8_t i2cWriteRegister(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data,
		uint16_t len) {

	uint32_t timeout;

	I2C3->CR1 |= I2C_CR1_START; // generate START
	timeout = 10000;
	while (!(I2C3->SR1 & I2C_SR1_SB)) {
		if (--timeout == 0)
			return 1;   // error if there is no SB flag
	}

	I2C3->DR = (dev_addr << 1) | 0; // send device address with write bit 0
	timeout = 10000;
	while (!(I2C3->SR1 & I2C_SR1_ADDR)) {
		if (--timeout == 0)
			return 2;   // error if there if no ADDR flag
	}

	(void) I2C3->SR1; // reset ADDR flag: read SR1 and SR2
	(void) I2C3->SR2;

	I2C3->DR = reg_addr; // send control byte
	timeout = 10000;
	while (!(I2C3->SR1 & I2C_SR1_TXE)) {
		if (--timeout == 0)
			return 3;   // error is there is no TXE flag
	}

	for (uint16_t i = 0; i < len; i++) { // send data bytes
		I2C3->DR = data[i];
		timeout = 10000;
		while (!(I2C3->SR1 & I2C_SR1_TXE)) {
			if (--timeout == 0)
				return 4; // error if there is no TXE flag
		}
	}

	timeout = 10000;
	while (!(I2C3->SR1 & I2C_SR1_BTF)) { // wait for the transmission end
		if (--timeout == 0)
			return 5;
	}
	I2C3->CR1 |= I2C_CR1_STOP; // generate STOP

	return 0;   // success
}

/**
 * Read data from the device
 * dev_addr: 7-bit device address
 * reg_addr: register address (control byte: 0x00 — command, 0x40 — data)
 * data:     read data array pointer
 * len:      number of data bytes
 * return 0: success, else - error code
 */
uint8_t i2cReadRegister(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data,
		uint16_t len) {

	uint32_t timeout;

	if (len == 0)
		return 1; // nothing to read

	// stage 1: write register address

	I2C3->CR1 |= I2C_CR1_START; // generate START
	timeout = 10000;
	while (!(I2C3->SR1 & I2C_SR1_SB)) {
		if (--timeout == 0)
			return 2;
	}

	I2C3->DR = (dev_addr << 1) | 0; // send device address with write bit 0
	timeout = 10000;
	while (!(I2C3->SR1 & I2C_SR1_ADDR)) {
		if (--timeout == 0)
			return 3;
	}

	(void) I2C3->SR1;
	(void) I2C3->SR2;

	I2C3->DR = reg_addr; // send control byte
	timeout = 10000;
	while (!(I2C3->SR1 & I2C_SR1_TXE)) {
		if (--timeout == 0)
			return 4;
	}

	// stage 2: read data

	I2C3->CR1 |= I2C_CR1_START; // generate repeat START
	timeout = 10000;
	while (!(I2C3->SR1 & I2C_SR1_SB)) {
		if (--timeout == 0)
			return 5;
	}

	I2C3->DR = (dev_addr << 1) | 1; // send device address with read bit 1

	if (len == 1) { // set ACK/STOP relevant to bits number
		I2C3->CR1 &= ~I2C_CR1_ACK; // if there is one byte - NACK and STOP
		I2C3->CR1 |= I2C_CR1_STOP;
	} else {
		I2C3->CR1 |= I2C_CR1_ACK; // if there are many bytes - ACK
	}

	timeout = 10000; // wait for ADDR and reset it
	while (!(I2C3->SR1 & I2C_SR1_ADDR)) {
		if (--timeout == 0)
			return 6;
	}

	(void) I2C3->SR1;
	(void) I2C3->SR2;

	for (uint16_t i = 0; i < len; i++) { // receive data
		if (len > 1 && i == len - 2) {
			I2C3->CR1 &= ~I2C_CR1_ACK; // last byte - NACK and STOP
			I2C3->CR1 |= I2C_CR1_STOP;
		}
		timeout = 10000;
		while (!(I2C3->SR1 & I2C_SR1_RXNE)) {
			if (--timeout == 0)
				return 7;
		}
		data[i] = I2C3->DR;
	}

	I2C3->CR1 |= I2C_CR1_ACK; // reset ACK for the next transmission

	return 0; // success
}


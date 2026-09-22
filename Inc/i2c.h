#ifndef I2C_H_
#define I2C_H_

#include "stm32f401xc.h"

void i2cInit(void);
uint8_t i2cWriteRegister(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data,
		uint16_t len);
uint8_t i2cReadRegister(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data,
		uint16_t len);

#endif /* I2C_H_ */

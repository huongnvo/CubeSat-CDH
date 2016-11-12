#ifndef I2C_H
#define I2C_H

#include <stdint.h>

#define I2C_RESTART   	1<<8 	/* repeated start */
#define I2C_READ		2<<8    /* read a byte */

void i2c_init(uint16_t baudrate);
uint8_t *i2cRead(uint16_t address, uint16_t registery, uint16_t numOfBytes);
void i2cWrite(uint16_t address, uint16_t registery, uint16_t data);

#endif

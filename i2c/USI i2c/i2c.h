#ifndef I2C_H
#define I2C_H

#include <stdint.h>

#define I2C_RESTART   	1<<8 	/* repeated start */
#define I2C_READ		2<<8    /* read a byte */

void i2cInitialize();
uint16_t i2cRead(uint16_t address, uint16_t registery, int numOfBytes);
void i2cWrite(uint16_t address, uint16_t registery, uint16_t data);

typedef enum i2c_state_enum {
  I2C_IDLE = 0,
  I2C_START = 2,
  I2C_PREPARE_ACKNACK = 4,
  I2C_HANDLE_RXTX = 6,
  I2C_RECEIVED_DATA = 8,
  I2C_PREPARE_STOP = 10,
  I2C_STOP = 12} i2c_state_type;

extern i2c_state_type i2c_state;

// Use this to check whether a previously scheduled I2C sequence has been fully processed.
inline unsigned int i2c_done() {
  return(i2c_state == I2C_IDLE);
}

#endif
#ifndef __I2C1_H__
#define __I2C1_H__

#include "stm32f4xx.h"

typedef enum
{
    I2C1_OK = 0,
    I2C1_TIMEOUT = 1,
    I2C1_ERROR = 2,
} I2C1_Status;

void i2c1_init(uint32_t clock_speed);
I2C1_Status i2c1_write(uint8_t dev_addr, const uint8_t *data, uint16_t len, uint32_t timeout);
I2C1_Status i2c1_read(uint8_t dev_addr, uint8_t *data, uint16_t len, uint32_t timeout);
I2C1_Status i2c1_mem_write(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, uint16_t len, uint32_t timeout);
I2C1_Status i2c1_mem_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len, uint32_t timeout);

#endif

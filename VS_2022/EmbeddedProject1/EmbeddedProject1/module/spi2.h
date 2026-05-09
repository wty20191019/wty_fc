#ifndef __SPI2_H
#define __SPI2_H

#include "stm32f4xx.h"

typedef enum
{
    SPI2_OK = 0,
    SPI2_TIMEOUT,
    SPI2_ERROR
} SPI2_Status;

void spi2_init(uint16_t prescaler);
SPI2_Status spi2_transfer(const uint8_t *tx, uint8_t *rx, uint16_t len, uint32_t timeout);
SPI2_Status spi2_write(const uint8_t *data, uint16_t len, uint32_t timeout);
SPI2_Status spi2_read(uint8_t *data, uint16_t len, uint32_t timeout);

#endif /* __SPI2_H */



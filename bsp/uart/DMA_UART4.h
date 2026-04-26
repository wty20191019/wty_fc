#ifndef __DMA_UART4_H__
#define __DMA_UART4_H__

#include "stm32f4xx.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void DMA_UART4_Init(uint32_t baudrate);
void DMA_UART4_Send(const uint8_t *data, uint16_t len);
void DMA_UART4_SendString(const char *str);
uint16_t DMA_UART4_Read(uint8_t *out, uint16_t maxLen);
void Serial4_Printf(char *format, ...);

#ifdef __cplusplus
}
#endif

#endif

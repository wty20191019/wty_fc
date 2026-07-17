#ifndef __DMA_UART1_H__
#define __DMA_UART1_H__




#include "stm32f4xx.h"
#include <stdint.h>



void DMA_USART1_Init(uint32_t baudrate);//初始化USART1的DMA功能
void DMA_USART1_Send(const uint8_t *data, uint16_t len);//通过DMA发送数据
void DMA_USART1_SendString(const char *str);//通过DMA发送字符串
uint16_t DMA_USART1_Read(uint8_t *out, uint16_t maxLen);//从DMA接收缓冲区读取数据





#endif

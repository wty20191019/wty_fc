#ifndef __DMA_UART2_H
#define __DMA_UART2_H

#define DMA_USART2_DMA_Stream   DMA1_Stream5 
#define DMA_USART2				USART2
#define DMA_USART2_GPIO_AF		GPIO_AF_USART2
#define DMA_USART2_IO_CLOCK		RCC_AHB1Periph_GPIOA
#define DMA_USART2_CLOCK		RCC_APB1Periph_USART2
#define DMA_USART2_IO_PORT		GPIOA
#define DMA_USART2_TX			GPIO_Pin_2
#define DMA_USART2_RX			GPIO_Pin_3
#define DMA_USART2_TX_PinSource  GPIO_PinSource2
#define DMA_USART2_RX_PinSource  GPIO_PinSource3

extern u8 DMA_USART2_RxBuffer[300];
extern int DMA_USART2_Length;

void DMA_USART2_Init(unsigned long bound);
void DMA_USART2_Send_Char(unsigned char tx_buf);
void DMA_USART2_Send_Buff(unsigned char *tx_buf, int Length);
void DMA_USART2_Data_Prase(void);


//USART_ClearFlag(DMA_USART2, USART_FLAG_TC);
//USART_ClearITPendingBit(DMA_USART2, USART_FLAG_TXE);


#endif

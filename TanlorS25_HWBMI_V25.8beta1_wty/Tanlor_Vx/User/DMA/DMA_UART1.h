#ifndef __DMA_UART1_H
#define __DMA_UART1_H

#define DMA_USART1_DMA_Stream   DMA2_Stream5 
#define DMA_USART1				USART1
#define DMA_USART1_GPIO_AF		GPIO_AF_USART1
#define DMA_USART1_IO_CLOCK		RCC_AHB1Periph_GPIOA
#define DMA_USART1_CLOCK		RCC_APB2Periph_USART1
#define DMA_USART1_IO_PORT		GPIOA
#define DMA_USART1_TX			GPIO_Pin_9
#define DMA_USART1_RX			GPIO_Pin_10
#define DMA_USART1_TX_PinSource  GPIO_PinSource9
#define DMA_USART1_RX_PinSource  GPIO_PinSource10

extern u8 DMA_USART1_RxBuffer[300];
extern int DMA_USART1_Length;

void DMA_USART1_Init(unsigned long bound);
void DMA_USART1_Send_Char(unsigned char tx_buf);
void DMA_USART1_Send_Buff(unsigned char *tx_buf, int Length);
void DMA_USART1_Data_Prase(void);


//USART_ClearFlag(DMA_USART1, USART_FLAG_TC);
//USART_ClearITPendingBit(DMA_USART1, USART_FLAG_TXE);


#endif

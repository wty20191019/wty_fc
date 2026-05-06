#ifndef __DMA_UART6_H
#define __DMA_UART6_H

#define DMA_USART6_DMA_Stream   DMA2_Stream1 
#define DMA_USART6				USART6
#define DMA_USART6_GPIO_AF		GPIO_AF_USART6
#define DMA_USART6_IO_CLOCK		RCC_AHB1Periph_GPIOC
#define DMA_USART6_CLOCK		RCC_APB2Periph_USART6
#define DMA_USART6_IO_PORT		GPIOC
#define DMA_USART6_TX			GPIO_Pin_6
#define DMA_USART6_RX			GPIO_Pin_7
#define DMA_USART6_TX_PinSource  GPIO_PinSource6
#define DMA_USART6_RX_PinSource  GPIO_PinSource7

extern u8 DMA_USART6_RxBuffer[300];
extern int DMA_USART6_Length;

void DMA_USART6_Init(unsigned long bound);
void DMA_USART6_Send_Char(unsigned char tx_buf);
void DMA_USART6_Send_Buff(unsigned char *tx_buf, int Length);
void Lu_Jing(void);

//USART_ClearFlag(DMA_USART6, USART_FLAG_TC);
//USART_ClearITPendingBit(DMA_USART6, USART_FLAG_TXE);


#endif

#ifndef __DMA_UART3_H
#define __DMA_UART3_H

#define DMA_USART3_DMA_Stream   DMA1_Stream1
#define DMA_USART3				USART3
#define DMA_USART3_GPIO_AF		GPIO_AF_USART3
#define DMA_USART3_IO_CLOCK		RCC_AHB1Periph_GPIOB
#define DMA_USART3_CLOCK		RCC_APB1Periph_USART3
#define DMA_USART3_IO_PORT		GPIOB
#define DMA_USART3_TX			GPIO_Pin_10
#define DMA_USART3_RX			GPIO_Pin_11
#define DMA_USART3_TX_PinSource  GPIO_PinSource10
#define DMA_USART3_RX_PinSource  GPIO_PinSource11

extern u8 DMA_USART3_RxBuffer[300];
extern int DMA_USART3_Length;

void DMA_USART3_Init(unsigned long bound);
void DMA_USART3_Send_Char(unsigned char tx_buf);
void DMA_USART3_Send_Buff(unsigned char *tx_buf, int Length);
void DMA_USART3_Data_Prase(void);


//USART_ClearFlag(DMA_USART3, USART_FLAG_TC);
//USART_ClearITPendingBit(DMA_USART3, USART_FLAG_TXE);
extern double UART_MPU_ROLL , UART_MPU_Pitch , UART_MPU_Yaw ;

#endif

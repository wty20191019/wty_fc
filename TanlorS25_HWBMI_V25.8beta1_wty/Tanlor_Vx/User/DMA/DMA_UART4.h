#ifndef __DMA_UART4_H
#define __DMA_UART4_H

#define DMA_USART4_DMA_Stream   DMA1_Stream2
#define DMA_USART4				UART4
#define DMA_USART4_GPIO_AF		GPIO_AF_UART4
#define DMA_USART4_IO_CLOCK		RCC_AHB1Periph_GPIOC
#define DMA_USART4_CLOCK		RCC_APB1Periph_UART4
#define DMA_USART4_IO_PORT		GPIOC
#define DMA_USART4_TX			GPIO_Pin_10
#define DMA_USART4_RX			GPIO_Pin_11
#define DMA_USART4_TX_PinSource  GPIO_PinSource10
#define DMA_USART4_RX_PinSource  GPIO_PinSource11

extern u8 DMA_USART4_RxBuffer[300];
extern int DMA_USART4_Length;

extern u8 QR_Date[6] ;
extern u8 DMA_USART4_GET_DATA ;  //DMA_USART4的数据获取标志位
extern u8 DMA_USART4_DATA_SUM ;  //DMA_USART4的数据校验和
extern u8 QR_Date_GET_DATA ;

void DMA_USART4_Init(unsigned long bound);
void DMA_USART4_Send_Char(unsigned char tx_buf);
void DMA_USART4_Send_Buff(unsigned char *tx_buf, int Length);
void DMA_USART4_Data_Prase(void);

void DMA_USART6_Data_Prase(void);

float Ax, Ay, Az, Gx, Gy, Gz, Roll_uart, Yaw_uart, Pitch_uart;

//USART_ClearFlag(DMA_USART4, USART_FLAG_TC);
//USART_ClearITPendingBit(DMA_USART4, USART_FLAG_TXE);


#endif

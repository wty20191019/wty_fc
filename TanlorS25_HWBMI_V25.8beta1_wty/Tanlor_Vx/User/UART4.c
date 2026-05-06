#include "Headfile.h"
#include "usart.h"
#include "stdio.h"
#include "stdarg.h"	

//void USART4_Init(unsigned long bound)
//{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
//
//
//	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
//	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);
//
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_10;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
//	GPIO_Init(GPIOC, &GPIO_InitStructure);
//
//	USART_InitStructure.USART_BaudRate = bound;//波特率设置
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
//	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
//	USART_Init(UART4, &USART_InitStructure); //初始化串口1
//
//	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
//	USART_Cmd(UART4, ENABLE);
//}
//
//
//void USART4_Send(unsigned char tx_buf)
//{
//	while (USART_GetFlagStatus(UART4, USART_FLAG_TC) != 1);
//	USART_SendData(UART4, tx_buf);
//	USART_ClearFlag(UART4, USART_FLAG_TC);
//	USART_ClearITPendingBit(UART4, USART_FLAG_TXE);
//}
//void UART4_Send(unsigned char *tx_buf, int len)
//{
//	USART_ClearFlag(UART4, USART_FLAG_TC);
//	USART_ClearITPendingBit(UART4, USART_FLAG_TXE);
//	while (len--)
//	{
//		USART_SendData(UART4, *tx_buf);
//		while (USART_GetFlagStatus(UART4, USART_FLAG_TC) != 1);
//		USART_ClearFlag(UART4, USART_FLAG_TC);
//		USART_ClearITPendingBit(UART4, USART_FLAG_TXE);
//		tx_buf++;
//	}
//}
//
//u8  USART4_PRINTF_Buffer[USB_USART_REC_LEN];	//usb_printf发送缓冲区
//
//void UART4_printf(char* fmt, ...)
//{
//	u16 i, j;
//	va_list ap;
//	va_start(ap, fmt);
//	vsprintf((char*)USART4_PRINTF_Buffer, fmt, ap);
//	va_end(ap);
//	i = strlen((const char*)USART4_PRINTF_Buffer);//此次发送数据的长度
//	for (j = 0; j < i; j++)//循环发送数据
//	{
//		USART4_Send(USART4_PRINTF_Buffer[j]);
//	}
//}


//int uart4_flag = 0;
//u8 RX4_state = 0;
//u8 RX4_Data;
//u8 RX4_CND = 0;
//u8 RX4_data[20];
//s16 OpticalFlow2_data[3];
//float OpticalFlow2_data_x, OpticalFlow2_data_y, OpticalFlow2_data_dt;
//float Det_x2, Det_y2, Det_dt2, Data_valid;
//extern void Optflow_above_Statemachine(void);
//
//int UWB_data = 0;
//u32 UWB_data_cnt = 0;
//u8 UWB_state = 0;
//int UWB_RxBuffer[200] = { 0 };
//u32 UWB_posx = 0;
//u32 UWB_posy = 0;
//u32 UWB_posz = 0;
//float UWB_posxresult = 0;
//
//float UWB_posyresult = 0;
//
//float UWB_poszresult = 0;
//float UWB_poszresult_0 = 0;
//float UWB_vel_x_result = 0;
//float UWB_vel_x_result_0 = 0;
//float UWB_vel_y_result = 0;
//float UWB_vel_y_result_0 = 0;
//float UWB_vel_z_result = 0;
//float UWB_vel_z_result_0 = 0;
//u32 UWB_velx = 0;
//u32 UWB_vely = 0;
//u32 UWB_velz = 0;
//
//
///***************************************************************************************************/
//float  UWBN_X = 0;
//float  UWBN_Y = 0;


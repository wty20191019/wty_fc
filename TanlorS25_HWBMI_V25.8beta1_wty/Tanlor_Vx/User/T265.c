/*----------------------------------------------------------------------------------------------------------------------/
*               本程序只供购买者学习使用，版权著作权属于飞凡创翼团队，飞凡创翼团队将飞控程序源码提供给购买者，
*               购买者要为飞凡创翼团队提供保护，未经作者许可，不得将源代码提供给他人，不得将源代码放到网上供他人免费下载，
*               更不能以此销售牟利，如发现上述行为，飞凡创翼团队将诉之以法律解决！！！
-----------------------------------------------------------------------------------------------------------------------/
*               修改日期:2023/5/6
*               版本：TanlorS V1.0
*               版权所有，盗版必究。
*               Copyright(C) 中国民航大学飞凡创翼团队 2017-2025
*               All rights reserved
----------------------------------------------------------------------------------------------------------------------*/
#include "Headfile.h"
#include "T265.h"

#define T265_UART				USART3
#define T265_UART_CLOCK			RCC_APB1Periph_USART3
#define T265_UART_GPIO_AF		GPIO_AF_USART3
#define T265_UART_IO_CLOCK		RCC_AHB1Periph_GPIOB
#define T265_UART_IO_PORT		GPIOB
#define T265_UART_TX			GPIO_Pin_10
#define T265_UART_RX			GPIO_Pin_11
#define T265_UART_TX_PinSource  GPIO_PinSource10
#define T265_UART_RX_PinSource  GPIO_PinSource11

void USART3_Init(unsigned long bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;//定义NVIC初始化结构体

	RCC_AHB1PeriphClockCmd(T265_UART_IO_CLOCK, ENABLE);
	RCC_APB1PeriphClockCmd(T265_UART_CLOCK, ENABLE);

	GPIO_PinAFConfig(T265_UART_IO_PORT, T265_UART_TX_PinSource, T265_UART_GPIO_AF); 
	GPIO_PinAFConfig(T265_UART_IO_PORT, T265_UART_RX_PinSource, T265_UART_GPIO_AF); 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_10; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(T265_UART_IO_PORT, &GPIO_InitStructure); 
									  
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(T265_UART, &USART_InitStructure); //初始化串口1

	USART_ITConfig(T265_UART, USART_IT_RXNE, ENABLE);//rx interrupt is enable
	USART_Cmd(T265_UART, ENABLE);

}
void USART3_Send(unsigned char tx_buf)
{
	while (USART_GetFlagStatus(T265_UART, USART_FLAG_TC) != 1);
	USART_SendData(T265_UART, tx_buf);
	USART_ClearFlag(T265_UART, USART_FLAG_TC);
	USART_ClearITPendingBit(T265_UART, USART_FLAG_TXE);
}
void UART3_Send(unsigned char *tx_buf, int len)
{
	USART_ClearFlag(T265_UART, USART_FLAG_TC);
	USART_ClearITPendingBit(T265_UART, USART_FLAG_TXE);
	while (len--)
	{
		USART_SendData(T265_UART, *tx_buf);
		while (USART_GetFlagStatus(T265_UART, USART_FLAG_TC) != 1);
		USART_ClearFlag(T265_UART, USART_FLAG_TC);
		USART_ClearITPendingBit(T265_UART, USART_FLAG_TXE);
		tx_buf++;
	}
}


/***********************************************************
@函数名：UART3_IRQHandler
功能描述：串口3数据接收
@作者：FFCY 孙艺东
*************************************************************/
//void USART3_IRQHandler(void)
//{
//	if (USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == SET)
//	{
//		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
//		T265_Data_Prase(USART_ReceiveData(USART3));
//	}
//
//}
u8 T265_Data_buff[200];
u8 T265_Data_CND = 0;
u8 T265_Data_state = 0;
u8 num_deta = 4;
float T265_Data_float = 0;
char *pdata = (char *)&T265_Data_float;

pose_data ps_data = { { 0,0,0 },{ 0,0,0 },0 };
pose_data ld_data = { { 0,0,0 },{ 0,0,0 },0 };
float  T265_data_Yaw = 0.0f;

/****************************************/
//T265_Data_orgial：数据结构
//T265 X轴位置：
//T265 Y轴位置：
//T265 Z轴位置：
//T265 X轴速度：
//T265 Y轴速度：
//T265 Z轴速度：
//T265 角度：
/***************************************/
float T265_Data_orgial[7] = {0,0,0,0,0,0,0};
u8 T265_Confidence = 2;
u8 T265_Start_Flag = 0;
extern float Position_Offset_X, Position_Offset_Y;

void T265_Data_Prase(u8 Data)
{
	T265_Data_buff[T265_Data_CND++] = Data;

	if (Data == 0xff)		T265_Data_state = 1;
	else if (T265_Data_state == 1) { if (Data == 0xfe)   T265_Data_state = 2; else  T265_Data_state = 0; }
	else if (T265_Data_state == 2) { if (Data == 0xfd)   T265_Data_state = 3; else  T265_Data_state = 0; }
	else if (T265_Data_state == 3) { if (Data == 0xfc)   T265_Data_state = 4; else  T265_Data_state = 0; }
	else if (T265_Data_state == 4) { if (Data == 0xfb)   T265_Data_state = 5; else  T265_Data_state = 0; }
	else if (T265_Data_state == 5) {
		if (Data == 0xfa) {
			T265_Data_CND = 6; T265_Data_state = 0;
			T265_Data_buff[0] = 0xff;	T265_Data_buff[1] = 0xfe;	T265_Data_buff[2] = 0xfd;
			T265_Data_buff[3] = 0xfc;	T265_Data_buff[4] = 0xfb;	T265_Data_buff[5] = 0xfa;
		}
		else  T265_Data_state = 0;
	}

	if (T265_Data_CND == 34)
	{
		if (T265_Data_buff[0] == 0xff && T265_Data_buff[1] == 0xfe && T265_Data_buff[2] == 0xfd && T265_Data_buff[3] == 0xfc && T265_Data_buff[4] == 0xfb && T265_Data_buff[5] == 0xfa)
		{
			ps_data.Is_normal = 1;   //T265接收数据正常
			for (u8 i = 0; i < 7; i++)
			{
				pdata = (char *)&T265_Data_float;
				*pdata++ = T265_Data_buff[6+i*4];	*pdata++ = T265_Data_buff[7+ i * 4];	*pdata++ = T265_Data_buff[8+ i * 4];	*pdata++ = T265_Data_buff[9+ i * 4];
				T265_Data_orgial[i]= T265_Data_float;
			}
		
			ps_data.translation.y = T265_Data_orgial[0] * 100; //  cm 机头向前，往右y增加
			ps_data.translation.z = T265_Data_orgial[1] * 100; //  cm 往上增加
			ps_data.translation.x = -T265_Data_orgial[2] * 100; //  cm 机头向前，往前x增加
			T265_data_Yaw = T265_Data_orgial[6] * 180 / 3.141592653;

			/****************T265原始坐标系转化为无人机机体坐标系*********************/
			ps_data.velocity.y = (T265_Data_orgial[3] * 100) *cos(T265_data_Yaw*DEG2RAD) + (-T265_Data_orgial[5] * 100)*sin(T265_data_Yaw*DEG2RAD);  //  cm/s 机头向前，往右y增加
			ps_data.velocity.z = T265_Data_orgial[4] * 100; //  cm/s 往上增加
			ps_data.velocity.x = (-T265_Data_orgial[5] * 100)*cos(T265_data_Yaw*DEG2RAD) - (T265_Data_orgial[3] * 100)*sin(T265_data_Yaw*DEG2RAD);   //  cm/s 机头向前，往前x增加		
		}
	}

	T265_Confidence = T265_Data_buff[34];
	Simu_T265_Wrong(0);

	//启动之后，先动一动T265
	if (T265_Start_Flag == 0 && T265_Confidence == 3)
	{
		T265_Start_Flag = 1;
	}
}

u8 Oled_Show_buff[25];
void T265_wait_and_check()
{
	while (ps_data.Is_normal == 0)
	{
		OLED_LStr(0, 2, "                     ");
		delay_ms(500);
		OLED_LStr(0, 2, "      Wait_T265      ");
		delay_ms(500);

		if (Key_Down == 0)
		{
			delay_ms(100);
			if (Key_Down == 0)          return;
		}

	}
	OLED_LStr(0, 2, "Wait_T265        OK  ");
	OLED_LStr(0, 3, "T265_data            ");
	OLED_LStr(0, 4, "Pos_X:               ");
	OLED_LStr(0, 5, "Pos_Y:               ");
	OLED_LStr(0, 6, "Pos_Z:               ");

	while (1)
	{
		sprintf(Oled_Show_buff, "%4.4f  ", ps_data.translation.x);
		OLED_LStr(40, 4, Oled_Show_buff);
		sprintf(Oled_Show_buff, "%4.4f  ", ps_data.translation.y);
		OLED_LStr(40, 5, Oled_Show_buff);
		sprintf(Oled_Show_buff, "%4.4f  ", ps_data.translation.z);
		OLED_LStr(40, 6, Oled_Show_buff);

		if (Key_Down == 0)
		{
			delay_ms(100);
			if (Key_Down == 0)          break;
		}
	}
	ps_data.Is_normal = 1;
	LCD_CLS();
	OLED_LStr(0, 4, "------T265_OK-------");
	delay_ms(500);
	LCD_CLS();
}
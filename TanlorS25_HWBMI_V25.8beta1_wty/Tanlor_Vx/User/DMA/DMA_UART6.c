#include "Headfile.h"
#include "DMA_UART6.h"



void DMA_USART6_Init(unsigned long bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;//定义NVIC初始化结构体
	DMA_InitTypeDef  DMA_InitStructure;

	RCC_AHB1PeriphClockCmd(DMA_USART6_IO_CLOCK, ENABLE);
	RCC_APB2PeriphClockCmd(DMA_USART6_CLOCK, ENABLE);

	GPIO_PinAFConfig(DMA_USART6_IO_PORT, DMA_USART6_TX_PinSource, DMA_USART6_GPIO_AF);
	GPIO_PinAFConfig(DMA_USART6_IO_PORT, DMA_USART6_RX_PinSource, DMA_USART6_GPIO_AF);

	GPIO_InitStructure.GPIO_Pin = DMA_USART6_TX | DMA_USART6_RX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //上拉
	GPIO_Init(DMA_USART6_IO_PORT, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(DMA_USART6, &USART_InitStructure); //初始化串口

	USART_ClearFlag(DMA_USART6, USART_FLAG_TC);//清除中断标志位
	USART_ITConfig(DMA_USART6, USART_IT_IDLE, ENABLE);//开启空闲中断，串口接收数据用到了空闲中断，因此必须开启！

	USART_DMACmd(DMA_USART6, USART_DMAReq_Rx, ENABLE);  //使能串口1的DMA接收,这两句与DMA传输相关，一定要使能！！！！！！！！！！！
	USART_DMACmd(DMA_USART6, USART_DMAReq_Tx, ENABLE);  //使能串口1的DMA发送，这两句与DMA传输相关，一定要使能！！！！！！！！！！！


														//DMAx的各通道配置,外设到存储器
														//这里的传输形式是固定的,这点要根据不同的情况来修改
														//从存储器->外设模式/8位数据宽度/存储器增量模式
														//DMA_Streamx:DMA数据流,DMA1_Stream0~7/DMA2_Stream0~7
														//chx:DMA通道选择,@ref DMA_channel DMA_Channel_0~DMA_Channel_7
														//par:外设地址
														//mar:存储器地址
														//ndtr:数据传输量  


	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);//DMA1时钟使能 
														/* 配置 DMA Stream */
	DMA_InitStructure.DMA_Channel = DMA_Channel_5;  //通道选择
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(DMA_USART6->DR));//DMA外设地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)DMA_USART6_RxBuffer;//DMA 存储器0地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//外设模式到存储器！！！，这个配置很关键
	DMA_InitStructure.DMA_BufferSize = 256;//数据传输量 
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据长度:8位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//存储器数据长度:8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;// 使用普通模式,不循环
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;//中等优先级
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
																	   //DMA1的数据流1
	DMA_Init(DMA_USART6_DMA_Stream, &DMA_InitStructure);//初始化DMA Stream
	DMA_Cmd(DMA_USART6_DMA_Stream, ENABLE);             //开启DMA传输,只是与前面配置不同的，注意！


	USART_Cmd(DMA_USART6, ENABLE);                    //使能串口

}



void DMA_USART6_Send_Char(unsigned char tx_buf)
{
	USART_SendData(DMA_USART6, tx_buf);
	while (USART_GetFlagStatus(DMA_USART6, USART_FLAG_TC) != 1);
}

void DMA_USART6_Send_Buff(unsigned char *tx_buf, int Length)
{
	uint16_t i;
	for (i = 0; i < Length; i++)		//遍历数组
	{
		DMA_USART6_Send_Char(tx_buf[i]);		//依次调用BusServo_SendByte发送每个字节数据
	}
}




void USART6_IRQHandler(void)                	//串口1中断服务程序
{
	if (USART_GetITStatus(DMA_USART6, USART_IT_IDLE) != RESET)//接收到一条完整数据进入空闲中断
	{
		USART_ClearITPendingBit(DMA_USART6, USART_IT_IDLE);//清除中断标志
		DMA_USART6->SR;  		DMA_USART6->DR;       //清除中断标志
		DMA_Cmd(DMA_USART6_DMA_Stream, DISABLE);                           //关闭DMA传输,此处一定要关闭，不然后面判断过不了！！！
		DMA_USART6_Length = 256 - DMA_GetCurrDataCounter(DMA_USART6_DMA_Stream);    //得到当前传输数据量
		DMA_USART6_DMA_Stream->NDTR = 256;

		DMA_Cmd(DMA_USART6_DMA_Stream, ENABLE);                            //开启DMA传输！！！！！！！！！！！
	}
}


u8 Avoid_X[5] = { 0 }, Avoid_Y[5] = { 0 };
extern u8 p_count;
u8 Fst_Been = 0;
u8 Sec_Been = 0;
u8 Thr_Been = 0;
u8 Been_Suanshu = 1;
u8 Start_Way = 0;

struct place {
	u8 x0;
	u8 y0;
	u8 cc;
	u8 av;
};
struct place p_place[3];//从右到左   从下到上
struct place lujing[30];
u8 p_count = 1;
u8 lu_count = 0;
u16 Lu_Cnd = 0;
u8 head_p;
u8 tail_p;
u16 count = 1;
u16 light_count = 150;
float Targ_Pos[3][30] = { 0.0f };
u8 avoid_hang = 0;
u8 avoid_lie = 0;
u8 hanglie_flag = 0;
u8 Next_Tar_Flag = 0;
u8 BackRoad_Flag = 0;
extern u8 AUTO_FLY_FLAG;
extern uint8_t SDK_Ctrl_Mode_LY;
extern u8 Been_Traverse[10][10];

void DMA_USART6_Data_Prase()
{

	if ((DMA_USART6_RxBuffer[0] == 0x00 && DMA_USART6_RxBuffer[1] == 0xff))
	{
		Avoid_X[0] = DMA_USART6_RxBuffer[2];
		Avoid_Y[0] = DMA_USART6_RxBuffer[3];
		Avoid_X[1] = DMA_USART6_RxBuffer[4];
		Avoid_Y[1] = DMA_USART6_RxBuffer[5];
		Avoid_X[2] = DMA_USART6_RxBuffer[6];
		Avoid_Y[2] = DMA_USART6_RxBuffer[7];

		p_place[0].y0 = Avoid_Y[0];
		p_place[0].x0 = Avoid_X[0];
		p_place[1].y0 = Avoid_Y[1];
		p_place[1].x0 = Avoid_X[1];
		p_place[2].y0 = Avoid_Y[2];
		p_place[2].x0 = Avoid_X[2];

		if (Been_Suanshu)
		{
			memset(Been_Traverse, 0, sizeof(Been_Traverse));
			Been_Traverse[p_place[0].x0][p_place[0].y0] = 1;
			Been_Traverse[p_place[1].x0][p_place[1].y0] = 1;
			Been_Traverse[p_place[2].x0][p_place[2].y0] = 1;
			p_count = 1;
			Been_Suanshu = 0;
		}

	}
}



void Lu_Jing(void) {

	if (p_count)
	{
		p_count = 0;
		lu_count = 0;
		lujing[lu_count].x0 = 1;
		lujing[lu_count].y0 = 1;
		if (p_place[0].y0 == p_place[1].y0) //横向
		{
			hanglie_flag = 1;
			for (int i = 1; i <= 7; i++)
			{
				if (i != p_place[0].y0)//本行无障碍
				{
					if (i % 2 == 1)//奇数行
					{
						lujing[lu_count].x0 = 1;
						lujing[lu_count].y0 = i;
						lu_count++;
						lujing[lu_count].x0 = 9;
						lujing[lu_count].y0 = i;
						lu_count++;
					}
					else//偶数行
					{
						lujing[lu_count].x0 = 9;
						lujing[lu_count].y0 = i;
						lu_count++;
						lujing[lu_count].x0 = 1;
						lujing[lu_count].y0 = i;
						lu_count++;
					}
				}
				else//本行有障碍
				{
					avoid_hang = i;
					if (i % 2 == 1)//奇数行
					{
						for (int i = 0; i <= 2; i++)
						{
							if (p_place[i].x0 == 1)//起点禁飞
								head_p = 1;
							if (p_place[i].x0 == 9)//起点禁飞
								tail_p = 1;
						}
						if (head_p == 1)
						{
							lujing[lu_count].av = 1;
							lujing[lu_count].x0 = 4;
							lujing[lu_count].y0 = i - 1;
							lu_count++;
							lujing[lu_count].av = 1;
							lujing[lu_count].x0 = 4;
							lujing[lu_count].y0 = i;
							lu_count++;
							lujing[lu_count].av = 1;
							lujing[lu_count].x0 = 9;
							lujing[lu_count].y0 = i;
							lu_count++;
						}
						else if (tail_p == 1)
						{
							lujing[lu_count].av = 1;
							lujing[lu_count].x0 = 1;
							lujing[lu_count].y0 = i;
							lu_count++;
							lujing[lu_count].av = 1;
							lujing[lu_count].x0 = 6;
							lujing[lu_count].y0 = i;
							lu_count++;
							lujing[lu_count].av = 1;
							lujing[lu_count].x0 = 6;
							lujing[lu_count].y0 = i + 1;
							lu_count++;
							if (p_place[0].y0 == 7)lu_count--;

						}
						else
						{
							lujing[lu_count].av = 1;
							lujing[lu_count].x0 = 1;
							lujing[lu_count].y0 = i;
							lu_count++;
							lujing[lu_count].av = 1;
							lujing[lu_count].x0 = p_place[0].x0 - 1;
							lujing[lu_count].y0 = i;
							lu_count++;
							lujing[lu_count].av = 1;
							lujing[lu_count].x0 = p_place[0].x0 - 1;
							lujing[lu_count].y0 = i - 1;
							if (p_place[0].y0 == 1)lujing[lu_count].y0 = i + 1;
							lu_count++;
							lujing[lu_count].av = 1;
							lujing[lu_count].x0 = p_place[2].x0 + 1;
							lujing[lu_count].y0 = i - 1;
							if (p_place[0].y0 == 1)lujing[lu_count].y0 = i + 1;
							lu_count++;
							lujing[lu_count].av = 1;
							lujing[lu_count].x0 = p_place[2].x0 + 1;
							lujing[lu_count].y0 = i;
							lu_count++;
							lujing[lu_count].av = 1;
							lujing[lu_count].x0 = 9;
							lujing[lu_count].y0 = i;
							lu_count++;

						}

					}
					else//偶数行
					{
						for (int i = 0; i <= 2; i++)
						{
							if (p_place[i].x0 == 9)//起点禁飞
								head_p = 1;
							if (p_place[i].x0 == 1)//起点禁飞
								tail_p = 1;
						}
						if (head_p == 1)
						{
							lujing[lu_count].av = 1;
							lujing[lu_count].x0 = 6;
							lujing[lu_count].y0 = i - 1;
							lu_count++;
							lujing[lu_count].av = 1;
							lujing[lu_count].x0 = 6;
							lujing[lu_count].y0 = i;
							lu_count++;
							lujing[lu_count].av = 1;
							lujing[lu_count].x0 = 1;
							lujing[lu_count].y0 = i;
							lu_count++;
						}
						else if (tail_p == 1)
						{
							lujing[lu_count].av = 1;
							lujing[lu_count].x0 = 9;
							lujing[lu_count].y0 = i;
							lu_count++;
							lujing[lu_count].av = 1;
							lujing[lu_count].x0 = 4;
							lujing[lu_count].y0 = i;
							lu_count++;
							lujing[lu_count].av = 1;
							lujing[lu_count].x0 = 4;
							lujing[lu_count].y0 = i + 1;
							lu_count++;

						}
						else
						{
							lujing[lu_count].av = 1;
							lujing[lu_count].x0 = 9;
							lujing[lu_count].y0 = i;
							lu_count++;
							lujing[lu_count].av = 1;
							lujing[lu_count].x0 = p_place[2].x0 + 1;
							lujing[lu_count].y0 = i;
							lu_count++;
							lujing[lu_count].av = 1;
							lujing[lu_count].x0 = p_place[2].x0 + 1;
							lujing[lu_count].y0 = i - 1;
							lu_count++;
							lujing[lu_count].av = 1;
							lujing[lu_count].x0 = p_place[0].x0 - 1;
							lujing[lu_count].y0 = i - 1;
							lu_count++;
							lujing[lu_count].av = 1;
							lujing[lu_count].x0 = p_place[0].x0 - 1;
							lujing[lu_count].y0 = i;
							lu_count++;
							lujing[lu_count].av = 1;
							lujing[lu_count].x0 = 1;
							lujing[lu_count].y0 = i;
							lu_count++;

						}
					}
				}
			}
		}
		else//纵向
		{
			hanglie_flag = 2;
			for (int i = 1; i <= 9; i++)
			{
				if (i != p_place[0].x0)//本列无障碍
				{
					if (i % 2 == 1)//奇数列
					{
						lujing[lu_count].x0 = i;
						lujing[lu_count].y0 = 1;
						lu_count++;
						lujing[lu_count].x0 = i;
						lujing[lu_count].y0 = 7;
						lu_count++;
					}
					else//偶数行
					{
						lujing[lu_count].x0 = i;
						lujing[lu_count].y0 = 7;
						lu_count++;
						lujing[lu_count].x0 = i;
						lujing[lu_count].y0 = 1;
						lu_count++;
					}
				}
				else//本列有障碍
				{
					avoid_lie = i;
					if (i % 2 == 1)//奇数列
					{
						for (int i = 0; i <= 2; i++)
						{
							if (p_place[i].y0 == 1)//起点禁飞
								head_p = 1;
							if (p_place[i].y0 == 7)//终点禁飞
								tail_p = 1;
						}
						if (head_p == 1)
						{
							lujing[lu_count].av = 1;
							lujing[lu_count].x0 = i - 1;
							lujing[lu_count].y0 = 4;
							lu_count++;
							lujing[lu_count].av = 1;
							lujing[lu_count].x0 = i;
							lujing[lu_count].y0 = 4;
							lu_count++;
							lujing[lu_count].av = 1;
							lujing[lu_count].x0 = i;
							lujing[lu_count].y0 = 7;
							lu_count++;
						}
						else if (tail_p == 1)
						{
							lujing[lu_count].av = 1;
							lujing[lu_count].x0 = i;
							lujing[lu_count].y0 = 1;
							lu_count++;
							lujing[lu_count].av = 1;
							lujing[lu_count].x0 = i;
							lujing[lu_count].y0 = 4;
							lu_count++;
							lujing[lu_count].av = 1;
							lujing[lu_count].x0 = i + 1;
							lujing[lu_count].y0 = 4;
							lu_count++;
							if (p_place[0].x0 == 9)lu_count--;

						}
						else
						{
							lujing[lu_count].av = 1;
							lujing[lu_count].x0 = i;
							lujing[lu_count].y0 = 1;
							lu_count++;
							lujing[lu_count].av = 1;
							lujing[lu_count].x0 = i;
							lujing[lu_count].y0 = p_place[0].y0 - 1;
							lu_count++;
							lujing[lu_count].av = 1;
							lujing[lu_count].x0 = i - 1;
							lujing[lu_count].y0 = p_place[0].y0 - 1;
							if (p_place[0].x0 == 1)lujing[lu_count].x0 = i + 1;
							lu_count++;
							lujing[lu_count].av = 1;
							lujing[lu_count].x0 = i - 1;
							lujing[lu_count].y0 = p_place[2].y0 + 1;
							if (p_place[0].x0 == 1)lujing[lu_count].x0 = i + 1;
							lu_count++;
							lujing[lu_count].av = 1;
							lujing[lu_count].x0 = i;
							lujing[lu_count].y0 = p_place[2].y0 + 1;
							lu_count++;
							lujing[lu_count].av = 1;
							lujing[lu_count].x0 = i;
							lujing[lu_count].y0 = 7;
							lu_count++;

						}

					}
					else//偶数列
					{
						for (int i = 0; i <= 2; i++)
						{
							if (p_place[i].y0 == 7)//起点禁飞
								head_p = 1;
							if (p_place[i].y0 == 1)//起点禁飞
								tail_p = 1;
						}
						if (head_p == 1)
						{
							lujing[lu_count].av = 1;
							lujing[lu_count].x0 = i - 1;
							lujing[lu_count].y0 = 4;
							lu_count++;
							lujing[lu_count].av = 1;
							lujing[lu_count].x0 = i;
							lujing[lu_count].y0 = 4;
							lu_count++;
							lujing[lu_count].av = 1;
							lujing[lu_count].x0 = i;
							lujing[lu_count].y0 = 1;
							lu_count++;
						}
						else if (tail_p == 1)
						{
							lujing[lu_count].av = 1;
							lujing[lu_count].x0 = i;
							lujing[lu_count].y0 = 7;
							lu_count++;
							lujing[lu_count].av = 1;
							lujing[lu_count].x0 = i;
							lujing[lu_count].y0 = 4;
							lu_count++;
							lujing[lu_count].av = 1;
							lujing[lu_count].x0 = i + 1;
							lujing[lu_count].y0 = 4;
							lu_count++;

						}
						else
						{
							lujing[lu_count].av = 1;
							lujing[lu_count].x0 = i;
							lujing[lu_count].y0 = 7;
							lu_count++;
							lujing[lu_count].av = 1;
							lujing[lu_count].x0 = i;
							lujing[lu_count].y0 = p_place[2].y0 + 1;
							lu_count++;
							lujing[lu_count].av = 1;
							lujing[lu_count].x0 = i - 1;
							lujing[lu_count].y0 = p_place[2].y0 + 1;
							lu_count++;
							lujing[lu_count].av = 1;
							lujing[lu_count].x0 = i - 1;
							lujing[lu_count].y0 = p_place[0].y0 - 1;
							lu_count++;
							lujing[lu_count].av = 1;
							lujing[lu_count].x0 = i;
							lujing[lu_count].y0 = p_place[0].y0 - 1;
							lu_count++;
							lujing[lu_count].av = 1;
							lujing[lu_count].x0 = i;
							lujing[lu_count].y0 = 1;
							lu_count++;

						}
					}
				}
			}
		}

		Lu_Cnd = lu_count;

		for (int i = 0; i < Lu_Cnd; i++)
		{
			Targ_Pos[0][i] = -(lujing[i].x0 - 1) * 50;
		}
		for (int i = 0; i < Lu_Cnd; i++)
		{
			Targ_Pos[1][i] = (lujing[i].y0 - 1) * 50;
		}
		for (int i = 1; i <= Lu_Cnd; i++)
		{
			Targ_Pos[2][i] = lujing[i].av;
		}

		if ((Targ_Pos[0][0] == 0) && (Targ_Pos[1][0] == 0))		Next_Tar_Flag = 1;
		else		Next_Tar_Flag = 0;

		if ((Avoid_X[0] < 7) && (Avoid_X[2] < 7) && (Avoid_Y[0] > 0) && (Avoid_Y[2] > 0) && Avoid_Y[1] != 1)
			BackRoad_Flag = 1;
		else if ((Avoid_X[0] == Avoid_X[2]) && (Avoid_Y[1] == 6))
			BackRoad_Flag = 1;
		else if ((Avoid_Y[0] == Avoid_Y[2]) && (Avoid_Y[1] == 7))
			BackRoad_Flag = 1;
		else 
			BackRoad_Flag = 0;

		if (Been_Suanshu == 0)
		{
			Start_Way = 1;
			LASER1_flag = 1;
			if ((SDK_Ctrl_Mode_LY == 1) && (Throttle_Control > 1300) && (Throttle_Control < 1650))
				AUTO_FLY_FLAG = 1;
		}
	}
}
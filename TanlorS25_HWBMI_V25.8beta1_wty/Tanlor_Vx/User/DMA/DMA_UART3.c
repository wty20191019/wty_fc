#include "Headfile.h"
#include "DMA_UART3.h"



u8 DMA_USART3_RxBuffer[300];
int DMA_USART3_Length = 0;
u8  DMA_USART3_DATA_SUM = 0;  //数据校验和


void DMA_USART3_Init(unsigned long bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;//定义NVIC初始化结构体
	DMA_InitTypeDef  DMA_InitStructure;

	RCC_AHB1PeriphClockCmd(DMA_USART3_IO_CLOCK, ENABLE);
	RCC_APB1PeriphClockCmd(DMA_USART3_CLOCK, ENABLE);

	GPIO_PinAFConfig(DMA_USART3_IO_PORT, DMA_USART3_TX_PinSource, DMA_USART3_GPIO_AF);
	GPIO_PinAFConfig(DMA_USART3_IO_PORT, DMA_USART3_RX_PinSource, DMA_USART3_GPIO_AF);

	GPIO_InitStructure.GPIO_Pin = DMA_USART3_TX | DMA_USART3_RX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(DMA_USART3_IO_PORT, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(DMA_USART3, &USART_InitStructure); //初始化串口

	USART_ClearFlag(DMA_USART3, USART_FLAG_TC);//清除中断标志位
	USART_ITConfig(DMA_USART3, USART_IT_IDLE, ENABLE);//开启空闲中断，串口接收数据用到了空闲中断，因此必须开启！

	USART_DMACmd(DMA_USART3, USART_DMAReq_Rx, ENABLE);  //使能串口1的DMA接收,这两句与DMA传输相关，一定要使能！！！！！！！！！！！
	USART_DMACmd(DMA_USART3, USART_DMAReq_Tx, ENABLE);  //使能串口1的DMA发送，这两句与DMA传输相关，一定要使能！！！！！！！！！！！


	//DMAx的各通道配置,外设到存储器
	//这里的传输形式是固定的,这点要根据不同的情况来修改
	//从存储器->外设模式/8位数据宽度/存储器增量模式
	//DMA_Streamx:DMA数据流,DMA1_Stream0~7/DMA2_Stream0~7
	//chx:DMA通道选择,@ref DMA_channel DMA_Channel_0~DMA_Channel_7
	//par:外设地址
	//mar:存储器地址
	//ndtr:数据传输量  


	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);//DMA1时钟使能 
	/* 配置 DMA Stream */
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //通道选择
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(DMA_USART3->DR));//DMA外设地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)DMA_USART3_RxBuffer;//DMA 存储器0地址
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
	DMA_Init(DMA_USART3_DMA_Stream, &DMA_InitStructure);//初始化DMA Stream
	DMA_Cmd(DMA_USART3_DMA_Stream, ENABLE);             //开启DMA传输,只是与前面配置不同的，注意！


	USART_Cmd(DMA_USART3, ENABLE);                    //使能串口

}



void DMA_USART3_Send_Char(unsigned char tx_buf)
{
	USART_SendData(DMA_USART3, tx_buf);
	while (USART_GetFlagStatus(DMA_USART3, USART_FLAG_TC) != 1);
}

void DMA_USART3_Send_Buff(unsigned char *tx_buf, int Length)
{
	uint16_t i;
	for (i = 0; i < Length; i++)		//遍历数组
	{
		DMA_USART3_Send_Char(tx_buf[i]);		//依次调用BusServo_SendByte发送每个字节数据
	}
}




void USART3_IRQHandler(void)                	//串口1中断服务程序
{
	if (USART_GetITStatus(DMA_USART3, USART_IT_IDLE) != RESET)//接收到一条完整数据进入空闲中断
	{
		USART_ClearITPendingBit(DMA_USART3, USART_IT_IDLE);//清除中断标志
		DMA_USART3->SR;  		DMA_USART3->DR;       //清除中断标志

		DMA_Cmd(DMA_USART3_DMA_Stream, DISABLE);                           //关闭DMA传输,此处一定要关闭，不然后面判断过不了！！！
		DMA_USART3_Length = 256 - DMA_GetCurrDataCounter(DMA_USART3_DMA_Stream);    //得到当前传输数据量
		DMA_USART3_DMA_Stream->NDTR = 256;

		DMA_Cmd(DMA_USART3_DMA_Stream, ENABLE);                            //开启DMA传输！！！！！！！！！！！
	}
}

extern float T265_Data_float;
extern char *pdata;
extern float T265_Data_orgial[10];
extern float  T265_data_Yaw;
extern u8 T265_Confidence;
extern void Radar_StateMachine(void);
u8 Radar_New = 0;
u8 Radar_New_CND = 0;
u8 Radar_New_CND_LST = 0;
extern uint8_t kp_decay_cnt;
extern float radar_error_kp;
typedef struct
{
	float qx;
	float qy;
	float qz;
	float qw;
}Quaternion;
Quaternion radar;
u8 Start_Beep = 0;

void DMA_USART3_Data_Prase()
{
	if ((DMA_USART3_RxBuffer[0] == 0xff && DMA_USART3_RxBuffer[1] == 0xfe
		&& DMA_USART3_RxBuffer[2] == 0xfd && DMA_USART3_RxBuffer[3] == 0xfc
		&& DMA_USART3_RxBuffer[4] == 0xfb && DMA_USART3_RxBuffer[5] == 0xfa))
	{

		if (Start_Beep == 0)
		{
			Start_Beep = 1;
			BEEP_flag = 1;
		}
		ps_data.Is_normal = 1;   //T265接收数据正常

		for (u8 i = 0; i < 7; i++)
		{
			pdata = (char *)&T265_Data_float;
			*pdata++ = DMA_USART3_RxBuffer[6 + i * 4];	*pdata++ = DMA_USART3_RxBuffer[7 + i * 4];	*pdata++ = DMA_USART3_RxBuffer[8 + i * 4];	*pdata++ = DMA_USART3_RxBuffer[9 + i * 4];
			T265_Data_orgial[i] = T265_Data_float;
		}

		ps_data.translation.y = T265_Data_orgial[0] * 100; //  cm 机头向前，往右y增加
		ps_data.translation.x = -T265_Data_orgial[1] * 100; //  cm 机头向前，往前x增加

		T265_data_Yaw = T265_Data_orgial[6] * RAD2DEG;

		/****************T265原始坐标系转化为无人机机体坐标系*********************/
		ps_data.velocity.y = (T265_Data_orgial[2] * 100) *cos(T265_data_Yaw*DEG2RAD) + (-T265_Data_orgial[5] * 100)*sin(T265_data_Yaw*DEG2RAD);  //  cm/s 机头向前，往右y增加
		ps_data.velocity.x = (-T265_Data_orgial[3] * 100)*cos(T265_data_Yaw*DEG2RAD) - (T265_Data_orgial[3] * 100)*sin(T265_data_Yaw*DEG2RAD);   //  cm/s 机头向前，往前x增加	

		ld_data.velocity.y = (T265_Data_orgial[4] * 100) *cos(T265_data_Yaw*DEG2RAD) + (-T265_Data_orgial[5] * 100)*sin(T265_data_Yaw*DEG2RAD);  //  cm/s 机头向前，往右y增加
		ld_data.velocity.x = (-T265_Data_orgial[5] * 100)*cos(T265_data_Yaw*DEG2RAD) - (T265_Data_orgial[3] * 100)*sin(T265_data_Yaw*DEG2RAD);   //  cm/s 机头向前，往前x增加	

		T265_Confidence = DMA_USART3_RxBuffer[34];

		Radar_New_CND_LST = Radar_New_CND;
		Radar_New_CND = DMA_USART3_RxBuffer[35];
		if (Radar_New_CND != Radar_New_CND_LST)		Radar_New = 1;
		else		Radar_New = 0;

		/***************** 7通道*************************/
		if (PPM_Databuf[6] >= (RC_Calibration[6].max - RC_Calibration[6].deadband))
		{
			T265_Confidence = 2;
		}

	}
}




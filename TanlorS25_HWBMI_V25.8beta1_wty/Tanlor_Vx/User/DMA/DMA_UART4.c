#include "Headfile.h"
#include "DMA_UART4.h"


u8 DMA_USART6_RxBuffer[300];
u8 DMA_USART4_RxBuffer[300];
int DMA_USART4_Length = 0;
u8  DMA_USART4_DATA_SUM = 0;  //数据校验和
int DMA_USART6_Length = 0;
u8  DMA_USART6_DATA_SUM = 0;  //数据校验和

void DMA_USART4_Init(unsigned long bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;//定义NVIC初始化结构体
	DMA_InitTypeDef  DMA_InitStructure;

	RCC_AHB1PeriphClockCmd(DMA_USART4_IO_CLOCK, ENABLE);
	RCC_APB1PeriphClockCmd(DMA_USART4_CLOCK, ENABLE);

	GPIO_PinAFConfig(DMA_USART4_IO_PORT, DMA_USART4_TX_PinSource, DMA_USART4_GPIO_AF);
	GPIO_PinAFConfig(DMA_USART4_IO_PORT, DMA_USART4_RX_PinSource, DMA_USART4_GPIO_AF);

	GPIO_InitStructure.GPIO_Pin = DMA_USART4_TX | DMA_USART4_RX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(DMA_USART4_IO_PORT, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(DMA_USART4, &USART_InitStructure); //初始化串口

	USART_ClearFlag(DMA_USART4, USART_FLAG_TC);//清除中断标志位
	USART_ITConfig(DMA_USART4, USART_IT_IDLE, ENABLE);//开启空闲中断，串口接收数据用到了空闲中断，因此必须开启！

	USART_DMACmd(DMA_USART4, USART_DMAReq_Rx, ENABLE);  //使能串口1的DMA接收,这两句与DMA传输相关，一定要使能！！！！！！！！！！！
	USART_DMACmd(DMA_USART4, USART_DMAReq_Tx, ENABLE);  //使能串口1的DMA发送，这两句与DMA传输相关，一定要使能！！！！！！！！！！！


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
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(DMA_USART4->DR));//DMA外设地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)DMA_USART4_RxBuffer;//DMA 存储器0地址
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
	//DMA1的数据流2
	DMA_Init(DMA_USART4_DMA_Stream, &DMA_InitStructure);//初始化DMA Stream
	DMA_Cmd(DMA_USART4_DMA_Stream, ENABLE);             //开启DMA传输,只是与前面配置不同的，注意！


	USART_Cmd(DMA_USART4, ENABLE);                    //使能串口

}



void DMA_USART4_Send_Char(unsigned char tx_buf)
{
	USART_SendData(DMA_USART4, tx_buf);
	while (USART_GetFlagStatus(DMA_USART4, USART_FLAG_TC) != 1);
}

void DMA_USART4_Send_Buff(unsigned char *tx_buf, int Length)
{
	uint16_t i;
	for (i = 0; i < Length; i++)		//遍历数组
	{
		DMA_USART4_Send_Char(tx_buf[i]);		//依次调用BusServo_SendByte发送每个字节数据
	}
}



void UART4_IRQHandler(void)                	//串口1中断服务程序
{
	if (USART_GetITStatus(DMA_USART4, USART_IT_IDLE) != RESET)//接收到一条完整数据进入空闲中断
	{
		USART_ClearITPendingBit(DMA_USART4, USART_IT_IDLE);//清除中断标志
		DMA_USART4->SR;  		DMA_USART4->DR;       //清除中断标志

		DMA_Cmd(DMA_USART4_DMA_Stream, DISABLE);                           //关闭DMA传输,此处一定要关闭，不然后面判断过不了！！！
		DMA_USART4_Length = 256 - DMA_GetCurrDataCounter(DMA_USART4_DMA_Stream);    //得到当前传输数据量
		DMA_USART4_DMA_Stream->NDTR = 256;

		DMA_Cmd(DMA_USART4_DMA_Stream, ENABLE);                            //开启DMA传输！！！！！！！！！！！
	}
}


OpticalFlowData optical_flow;
u8 USART6_flag = 0, head_USART6 = 0;
int16_t flow_x_integral = 0;
int16_t flow_y_integral = 0;
uint16_t integration_time = 0;
uint16_t distance_mm = 0;

void DMA_USART4_Data_Prase()
{
	// 完整数据包为14字节 (0xFE头 + 0x0A长度 + 10数据字节 + XOR校验 + 0x55尾)
	if ((DMA_USART4_RxBuffer[0] == 0xFE && DMA_USART4_RxBuffer[1] == 0x0A))
	{
		// 检查包尾和校验
		if (DMA_USART4_RxBuffer[13] == 0x55)
		{
			// 解析数据
			flow_x_integral = (DMA_USART4_RxBuffer[2] | (DMA_USART4_RxBuffer[3] << 8));
			flow_y_integral = (DMA_USART4_RxBuffer[4] | (DMA_USART4_RxBuffer[5] << 8));
			integration_time = (DMA_USART4_RxBuffer[6] | (DMA_USART4_RxBuffer[7] << 8));
			distance_mm = (DMA_USART4_RxBuffer[8] | (DMA_USART4_RxBuffer[9] << 8));

			// 转换为物理量
			optical_flow.flow_x = (flow_x_integral / 10000.0f) / (integration_time / 1000000.0f);
			optical_flow.flow_y = (flow_y_integral / 10000.0f) / (integration_time / 1000000.0f);
			optical_flow.distance = distance_mm / 1000.0f;
			optical_flow.valid = DMA_USART4_RxBuffer[10];
			optical_flow.confidence = DMA_USART4_RxBuffer[11];
		}
	}
}


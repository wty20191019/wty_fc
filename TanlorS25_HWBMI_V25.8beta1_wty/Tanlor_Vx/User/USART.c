#include "Headfile.h"
#include "usart.h"
#include "stdio.h"




u16 DMA1_MEM_LEN;															//保存DMA每次数据传送的长度
RingBuff_t Ground_Station_Ringbuf;

void DMA_Use_USART1_Tx_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	/* 1.使能DMA2时钟 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	/* 2.配置使用DMA发送数据 */
	DMA_DeInit(DMA2_Stream7);

	DMA_InitStructure.DMA_Channel = DMA_Channel_4;               /* 配置DMA通道 */
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(USART1->DR));   /* 目的 */
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)TransmitBuffer.Buffer;             /* 源 */
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;    /* 方向 */
	DMA_InitStructure.DMA_BufferSize = TRANSMIT_BUF_MAX_SIZE;                    /* 长度 */
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;    /* 外设地址是否自增 */
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;         /* 内存地址是否自增 */
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;      /* 目的数据带宽 */
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;      /* 源数据宽度 */
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;              /* 单次传输模式/循环传输模式 */
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;             /* DMA优先级 */
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;          /* FIFO模式/直接模式 */
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull; /* FIFO大小 */
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;       /* 单次传输 */
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	/* 3. 配置DMA */
	DMA_Init(DMA2_Stream7, &DMA_InitStructure);

	/* 4.使能DMA中断 */
	DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);


	/* 5.使能串口的DMA发送接口 */
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);

	/* 7.不使能DMA */
	DMA_Cmd(DMA2_Stream7, DISABLE);
}

void DMA2_Stream7_IRQHandler(void)
{
	//清除标志
	if (DMA_GetFlagStatus(DMA2_Stream7, DMA_FLAG_TCIF7) != RESET)//等待DMA2_Steam7传输完成  
	{
		DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7 | DMA_FLAG_FEIF7 | DMA_FLAG_DMEIF7 | DMA_FLAG_TEIF7 | DMA_FLAG_HTIF7);//清除DMA2_Steam7传输完成标志
		DMA_Cmd(DMA2_Stream7, DISABLE);
		DMA_SetCurrDataCounter(DMA2_Stream7, 0);
	}
}

void DMA_Send(uint8_t* data, uint32_t writeLen)
{
	// 等待DMA可配置
	while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE) {}

	memcpy(TransmitBuffer.Buffer, data, writeLen);
	DMA_SetCurrDataCounter(DMA2_Stream7, writeLen);
	DMA_Cmd(DMA2_Stream7, ENABLE);
}


/***************************************************
函数名: void USART1_Init(unsigned long bound)
说明:	串口1初始化
入口:	波特率
出口:	无
备注:	上电初始化，运行一次
****************************************************/
void USART1_Init(unsigned long bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);//使能USART1时钟

														  //串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1); //GPIOA10复用为USART1

															   //USART1端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure); //初始化PA9，PA10

										   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART1, &USART_InitStructure); //初始化串口1


	USART_Cmd(USART1, ENABLE);  //使能串口1 
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断
}


//u8 ldca = 0;
//void USART1_IRQHandler(void)
//{
//	if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET)
//	{
//		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
//		RingBuf_Write(USART_ReceiveData(USART1), &Ground_Station_Ringbuf, 190);//往环形队列里面写数据
//	}
//}
//

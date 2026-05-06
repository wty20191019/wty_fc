#include "Headfile.h"
#include "DMA_UART2.h"



u8 DMA_USART2_RxBuffer[300];
int DMA_USART2_Length = 0;
u8  DMA_USART2_DATA_SUM = 0;  //数据校验和

void DMA_USART2_Init(unsigned long bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;//定义NVIC初始化结构体
	DMA_InitTypeDef  DMA_InitStructure;

	RCC_AHB1PeriphClockCmd(DMA_USART2_IO_CLOCK, ENABLE);
	RCC_APB1PeriphClockCmd(DMA_USART2_CLOCK, ENABLE);

	GPIO_PinAFConfig(DMA_USART2_IO_PORT, DMA_USART2_TX_PinSource, DMA_USART2_GPIO_AF);
	GPIO_PinAFConfig(DMA_USART2_IO_PORT, DMA_USART2_RX_PinSource, DMA_USART2_GPIO_AF);

	GPIO_InitStructure.GPIO_Pin = DMA_USART2_TX | DMA_USART2_RX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(DMA_USART2_IO_PORT, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(DMA_USART2, &USART_InitStructure); //初始化串口

	USART_ClearFlag(DMA_USART2, USART_FLAG_TC);//清除中断标志位
	USART_ITConfig(DMA_USART2, USART_IT_IDLE, ENABLE);//开启空闲中断，串口接收数据用到了空闲中断，因此必须开启！

	USART_DMACmd(DMA_USART2, USART_DMAReq_Rx, ENABLE);  //使能串口1的DMA接收,这两句与DMA传输相关，一定要使能！！！！！！！！！！！
	USART_DMACmd(DMA_USART2, USART_DMAReq_Tx, ENABLE);  //使能串口1的DMA发送，这两句与DMA传输相关，一定要使能！！！！！！！！！！！


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
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(DMA_USART2->DR));//DMA外设地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)DMA_USART2_RxBuffer;//DMA 存储器0地址
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
	DMA_Init(DMA_USART2_DMA_Stream, &DMA_InitStructure);//初始化DMA Stream
	DMA_Cmd(DMA_USART2_DMA_Stream, ENABLE);             //开启DMA传输,只是与前面配置不同的，注意！


	USART_Cmd(DMA_USART2, ENABLE);                    //使能串口

}

void DMA_USART2_Send_Char(unsigned char tx_buf)
{
	USART_SendData(DMA_USART2, tx_buf);
	while (USART_GetFlagStatus(DMA_USART2, USART_FLAG_TC) != 1);
}

void DMA_USART2_Send_Buff(unsigned char *tx_buf, int Length)
{
	uint16_t i;
	for (i = 0; i < Length; i++)		//遍历数组
	{
		DMA_USART2_Send_Char(tx_buf[i]);		//依次调用BusServo_SendByte发送每个字节数据
	}
}



void USART2_IRQHandler(void)                	//串口1中断服务程序
{
	if (USART_GetITStatus(DMA_USART2, USART_IT_IDLE) != RESET)//接收到一条完整数据进入空闲中断
	{
		USART_ClearITPendingBit(DMA_USART2, USART_IT_IDLE);//清除中断标志
		DMA_USART2->SR;  		DMA_USART2->DR;       //清除中断标志

		DMA_Cmd(DMA_USART2_DMA_Stream, DISABLE);                           //关闭DMA传输,此处一定要关闭，不然后面判断过不了！！！
		DMA_USART2_Length = 256 - DMA_GetCurrDataCounter(DMA_USART2_DMA_Stream);    //得到当前传输数据量
		DMA_USART2_DMA_Stream->NDTR = 256;

		DMA_Cmd(DMA_USART2_DMA_Stream, ENABLE);                            //开启DMA传输！！！！！！！！！！！
	}
}

u8 Anml_Num = 0;
u8 Fst_Anml = 0;
float Fst_Anml_X = 0;
float Fst_Anml_Y = 0;
u8 Sec_Anml = 0;
float Sec_Anml_X = 0;
float Sec_Anml_Y = 0;
u8 Thr_Anml = 0;
float Thr_Anml_X = 0;
float Thr_Anml_Y = 0;
extern u8 Anm_Flag;
u16 Pre_Anm_StX = 0;
float Rec_Anm_Pos_X[10] = { 0 };
float Rec_Anm_Pos_Y[10] = { 0 };
u8 Rec_Anm_Gx[10] = { 0 };
u8 Rec_Anm_Gy[10] = { 0 };
u8 Rec_Num = 0;
extern void Where_RU(float pos_x, float pos_y, u8 i);

float BianX = 0;
float BianY = 0;

#define  BF_MAX 75
u8 An_bf[BF_MAX];
u8 ct_start_flag = 0;
u8 at_count[20] = { 0 };
float X_bf[BF_MAX] = { 0 };
float Y_bf[BF_MAX] = { 0 };
u8 leisi_rec_buff[BF_MAX] = {0};
u8 bf_pt = 0;
u8 An_bf_C[10];
float X_bf_C[10] = { 0 };
float Y_bf_C[10] = { 0 };
u8 bf_C_Pt = 0;

u8 leisi(float X0, float Y0, float X, float Y)
{
	if( ((X0-X)* (X0 - X) + (Y0 - Y)* (Y0 - Y)) <15*15) 	return 1;
	return 0;
}

int Fst_Anml_bf[11] = { 0 };
int Fst_Anml_X_bf[11] = { 0 };
int Fst_Anml_Y_bf[11] = { 0 };
int Sec_Anml_bf[11] = { 0 };
int Sec_Anml_X_bf[11] = { 0 };
int Sec_Anml_Y_bf[11] = { 0 };
int Thr_Anml_bf[11] = { 0 };
int Thr_Anml_X_bf[11] = { 0 };
int Thr_Anml_Y_bf[11] = { 0 };
u8 Get_Camera_Flag = 1;
extern int find_mode(int arr[], int size);
int Anml_Num_bf[11] = { 0 };
u8 caa = 0;
extern void Let_s_Try(void);
float X_Add = 0;
float Y_Add = 0;
float addK = 95.0f;

void DMA_USART2_Data_Prase()
{

	u8 ccc = 0;
	u8 cdd = 0;
	u8 leisi_c = 0;
	if ((DMA_USART2_RxBuffer[0] == 0xFF && DMA_USART2_RxBuffer[1] == 0xFA))
	{
		DMA_USART2_RxBuffer[1] = 0xFC;
		Anml_Num = 0;

		X_Add = -addK * (-PI* 1.2 * MRoll * DEG2RAD / 3.0f);
		Y_Add = -addK * (PI* 1.2 * MPitch * DEG2RAD / 3.0f);

		if ((DMA_USART2_RxBuffer[4] != 0) || (DMA_USART2_RxBuffer[5] != 0))
		{
			Fst_Anml = DMA_USART2_RxBuffer[3];
			An_bf[1] = Fst_Anml;
			Fst_Anml_X = ((DMA_USART2_RxBuffer[4] - 80) - X_Add) / 160.0f*100.0f;
			Fst_Anml_Y = ((DMA_USART2_RxBuffer[5] - 80) - Y_Add) / 160.0f*100.0f;
			X_bf[1] = Fst_Anml_X + PosSenser_SINS.Location.x;
			Y_bf[1] = Fst_Anml_Y + PosSenser_SINS.Location.y;
		}
		else
		{
			Fst_Anml = 0;
			An_bf[1] = 0;
			Fst_Anml_X = 0;
			Fst_Anml_Y = 0;
			X_bf[1] = 0;
			Y_bf[1] = 0;
		}

		if ((DMA_USART2_RxBuffer[7] != 0) || (DMA_USART2_RxBuffer[8] != 0))
		{
			Sec_Anml = DMA_USART2_RxBuffer[6];
			An_bf[2] = Sec_Anml;
			Sec_Anml_X = ((DMA_USART2_RxBuffer[7] - 80) - X_Add) / 160.0f*100.0f;
			Sec_Anml_Y = ((DMA_USART2_RxBuffer[8] - 80) - Y_Add) / 160.0f*100.0f;
			X_bf[2] = Sec_Anml_X + PosSenser_SINS.Location.x;
			Y_bf[2] = Sec_Anml_Y + PosSenser_SINS.Location.y;
		}
		else
		{
			Sec_Anml = 0;
			An_bf[2] = 0;
			Sec_Anml_X = 0;
			Sec_Anml_Y = 0;
			An_bf[2] = 0;
			X_bf[2] = 0;
		}


		Thr_Anml = 0;
		An_bf[3] = 0;
		Thr_Anml_X = 0;
		Thr_Anml_Y = 0;
		X_bf[3] = 0;
		Y_bf[3] = 0;
		
		if (Fst_Anml)		Anml_Num++;
		if (Sec_Anml)		Anml_Num++;

		//Anml_Num_bf[caa] = Anml_Num;
		//Fst_Anml_bf[caa] = Fst_Anml;
		//Fst_Anml_X_bf[caa] = (int)Fst_Anml_X;
		//Fst_Anml_Y_bf[caa] = (int)Fst_Anml_Y;
		//Sec_Anml_bf[caa] = Sec_Anml;
		//Sec_Anml_X_bf[caa] = (int)Sec_Anml_X;
		//Sec_Anml_Y_bf[caa] = (int)Sec_Anml_Y;
		//Thr_Anml_bf[caa] = Thr_Anml;
		//Thr_Anml_X_bf[caa] = (int)Thr_Anml_X;
		//Thr_Anml_Y_bf[caa] = (int)Thr_Anml_Y;

		//caa++;
		//if (caa >= 5)
		//	caa = 0;

		if (Get_Camera_Flag)
		{
			Get_Camera_Flag = 0;

			//Anml_Num = find_mode(Anml_Num_bf, 5);
			//Fst_Anml = find_mode(Fst_Anml_bf, 5);
			//Sec_Anml = find_mode(Sec_Anml_bf, 5);
			//Thr_Anml = find_mode(Thr_Anml_bf, 5);

			//Fst_Anml_X = find_mode(Fst_Anml_X_bf, 5);
			//Fst_Anml_Y = find_mode(Fst_Anml_Y_bf, 5);
			//Sec_Anml_X = find_mode(Sec_Anml_X_bf, 5);
			//Sec_Anml_Y = find_mode(Sec_Anml_Y_bf, 5);
			//Thr_Anml_X = find_mode(Thr_Anml_X_bf, 5);
			//Thr_Anml_Y = find_mode(Thr_Anml_Y_bf, 5);

			memset(Rec_Anm_Gx, 0, sizeof(Rec_Anm_Gx));
			memset(Rec_Anm_Gy, 0, sizeof(Rec_Anm_Gy));

			Where_RU(X_bf[1], Y_bf[1], 1);
			Where_RU(X_bf[2], Y_bf[2], 2);
			//Where_RU(X_bf[3], Y_bf[3], 3);

			//memset(Anml_Num_bf, 0, sizeof(Anml_Num_bf));
			//memset(Fst_Anml_bf, 0, sizeof(Fst_Anml_bf));
			//memset(Sec_Anml_bf, 0, sizeof(Sec_Anml_bf));
			//memset(Thr_Anml_bf, 0, sizeof(Thr_Anml_bf));

			//memset(Fst_Anml_X_bf, 0, sizeof(Fst_Anml_X_bf));
			//memset(Fst_Anml_Y_bf, 0, sizeof(Fst_Anml_Y_bf));
			//memset(Sec_Anml_X_bf, 0, sizeof(Sec_Anml_X_bf));
			//memset(Sec_Anml_Y_bf, 0, sizeof(Sec_Anml_Y_bf));
			//memset(Thr_Anml_X_bf, 0, sizeof(Thr_Anml_X_bf));
			//memset(Thr_Anml_Y_bf, 0, sizeof(Thr_Anml_Y_bf));

			Let_s_Try();
		}
	}
	else
	{
		Anml_Num = 0;
	}

}

extern Vector3f SDK_Zero_POSITION;
extern float Cnter_Gz[3];

void Where_RU(float pos_x, float pos_y, u8 i)
{

	if (pos_y <= 25 + SDK_Zero_POSITION.y)				Rec_Anm_Gy[i] = 1;
	else if (pos_y <= 75 + SDK_Zero_POSITION.y)			Rec_Anm_Gy[i] = 2;
	else if (pos_y <= 125 + SDK_Zero_POSITION.y)		Rec_Anm_Gy[i] = 3;
	else if (pos_y <= 175 + SDK_Zero_POSITION.y)		Rec_Anm_Gy[i] = 4;
	else if (pos_y <= 225 + SDK_Zero_POSITION.y)		Rec_Anm_Gy[i] = 5;
	else if (pos_y <= 275 + SDK_Zero_POSITION.y)		Rec_Anm_Gy[i] = 6;
	else												Rec_Anm_Gy[i] = 7;

	if (pos_x >= -25 + SDK_Zero_POSITION.x)				Rec_Anm_Gx[i] = 1;
	else if (pos_x >= -75 + SDK_Zero_POSITION.x)		Rec_Anm_Gx[i] = 2;
	else if (pos_x >= -125 + SDK_Zero_POSITION.x)		Rec_Anm_Gx[i] = 3;
	else if (pos_x >= -175 + SDK_Zero_POSITION.x)		Rec_Anm_Gx[i] = 4;
	else if (pos_x >= -225 + SDK_Zero_POSITION.x)		Rec_Anm_Gx[i] = 5;
	else if (pos_x >= -275 + SDK_Zero_POSITION.x)		Rec_Anm_Gx[i] = 6;
	else if (pos_x >= -325 + SDK_Zero_POSITION.x)		Rec_Anm_Gx[i] = 7;
	else if (pos_x >= -375 + SDK_Zero_POSITION.x)		Rec_Anm_Gx[i] = 8;
	else 												Rec_Anm_Gx[i] = 9;
}


void Anti_Where_RU_X(u8 gez_x, u8 gez_y)
{
	if (gez_x == 1)				Cnter_Gz[1] = 0;
	else if (gez_x == 2)		Cnter_Gz[1] = -50;
	else if (gez_x == 3)		Cnter_Gz[1] = -100;
	else if (gez_x == 4)		Cnter_Gz[1] = -150;
	else if (gez_x == 5)		Cnter_Gz[1] = -200;
	else if (gez_x == 6)		Cnter_Gz[1] = -250;
	else if (gez_x == 7)		Cnter_Gz[1] = -300;
	else if (gez_x == 8)		Cnter_Gz[1] = -350;
	else						Cnter_Gz[1] = -400;

	if (gez_y == 1)				Cnter_Gz[2] = 0;
	else if (gez_y == 2)		Cnter_Gz[2] = 50;
	else if (gez_y == 3)		Cnter_Gz[2] = 100;
	else if (gez_y == 4)		Cnter_Gz[2] = 150;
	else if (gez_y == 5)		Cnter_Gz[2] = 200;
	else if (gez_y == 6)		Cnter_Gz[2] = 250;
	else						Cnter_Gz[2] = 300;
}


int find_mode(int arr[], int size) 
{
	int max_count = 0, mode = arr[0];

	for (int i = 0; i < size; i++) {
		if (arr[i] == 0) continue;  // 跳过无效值（0）

		int count = 0;
		for (int j = 0; j < size; j++) {
			if (arr[j] == arr[i]) count++;
		}

		if (count > max_count) {
			max_count = count;
			mode = arr[i];
		}
	}
	return mode;
}



//for (caa = 0; caa < 75; caa++)
//{
//	if (An_bf[caa] < 6 && An_bf[caa]>0)
//	{
//		if(An_bf[caa]==1)		at_count[1]++;
//		if (An_bf[caa] == 2)		at_count[2]++;
//		if (An_bf[caa] == 3)		at_count[3]++;
//		if (An_bf[caa] == 4)		at_count[4]++;
//		if (An_bf[caa] == 5)		at_count[5]++;

//	}
//		 
//}

//for (caa = 1; caa < 6; caa++)
//{
//	if (at_count[caa] > 15)
//	{
//		ct_start_flag = 1;

//	}
//}

//if (ct_start_flag == 1)
//{
//	caa = bf_pt;	ccc = 0;
//	
//	while (ccc < 75)
//	{
//		cbb = bf_pt;	cdd = 0;
//		while (cdd < 75)
//		{
//			if (leisi(X_bf[caa], Y_bf[caa], X_bf[cbb], Y_bf[cbb]))
//			{
//				leisi_rec_buff[leisi_c] = cbb; //记录类似的点的序号
//				leisi_c++;	
//			}
//			if (cbb == 0) cbb = 75;    cbb--; 
//			cdd++;
//		}

//		//找到相似点
//		if (leisi_c > 15)
//		{
//			An_bf_C[bf_C_Pt] = An_bf[caa];	X_bf_C[bf_C_Pt] = X_bf[caa];	Y_bf_C[bf_C_Pt] = Y_bf[caa];
//			bf_C_Pt++;
//			if (bf_C_Pt > 9) bf_C_Pt = 0;

//			//清除类似点
//			for (cdd = 0; cdd < leisi_c; cdd++)
//			{
//				An_bf[leisi_rec_buff[cdd]] = 0;
//				X_bf[leisi_rec_buff[cdd]] = 0;
//				Y_bf[leisi_rec_buff[cdd]] = 0;
//			}
//		}
//		else {	leisi_c = 0;}
//		cdd=0;
//		ccc++;
//		if (caa == 0) caa = 75;    caa--;
//	}
//}



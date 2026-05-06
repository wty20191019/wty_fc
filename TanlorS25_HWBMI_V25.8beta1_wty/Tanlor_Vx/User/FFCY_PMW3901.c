#include "Headfile.h"
#include "PMW3901.h"

s16 DetX, DetY;
s16 DetX_Last, DetY_Last;

void PMW3901_SPI_Init(void)
{

	GPIO_InitTypeDef  GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOB时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);//使能SPI1时钟

														//GPIOFB3,4,5初始化设置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;//PB3~5复用功能输出	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;//PB14
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//输出
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化


	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2); //PB3复用为 SPI1
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2); //PB4复用为 SPI1
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2); //PB5复用为 SPI1

															//这里只针对SPI口初始化
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2, ENABLE);//复位SPI1
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2, DISABLE);//停止复位SPI1

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//设置SPI工作模式:设置为主SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//设置SPI的数据大小:SPI发送接收8位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		//串行同步时钟的空闲状态为高电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	//串行同步时钟的第二个跳变沿（上升或下降）数据被采样
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;		//定义波特率预分频的值:波特率预分频值为256
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial = 0;	//CRC值计算的多项式
	SPI_Init(SPI2, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器

	SPI_Cmd(SPI2, ENABLE); //使能SPI外设

	PMW3901_CS = 0;
	PMW3901_CS = 1;//启动传输		 
}


/***************************************
函数名：u8 SPI_ReadWriteByte(u8 txData)
说明：SPI读写一个字节函数
入口：u8 txData MOSI写出的字节
出口：u8	MISO写入的字节
备注：返回值为0时，表明没有发送/接收成功
****************************************/
u8 SPI2_ReadWriteByte(u8 txData)
{
	u8 retry = 0;

	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) //检查指定的SPI标志位设置与否:发送缓存空标志位
	{
		retry++;
		if (retry > 200)			//超时
			return 0;
	}
	SPI_I2S_SendData(SPI2, txData); 							//通过外设SPIx发送一个数据
	retry = 0;
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET) //检查指定的SPI标志位设置与否:接受缓存非空标志位
	{
		retry++;
		if (retry > 200)				//超时
			return 0;
	}
	return SPI_I2S_ReceiveData(SPI2); 						//返回通过SPIx最近接收的数据
}



//SPI 写寄存器
//reg:指定寄存器地址
//value:写入的值
u8 PMW3901_Write_Reg(u8 reg, u8 value)
{
	u8 status;
	PMW3901_CS = 0;   //使能 SPI 传输 
	//Delay_Ms(1);
	reg = reg | 0x80;
	SPI2_ReadWriteByte(reg);//发送寄存器号 
	SPI2_ReadWriteByte(value);             //写入寄存器的值 
	//Delay_Ms(1);
	PMW3901_CS =1;                            //禁止 SPI 传输 
	return(status);              //返回状态值
}


//读取 SPI 寄存器值
//reg:要读的寄存器
u8 PMW3901_Read_Reg(u8 reg)
{
	u8 reg_val;
	PMW3901_CS = 0;                   //使能 SPI 传输 
	reg = reg & 0x7F;
	SPI2_ReadWriteByte(reg);             //发送寄存器号 
	reg_val=SPI2_ReadWriteByte(0XFF);    //读取寄存器内容 
	PMW3901_CS = 1;                  //禁止 SPI 传输 
	return(reg_val);                  //返回状态值
}


/*******************************************************
函数名：void NRF24L01_Read_Buf(u8 reg, u8 *pBuf, u8 len)
说明：在指定位置读出指定长度的数据
入口：u8 reg		寄存器(位置)
u8 *pBuf	数据指针
u8 len		数据长度
出口：无
备注：无
*******************************************************/
void PMW3901_Read_Buf(u8 reg, u8 *pBuf, u8 len)
{
	u8 i;

	PMW3901_CS;           							//使能SPI传输
	SPI2_ReadWriteByte(reg);									//发送寄存器值(位置)
	for (i = 0; i < len; i++)
		pBuf[i] = SPI2_ReadWriteByte(0XFF);		//读出数据
	PMW3901_CS;       									//关闭SPI传输
}

/**********************************FFCY 2018**************************************
** Function name:     PMW3901_initRegisters()
** Descriptions:      初始化PMW3901的寄存器，即性能优化寄存器
** input parameters:  
** output parameters: 
** Returned value:    
**********************************FFCY 2018**************************************/

u8 PMW3901_Reg_047 = 0;
u8 PMW3901_Reg_067 = 0;
u8 PMW3901_Reg_073 = 0;
u8 PMW3901_Reg_070 = 0;
u8 PMW3901_Reg_071 = 0;
void PMW3901_initRegisters()
{


	PMW3901_Write_Reg(0x7F, 0x00);
	PMW3901_Write_Reg(0x61, 0xAD);
	PMW3901_Write_Reg(0x7F, 0x03);
	PMW3901_Write_Reg(0x40, 0x00);
	PMW3901_Write_Reg(0x7F, 0x05);
	PMW3901_Write_Reg(0x41, 0xB3);
	PMW3901_Write_Reg(0x43, 0xF1);
	PMW3901_Write_Reg(0x45, 0x14);
	PMW3901_Write_Reg(0x5B, 0x32);
	PMW3901_Write_Reg(0x5F, 0x34);
	PMW3901_Write_Reg(0x7B, 0x08);
	PMW3901_Write_Reg(0x7F, 0x06);
	PMW3901_Write_Reg(0x44, 0x1B);
	PMW3901_Write_Reg(0x40, 0xBF);
	PMW3901_Write_Reg(0x4E, 0x3F);
	PMW3901_Write_Reg(0x7F, 0x08);
	PMW3901_Write_Reg(0x65, 0x20);
	PMW3901_Write_Reg(0x6A, 0x18);
	PMW3901_Write_Reg(0x7F, 0x09);
	PMW3901_Write_Reg(0x4F, 0xAF);
	PMW3901_Write_Reg(0x5F, 0x40);
	PMW3901_Write_Reg(0x48, 0x80);
	PMW3901_Write_Reg(0x49, 0x80);
	PMW3901_Write_Reg(0x57, 0x77);
	PMW3901_Write_Reg(0x60, 0x78);
	PMW3901_Write_Reg(0x61, 0x78);
	PMW3901_Write_Reg(0x62, 0x08);
	PMW3901_Write_Reg(0x63, 0x50);
	PMW3901_Write_Reg(0x7F, 0x0A);
	PMW3901_Write_Reg(0x45, 0x60);
	PMW3901_Write_Reg(0x7F, 0x00);
	PMW3901_Write_Reg(0x4D, 0x11);
	PMW3901_Write_Reg(0x55, 0x80);
	PMW3901_Write_Reg(0x74, 0x1F);
	PMW3901_Write_Reg(0x75, 0x1F);
	PMW3901_Write_Reg(0x4A, 0x78);
	PMW3901_Write_Reg(0x4B, 0x78);
	PMW3901_Write_Reg(0x44, 0x08);
	PMW3901_Write_Reg(0x45, 0x50);
	PMW3901_Write_Reg(0x64, 0xFF);
	PMW3901_Write_Reg(0x65, 0x1F);
	PMW3901_Write_Reg(0x7F, 0x14);
	PMW3901_Write_Reg(0x65, 0x67);
	PMW3901_Write_Reg(0x66, 0x08);
	PMW3901_Write_Reg(0x63, 0x70);
	PMW3901_Write_Reg(0x7F, 0x15);
	PMW3901_Write_Reg(0x48, 0x48);
	PMW3901_Write_Reg(0x7F, 0x07);
	PMW3901_Write_Reg(0x41, 0x0D);
	PMW3901_Write_Reg(0x43, 0x14);
	PMW3901_Write_Reg(0x4B, 0x0E);
	PMW3901_Write_Reg(0x45, 0x0F);
	PMW3901_Write_Reg(0x44, 0x42);
	PMW3901_Write_Reg(0x4C, 0x80);
	PMW3901_Write_Reg(0x7F, 0x10);
	PMW3901_Write_Reg(0x5B, 0x02);
	PMW3901_Write_Reg(0x7F, 0x07);
	PMW3901_Write_Reg(0x40, 0x41);
	PMW3901_Write_Reg(0x70, 0x00);

	delay_ms(10); // delay 10ms

	PMW3901_Write_Reg(0x32, 0x44);
	PMW3901_Write_Reg(0x7F, 0x07);
	PMW3901_Write_Reg(0x40, 0x40);
	PMW3901_Write_Reg(0x7F, 0x06);
	PMW3901_Write_Reg(0x62, 0xF0);
	PMW3901_Write_Reg(0x63, 0x00);
	PMW3901_Write_Reg(0x7F, 0x0D);
	PMW3901_Write_Reg(0x48, 0xC0);
	PMW3901_Write_Reg(0x6F, 0xD5);
	PMW3901_Write_Reg(0x7F, 0x00);
	PMW3901_Write_Reg(0x5B, 0xA0);
	PMW3901_Write_Reg(0x4E, 0xA8);
	PMW3901_Write_Reg(0x5A, 0x50);
	PMW3901_Write_Reg(0x40, 0x80);


}


/**********************************FFCY 2018**************************************
** Function name:      PMW3901_Init（）
** Descriptions:       PMW3901初始化
** input parameters:  
** output parameters: 
** Returned value:    
**********************************FFCY 2018**************************************/
u8  PMW3901_Init(void )
{
	u8 chipId=0, dIpihc=0;
	PMW3901_CS = 1;
	PMW3901_SPI_Init();
	delay_ms(40);
	PMW3901_CS = 1;
	delay_ms(1);
	PMW3901_CS = 0;

    PMW3901_Write_Reg(0x3A, 0x5A);


	delay_ms(10);

	PMW3901_Read_Reg(0x02);
	PMW3901_Read_Reg(0x03);
	PMW3901_Read_Reg(0x04);
	PMW3901_Read_Reg(0x05);
	PMW3901_Read_Reg(0x06);
	delay_ms(10);


	// PMW3901_Write_Reg(0x3a, 0x5a);
	// PMW3901_Write_Reg(0x3B, 0xb6);
	 

	 PMW3901_initRegisters();
	 delay_ms(10);
    //PMW3901_Write_Reg(0x3B, 0xb6);
	 return 1;
}


/**********************************FFCY 2018**************************************
** Function name:     PMW3901_Getinit_OK()
** Descriptions:      检测PMW3901是否在线
** input parameters:  
** output parameters: 
** Returned value:    
**********************************FFCY 2018**************************************/
u8  PMW3901_Getinit_OK()
{
	u8 PMW3901_Chip_ID = 0;
	u8 PMW3901_dIpihc_ID = 0;

	//读取传感器ID值
	PMW3901_Chip_ID = PMW3901_Read_Reg(0x00);
	PMW3901_dIpihc_ID = PMW3901_Read_Reg(0x5F);


	if (PMW3901_Chip_ID != 0)
		return 1;
	else
		return 0;

}

/**********************************FFCY 2018**************************************
** Function name:     PMW3901_Read(int *deltaX, int *deltaY)
** Descriptions:      读取PMW3901的deltaX,deltaY
** input parameters:  
** output parameters: 
** Returned value:    
**********************************FFCY 2018**************************************/
u8 PMW3901_SQUAL = 0;
u16 PMW3901_Shutter_Upper = 0;
u16 PMW3901_Shutter_Lower = 0;
u16 PMW3901_Shutter = 0;
void PMW3901_Read_deltaXY(s16 *deltaX, s16 *deltaY)
{
	u8 b2 = 0; u8 b3 = 0; u8 b4 = 0; u8 b5 = 0; u8 b6 = 0; u8 b7 = 0; u8 b8 = 0;  u8 b9 = 0;
	b2 = PMW3901_Read_Reg(0x02);
	b3 = PMW3901_Read_Reg(0x03);
	b4 = PMW3901_Read_Reg(0x04);
	b5 = PMW3901_Read_Reg(0x05);
	b6 = PMW3901_Read_Reg(0x06);
	b7 = PMW3901_Read_Reg(0x07);  //SQUAL value
	b8 = PMW3901_Read_Reg(0x0B);  //Shutter_Upper
	b9 = PMW3901_Read_Reg(0x0c);  //Shutter_Upper
	DetX = ((s16)b4 << 8) | b3;
	DetY = ((s16)b6 << 8) | b5;
	PMW3901_SQUAL = b7;
	PMW3901_Shutter_Lower = b8;
	PMW3901_Shutter_Upper = b9;// (((u16)b9 << 8) & 0x1F00) | b8;
	PMW3901_Shutter = (((u16)b9 << 8) & 0x1F00) | b8;
	//To suppress false motion reports, discard Delta_X and Delta Y values 
	//if the SQUAL value < 0x19 and Shutter_Upper = 0x1F
}





u8 RawData[1225];
u8 datadadsad = 0;
u8 Rdg_Data = 0; 
u32 i_cnt = 0;
u32 fail_cnt = 0;
void PMW3901_Read_Images(void) 
{
	u8 RDG_One_buff = 0;
	i_cnt = 0;

	PMW3901_Write_Reg(0x70, 0x00);
	PMW3901_Write_Reg(0x58, 0xFF);

	while (1)
	{
		Rdg_Data = PMW3901_Read_Reg(0x58);
		if (Rdg_Data & 0b11000000 == 0b11000000)
			break;
		else
			delay_ms(1);
	}

	while (i_cnt<1224)
	{
		Rdg_Data = PMW3901_Read_Reg(0x58);
		if ((Rdg_Data & 0xC0) == 0x40)
			RDG_One_buff = (Rdg_Data & 0b00111111)<<2;
		else if ((Rdg_Data & 0b11000000) == 0b10000000)
			RawData[i_cnt++] = RDG_One_buff| (Rdg_Data >> 2 & 0b00000011);
		else
			fail_cnt++;
	}

	datadadsad = RawData[50];
	delay_ms(20);
}



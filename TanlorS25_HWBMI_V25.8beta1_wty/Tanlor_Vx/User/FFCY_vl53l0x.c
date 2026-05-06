#include "Headfile.h"
#include "FFCY_vl53l0x.h"
#include "delay.h"

float HC_SR04_Distance, HC_SR04_Distance_Filter, HC_SR04_Bubble_Distance;
float HC_SR04_Distance_Record[5], HCSR04_Data[5];
 uint8_t HCSR_04_Update_Flag;


u8 VL53L0X_Read_buff[16];
u32 VL53L0X_Distance_data = 0;
VL53LX VL53Lx_Data;

#pragma region Vl53LX的模拟IIC

//**************************飞**********凡*********创**********翼******************************************
//*	函 数 名 :  VL53LX_delay_us(void)
//*	功能说明 : Vl53延时 ，因为在中断中调用，因此不能使用滴答定时器的Delay
//*	形    参 ：无
//*	返 回 值 : 无
//* 作者    ：FFCY
//* 最后更改：CP_SYD 孙艺东 2019:7:15 14:02:00 
//**********************************(C)COPYRIGHT 2019 FFCY Team*********************************************/
void VL53LX_delay_us(u8 us)
{
	volatile int i = us;	//7
	while (i)
	    i--;
}
//**************************飞**********凡*********创**********翼******************************************
//*	函 数 名 :  VL53L0X_I2C_init(void)
//*	功能说明 : IIC  IO初始化  PB8 PB9
//*	形    参 ：无
//*	返 回 值 : 无
//* 作者    ：FFCY
//* 最后更改：CP_SYD 孙艺东 2019:7:15 14:02:00 
//**********************************(C)COPYRIGHT 2019 FFCY Team*********************************************/
void VL53LX_I2C_init(void)
{
	//GPIO_InitTypeDef GPIO_InitStructure;
	//RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );	//使能GPIOB时钟

	//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9| GPIO_Pin_8;  //端口配置
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;       //推挽输出
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;       //50Mhz速度
	//GPIO_Init(GPIOB, &GPIO_InitStructure);	
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//使能GPIOB时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化

	VL_IIC_SCL = 1;
	VL_IIC_SDA = 1;
}

//**************************飞**********凡*********创**********翼******************************************
//*	函 数 名 :  void VL_IIC_Start(void)
//*	功能说明 : 产生IIC起始信号
//*	形    参 ：无
//*	返 回 值 : 无
//* 作者    ：FFCY
//* 最后更改：CP_SYD 孙艺东 2019:7:15 14:03:55 
//**********************************(C)COPYRIGHT 2019 FFCY Team*********************************************/

void VL_IIC_Start(void)
{
	VL_SDA_OUT();//sda线输出
	VL_IIC_SDA=1;	  	  
	VL_IIC_SCL=1;
	VL53LX_delay_us(4);
 	VL_IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	VL53LX_delay_us(4);
	VL_IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
}


//**************************飞**********凡*********创**********翼******************************************
//*	函 数 名 :  void VL_IIC_Stop(void)
//*	功能说明 : 产生IIC停止信号
//*	形    参 ：无
//*	返 回 值 : 无
//* 作者    ：FFCY
//* 最后更改：CP_SYD 孙艺东 2019:7:15 14:04:11 
//**********************************(C)COPYRIGHT 2019 FFCY Team*********************************************/
void VL_IIC_Stop(void)
{
	VL_SDA_OUT();//sda线输出
	VL_IIC_SCL=0;
	VL_IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
	VL53LX_delay_us(4);
	VL_IIC_SCL=1; 
	VL_IIC_SDA=1;//发送I2C总线结束信号
	VL53LX_delay_us(4);							   	
}
//**************************飞**********凡*********创**********翼******************************************
//*	函 数 名 :  u8 VL_IIC_Wait_Ack(void)
//*	功能说明 : 等待应答信号到来
//*	形    参 ：无
//*	返 回 值 : 1，接收应答失败
//*            0，接收应答成功
//* 作者    ：FFCY
//* 最后更改：CP_SYD 孙艺东 2019:7:15 14:04:37 
//**********************************(C)COPYRIGHT 2019 FFCY Team*********************************************/
u8 VL_IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	VL_SDA_IN();  //SDA设置为输入  
	VL_IIC_SDA=1;VL53LX_delay_us(1);	   
	VL_IIC_SCL=1;VL53LX_delay_us(1);	 
	while(VL_READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			VL_IIC_Stop();
			return 1;
		}
	}
	VL_IIC_SCL=0;//时钟输出0 	   
	return 0;  
}

//**************************飞**********凡*********创**********翼******************************************
//*	函 数 名 :  void VL_IIC_Ack(void)
//*	功能说明 : 产生ACK应答
//*	形    参 ：无
//*	返 回 值 : 无
//* 作者    ：FFCY
//* 最后更改：CP_SYD 孙艺东 2019:7:15 14:05:18 
//**********************************(C)COPYRIGHT 2019 FFCY Team*********************************************/
void VL_IIC_Ack(void)
{
	VL_IIC_SCL=0;
	VL_SDA_OUT();
	VL_IIC_SDA=0;
	VL53LX_delay_us(2);
	VL_IIC_SCL=1;
	VL53LX_delay_us(2);
	VL_IIC_SCL=0;
}
//**************************飞**********凡*********创**********翼******************************************
//*	函 数 名 :  void VL_IIC_NAck(void)
//*	功能说明 : 不产生ACK应答	
//*	形    参 ：无
//*	返 回 值 : 无
//* 作者    ：FFCY
//* 最后更改：CP_SYD 孙艺东 2019:7:15 14:08:27 
//**********************************(C)COPYRIGHT 2019 FFCY Team*********************************************/
void VL_IIC_NAck(void)
{
	VL_IIC_SCL=0;
	VL_SDA_OUT();
	VL_IIC_SDA=1;
	VL53LX_delay_us(2);
	VL_IIC_SCL=1;
	VL53LX_delay_us(2);
	VL_IIC_SCL=0;
}
//**************************飞**********凡*********创**********翼******************************************
//*	函 数 名 :  void VL_IIC_Send_Byte(u8 txd)
//*	功能说明 : IIC发送一个字节
//*	形    参 ：无
//*	返 回 值 : 1，有应答
/*             0，无应答	
//* 作者    ：FFCY
//* 最后更改：CP_SYD 孙艺东 2019:7:15 14:08:51 
//**********************************(C)COPYRIGHT 2019 FFCY Team*********************************************/
void VL_IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	VL_SDA_OUT(); 	    
    VL_IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
		if((txd&0x80)>>7)
			VL_IIC_SDA=1;
		else
			VL_IIC_SDA=0;
		txd<<=1; 	  
		VL53LX_delay_us(2);  
		VL_IIC_SCL=1;
		VL53LX_delay_us(2); 
		VL_IIC_SCL=0;	
		VL53LX_delay_us(2);
    }	 
} 
//**************************飞**********凡*********创**********翼******************************************
//*	函 数 名 :  u8 VL_IIC_Read_Byte(unsigned char ack)
//*	功能说明 : 读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
//*	形    参 ：无
//*	返 回 值 : 无
//* 作者    ：FFCY
//* 最后更改：CP_SYD 孙艺东 2019:7:15 14:09:36 
//**********************************(C)COPYRIGHT 2019 FFCY Team*********************************************/
u8 VL_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	VL_SDA_IN();      //SDA设置为输入
	for(i=0;i<8;i++ )
	{
		VL_IIC_SCL=0; 
		VL53LX_delay_us(4);
	  VL_IIC_SCL=1;
		receive<<=1;
		if(VL_READ_SDA)receive++;   
	  VL53LX_delay_us(4); //1
	}					 
	if (!ack)
		VL_IIC_NAck();//发送nACK
	else
		VL_IIC_Ack(); //发送ACK   
	return receive;
}

//**************************飞**********凡*********创**********翼******************************************
//*	函 数 名 :  u8 VL_IIC_Write_1Byte(u8 SlaveAddress, u8 REG_Address,u8 REG_data)
//*	功能说明 :  写1byte数据
//*	形    参 ：无
//*	返 回 值 : 1.失败
//*            0.成功
//* 作者    ：FFCY
//* 最后更改：CP_SYD 孙艺东 2019:7:15 14:21:09 
//**********************************(C)COPYRIGHT 2019 FFCY Team*********************************************/
u8 VL_IIC_Write_1Byte(u8 SlaveAddress, u8 REG_Address,u8 REG_data)
{
	VL_IIC_Start();
	VL_IIC_Send_Byte(SlaveAddress);
	if(VL_IIC_Wait_Ack())
	{
		VL_IIC_Stop();//释放总线
		return 1;//没应答则退出

	}
	VL_IIC_Send_Byte(REG_Address);
	VL_IIC_Wait_Ack();	
	VL_IIC_Send_Byte(REG_data);
	VL_IIC_Wait_Ack();	
	VL_IIC_Stop();

	return 0;
}


//**************************飞**********凡*********创**********翼******************************************
//*	函 数 名 :  u8 VL_IIC_Write_16address_Byte(u8 SlaveAddress, u16 REG_Address, u8 REG_data)
//*	功能说明 :  向16位地址写1byte数据
//*	形    参 ：无
//*	返 回 值 : 1.失败
//*            0.成功
//* 作者    ：FFCY
//* 最后更改：CP_SYD 孙艺东 2019:7:15 14:22:48 
//**********************************(C)COPYRIGHT 2019 FFCY Team*********************************************/
u8 VL_IIC_Write_16address_Byte(u8 SlaveAddress, u16 REG_Address, u8 REG_data)
{

	VL_IIC_Start();
	VL_IIC_Send_Byte(SlaveAddress);//发写命令

	if (VL_IIC_Wait_Ack())
	{
		VL_IIC_Stop();//释放总线
		return 1;//没应答则退出
	}

	VL_IIC_Send_Byte((REG_Address >> 8) & 0xFF);
	VL_IIC_Wait_Ack();
	VL_IIC_Send_Byte(REG_Address & 0xFF);
	VL_IIC_Wait_Ack();


	VL_IIC_Send_Byte(REG_data );//发送buff的数据
	VL_IIC_Wait_Ack();

	VL_IIC_Stop();//释放总线
	return 0;
}

//**************************飞**********凡*********创**********翼******************************************
//*	函 数 名 :  u8 VL_IIC_Write_16address_4Byte(u8 SlaveAddress, u16 REG_Address, u32 REG_data)
//*	功能说明 :  向16位地址 写4Byet数据
//*	形    参 ：无
//*	返 回 值 : 1.失败
//*            0.成功
//* 作者    ：FFCY
//* 最后更改：CP_SYD 孙艺东 2019:7:15 14:24:27 
//**********************************(C)COPYRIGHT 2019 FFCY Team*********************************************/
u8 VL_IIC_Write_16address_4Byte(u8 SlaveAddress, u16 REG_Address, u32 REG_data)
{

	VL_IIC_Start();
	VL_IIC_Send_Byte(SlaveAddress);//发写命令

	if (VL_IIC_Wait_Ack())
	{
		VL_IIC_Stop();//释放总线
		return 1;//没应答则退出
	}

	VL_IIC_Send_Byte((REG_Address >> 8) & 0xFF);
	VL_IIC_Wait_Ack();
	VL_IIC_Send_Byte(REG_Address & 0xFF);
	VL_IIC_Wait_Ack();

	 	VL_IIC_Send_Byte((REG_data >> 24) & 0xFF);//发送buff的数据
	 	VL_IIC_Wait_Ack();
		VL_IIC_Send_Byte((REG_data >> 16) & 0xFF);//发送buff的数据
		VL_IIC_Wait_Ack();
		VL_IIC_Send_Byte((REG_data >> 8) & 0xFF);//发送buff的数据
		VL_IIC_Wait_Ack();
	VL_IIC_Send_Byte(REG_data & 0xFF);//发送buff的数据
	VL_IIC_Wait_Ack();

	VL_IIC_Stop();//释放总线
	return 0;

}

//**************************飞**********凡*********创**********翼******************************************
//*	函 数 名 :  u8 VL_IIC_Read_1Byte(u8 SlaveAddress, u8 REG_Address,u8 *REG_data)
//*	功能说明 : IIC读一个字节数据
//*	形    参 ：无
//*	返 回 值 : 1.失败
//*            0.成功
//* 作者    ：FFCY
//* 最后更改：CP_SYD 孙艺东 2019:7:15 14:25:23 
//**********************************(C)COPYRIGHT 2019 FFCY Team*********************************************/
u8 VL_IIC_Read_1Byte(u8 SlaveAddress, u8 REG_Address,u8 *REG_data)
{
	VL_IIC_Start();
	VL_IIC_Send_Byte(SlaveAddress);//发写命令
	if(VL_IIC_Wait_Ack())
	{
		 VL_IIC_Stop();//释放总线
		 return 1;//没应答则退出
	}		
	VL_IIC_Send_Byte(REG_Address);
	VL_IIC_Wait_Ack();
	VL_IIC_Start(); 
	VL_IIC_Send_Byte(SlaveAddress|0x01);//发读命令
	VL_IIC_Wait_Ack();
	*REG_data = VL_IIC_Read_Byte(0);
	VL_IIC_Stop();
	return 0;
}
//**************************飞**********凡*********创**********翼******************************************
//*	函 数 名 :  u16  VL_IIC_Read_2Byte(u8 SlaveAddress, u16 REG_Address, u16 *REG_DATA)
//*	功能说明 : 从16位地址读2Byet数据
//*	形    参 ：无
//*	返 回 值 : 1.失败
//*            0.成功
//* 作者    ：FFCY
//* 最后更改：CP_SYD 孙艺东 2019:7:15 14:26:30 
//**********************************(C)COPYRIGHT 2019 FFCY Team*********************************************/
u16  VL_IIC_Read_2Byte(u8 SlaveAddress, u16 REG_Address, u16 *REG_DATA)
{

	VL_IIC_Start();
	VL_IIC_Send_Byte(SlaveAddress);//发写命令

	if (VL_IIC_Wait_Ack())
	{
		VL_IIC_Stop();//释放总线
		return 1;//没应答则退出
	}
	VL_IIC_Send_Byte((REG_Address>>8)&0xFF);
	VL_IIC_Wait_Ack();
	VL_IIC_Send_Byte(REG_Address&0xff);
	VL_IIC_Wait_Ack();

	VL_IIC_Start();
	VL_IIC_Send_Byte(SlaveAddress | 0x01);//发读命令
	VL_IIC_Wait_Ack();


	*REG_DATA= VL_IIC_Read_Byte(1) & 0x00FF;
	*REG_DATA= *REG_DATA << 8;
	*REG_DATA= *REG_DATA+VL_IIC_Read_Byte(0);

	VL_IIC_Stop();//释放总线
	return *REG_DATA;
}
//**************************飞**********凡*********创**********翼******************************************
//*	函 数 名 :  u16  VL_IIC_Read_2ByteQ(u16 REG_Address)
//*	功能说明 :  快速读2B
//*	形    参 ：无
//*	返 回 值 : 无
//* 作者    ：FFCY
//* 最后更改：CP_SYD 孙艺东 2019:7:15 14:28:29 
//**********************************(C)COPYRIGHT 2019 FFCY Team*********************************************/
u16  VL_IIC_Read_2ByteQ(u16 REG_Address)
{
	u16 REG_DATA;

	VL_IIC_Start();
	VL_IIC_Send_Byte(0x52);//发写命令

	if (VL_IIC_Wait_Ack())
	{
		VL_IIC_Stop();//释放总线
		return 1;//没应答则退出
	}
	VL_IIC_Send_Byte((REG_Address >> 8) & 0xFF);
 	VL_IIC_Wait_Ack();
	VL_IIC_Send_Byte(REG_Address & 0xff);
	VL_IIC_Wait_Ack();

	VL_IIC_Start();
	VL_IIC_Send_Byte(0x52 | 0x01);//发读命令
	VL_IIC_Wait_Ack();


	REG_DATA = VL_IIC_Read_Byte(1) & 0x00FF;
	REG_DATA = REG_DATA << 8;
	REG_DATA = REG_DATA + VL_IIC_Read_Byte(0);

	VL_IIC_Stop();//释放总线
	return REG_DATA;
}

//**************************飞**********凡*********创**********翼******************************************
//*	函 数 名 :  u8 VL_IIC_Write_nByte(u8 SlaveAddress, u8 REG_Address,u16 len, u8 *buf)
//*	功能说明 : IIC写n字节数据
//*	形    参 ：无
//*	返 回 值 : 无
//* 作者    ：FFCY
//* 最后更改：CP_SYD 孙艺东 2019:7:15 14:47:34 
//**********************************(C)COPYRIGHT 2019 FFCY Team*********************************************/
u8 VL_IIC_Write_nByte(u8 SlaveAddress, u8 REG_Address,u16 len, u8 *buf)
{

	VL_IIC_Start();
	VL_IIC_Send_Byte(SlaveAddress);//发写命令
	if(VL_IIC_Wait_Ack()) 
	{
		VL_IIC_Stop();//释放总线
		return 1;//没应答则退出
	}
	VL_IIC_Send_Byte(REG_Address);
	VL_IIC_Wait_Ack();
	while(len--)
	{
		VL_IIC_Send_Byte(*buf++);//发送buff的数据
		VL_IIC_Wait_Ack();	
	}
	VL_IIC_Stop();//释放总线

	return 0;
}
//**************************飞**********凡*********创**********翼******************************************
//*	函 数 名 :  u8 VL_IIC_Read_nByte(u8 SlaveAddress, u8 REG_Address,u16 len,u8 *buf)
//*	功能说明 : IIC读n字节数据
//*	形    参 ：无
//*	返 回 值 : 无
//* 作者    ：FFCY
//* 最后更改：CP_SYD 孙艺东 2019:7:15 14:47:56 
//**********************************(C)COPYRIGHT 2019 FFCY Team*********************************************/
u8 VL_IIC_Read_nByte(u8 SlaveAddress, u8 REG_Address,u16 len,u8 *buf)
{
	VL_IIC_Start();
	VL_IIC_Send_Byte(SlaveAddress);//发写命令
	if(VL_IIC_Wait_Ack()) 
	{
		VL_IIC_Stop();//释放总线
		return 1;//没应答则退出
	}
	VL_IIC_Send_Byte(REG_Address);
	VL_IIC_Wait_Ack();

	VL_IIC_Start();
	VL_IIC_Send_Byte(SlaveAddress|0x01);//发读命令
	VL_IIC_Wait_Ack();
	while(len)
	{
		if(len==1)
		{
			*buf = VL_IIC_Read_Byte(0);
		}
		else
		{
			*buf = VL_IIC_Read_Byte(1);
		}
		buf++;
		len--;
	}
	VL_IIC_Stop();//释放总线

	return 0;
	
}
//**************************飞**********凡*********创**********翼******************************************
//*	函 数 名 :  u8 VL_IIC_Read_nByte16(u8 SlaveAddress, u16 REG_Address, u16 len, u8 *buf)
//*	功能说明 :  向16位地址读nByte
//*	形    参 ：无
//*	返 回 值 : 无
//* 作者    ：FFCY
//* 最后更改：CP_SYD 孙艺东 2019:7:15 14:48:16 
//**********************************(C)COPYRIGHT 2019 FFCY Team*********************************************/
u8 VL_IIC_Read_nByte16(u8 SlaveAddress, u16 REG_Address, u16 len, u8 *buf)
{
	VL_IIC_Start();
	VL_IIC_Send_Byte(SlaveAddress);//发写命令
	if (VL_IIC_Wait_Ack())
	{
		VL_IIC_Stop();//释放总线
		return 1;//没应答则退出
	}
	VL_IIC_Send_Byte((REG_Address>>8)&0xff);
	VL_IIC_Wait_Ack();
	VL_IIC_Send_Byte(REG_Address&0xff);
	VL_IIC_Wait_Ack();

	VL_IIC_Start();
	VL_IIC_Send_Byte(SlaveAddress | 0x01);//发读命令
	VL_IIC_Wait_Ack();
	while (len)
	{
		if (len == 1)
		{
			*buf = VL_IIC_Read_Byte(0);
		}
		else
		{
			*buf = VL_IIC_Read_Byte(1);
		}
		buf++;
		len--;
	}
	VL_IIC_Stop();//释放总线

	return 0;

}
#pragma endregion
//**************************飞**********凡*********创**********翼******************************************
//*	函 数 名 :   u16 makeuint16(int lsb, int msb)  
//*	功能说明 :  将两个U8类型的数转化成U16类型
//*	形    参 ：无
//*	返 回 值 : 无
//* 作者    ：FFCY
//* 最后更改：CP_SYD 孙艺东 2019:7:15 14:51:33 
//**********************************(C)COPYRIGHT 2019 FFCY Team*********************************************/
u16 makeuint16(u8 lsb, u8 msb)
{
	return (((u16)msb & 0x00FF) << 8) | (lsb & 0xFF);
}

//**************************飞**********凡*********创**********翼******************************************
//*	函 数 名 :  bool VL53L1X_setDistanceMode(u8 mode)
//*	功能说明 :  设置VL53L1 测量模式
//*	形    参 ：无
//*	返 回 值 : 无
//* 作者    ：FFCY
//* 最后更改：CP_SYD 孙艺东 2019:7:15 14:15:38 
//**********************************(C)COPYRIGHT 2019 FFCY Team*********************************************/
u8 VL53L1X_setDistanceMode(u8 mode)
{
	// save existing timing budget
	//uint32_t budget_us = getMeasurementTimingBudget();

	switch (mode)
	{
	case 1 :  //Short:
		// from VL53L1_preset_mode_standard_ranging_short_range()

		// timing config
		VL_IIC_Write_16address_Byte(VL53L1X_Address,RANGE_CONFIG__VCSEL_PERIOD_A, 0x07);
		VL_IIC_Write_16address_Byte(VL53L1X_Address, RANGE_CONFIG__VCSEL_PERIOD_B, 0x05);
		VL_IIC_Write_16address_Byte(VL53L1X_Address, RANGE_CONFIG__VALID_PHASE_HIGH, 0x38);

		// dynamic config
		VL_IIC_Write_16address_Byte(VL53L1X_Address, SD_CONFIG__WOI_SD0, 0x07);
		VL_IIC_Write_16address_Byte(VL53L1X_Address, SD_CONFIG__WOI_SD1, 0x05);
		VL_IIC_Write_16address_Byte(VL53L1X_Address, SD_CONFIG__INITIAL_PHASE_SD0, 6); // tuning parm default
		VL_IIC_Write_16address_Byte(VL53L1X_Address, SD_CONFIG__INITIAL_PHASE_SD1, 6); // tuning parm default

		break;

	case 22://Medium:
		// from VL53L1_preset_mode_standard_ranging()

		// timing config
		VL_IIC_Write_16address_Byte(VL53L1X_Address, RANGE_CONFIG__VCSEL_PERIOD_A, 0x0B);
		VL_IIC_Write_16address_Byte(VL53L1X_Address, RANGE_CONFIG__VCSEL_PERIOD_B, 0x09);
		VL_IIC_Write_16address_Byte(VL53L1X_Address, RANGE_CONFIG__VALID_PHASE_HIGH, 0x78);

		// dynamic config
		VL_IIC_Write_16address_Byte(VL53L1X_Address, SD_CONFIG__WOI_SD0, 0x0B);
		VL_IIC_Write_16address_Byte(VL53L1X_Address, SD_CONFIG__WOI_SD1, 0x09);
		VL_IIC_Write_16address_Byte(VL53L1X_Address, SD_CONFIG__INITIAL_PHASE_SD0, 10); // tuning parm default
		VL_IIC_Write_16address_Byte(VL53L1X_Address, SD_CONFIG__INITIAL_PHASE_SD1, 10); // tuning parm default

		break;

	case 3:// Long: // long
			   // from VL53L1_preset_mode_standard_ranging_long_range()

			   // timing config
		VL_IIC_Write_16address_Byte(VL53L1X_Address, RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F);
		VL_IIC_Write_16address_Byte(VL53L1X_Address, RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D);
		VL_IIC_Write_16address_Byte(VL53L1X_Address, RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8);

		// dynamic config
		VL_IIC_Write_16address_Byte(VL53L1X_Address, SD_CONFIG__WOI_SD0, 0x0F);
		VL_IIC_Write_16address_Byte(VL53L1X_Address, SD_CONFIG__WOI_SD1, 0x0D);
		VL_IIC_Write_16address_Byte(VL53L1X_Address, SD_CONFIG__INITIAL_PHASE_SD0, 14); // tuning parm default
		VL_IIC_Write_16address_Byte(VL53L1X_Address, SD_CONFIG__INITIAL_PHASE_SD1, 14); // tuning parm default

		break;

	default:
		// unrecognized mode - do nothing
		return 0;
	}

	// reapply timing budget
	//setMeasurementTimingBudget(budget_us);

	// save mode so it can be returned by getDistanceMode()

	return 1;
}
//**************************飞**********凡*********创**********翼******************************************
//*	函 数 名 :  void VL53L1X_startContinuous(u16 period_ms)
//*	功能说明 :  设置连续测量模式
//*	形    参 ：  period_ms 周期
//*	返 回 值 : 无
//* 作者    ：FFCY
//* 最后更改：CP_SYD 孙艺东 2019:7:15 14:13:20 
//**********************************(C)COPYRIGHT 2019 FFCY Team*********************************************/
void VL53L1X_startContinuous(u16 period_ms,u16 OSC_VAL)
{
	VL_IIC_Write_16address_4Byte(VL53L1X_Address,SYSTEM__INTERMEASUREMENT_PERIOD, period_ms * OSC_VAL);
	VL_IIC_Write_16address_Byte(VL53L1X_Address, SYSTEM__INTERRUPT_CLEAR, 0x01); // sys_interrupt_clear_range
	VL_IIC_Write_16address_Byte(VL53L1X_Address, SYSTEM__MODE_START, 0x40); // mode_range__timed
}

//**************************飞**********凡*********创**********翼******************************************
//*	函 数 名 :  u8 VL53L0X_Getinit_OK()
//*	功能说明 :  检查Vl53L0是否在线
//*	形    参 ：无
//*	返 回 值 :  1  成功
//*             0  失败
//* 作者    ：FFCY
//* 最后更改：CP_SYD 孙艺东 2019:7:15 14:33:16 
//**********************************(C)COPYRIGHT 2019 FFCY Team*********************************************/
u8 VL53L0X_Getinit_OK()
{
	u8 VL53l0_Revision_ID = 0;
	u8 VL53l0_Device_ID = 0;

	//读取传感器ID值
	VL_IIC_Read_1Byte(VL53L0X_Address, VL53L0X_REG_IDENTIFICATION_REVISION_ID, &VL53l0_Revision_ID);
	VL_IIC_Read_1Byte(VL53L0X_Address, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &VL53l0_Device_ID);

	if (VL53l0_Revision_ID == 0x10 && VL53l0_Device_ID== 0xee)
		return 1;
	else
		return 0;

}
//**************************飞**********凡*********创**********翼******************************************
//*	函 数 名 :  VL53L1_Getinit_OK()
//*	功能说明 :  检测VL53L1 是否在线
//*	形    参 ：无
//*	返 回 值 :  1  成功
//*             0  失败
//* 作者    ：FFCY
//* 最后更改：CP_SYD 孙艺东 2019:7:15 14:34:10 
//**********************************(C)COPYRIGHT 2019 FFCY Team*********************************************/
u8  VL53L1_Getinit_OK()
{
	u16 VL53l1_ID = 0;
	
	//等待验证产品ID
	VL53l1_ID = VL_IIC_Read_2ByteQ(IDENTIFICATION__MODEL_ID);
	if (VL53l1_ID == 0xEACC)
		return 1;  //检测到VL53L1
	

	return 0;
}
#define VL53L0X   1
#define VL53L1X   2
#define VL53LX_Type VL53L1X

//**************************飞**********凡*********创**********翼******************************************
//*	函 数 名 :  u8 VL53LX_Check_type()
//*	功能说明 :  检测Vl53LX的类型 L0/L1
//*	形    参 ：无
//*	返 回 值 : 0 都不存在
//*            1 VL53L0
//*            2 VL53L1
//* 作者    ：FFCY
//* 最后更改：CP_SYD 孙艺东 2019:7:15 14:41:17 
//**********************************(C)COPYRIGHT 2019 FFCY Team*********************************************/
u8 VL53LX_Check_type()
{
//#if( VL53LX_Type == VL53L1X)
//	if (VL53L1_Getinit_OK() == 1)	return 2;
//#endif
//#if( VL53LX_Type == VL53L0X)
//	if (VL53L0X_Getinit_OK() == 1)	return 1;
//#endif
//	return 0;

	if (VL53L0X_Getinit_OK() == 1)	return 1;
	if (VL53L1_Getinit_OK() == 1)	return 2;
	return 0;

}

//**************************飞**********凡*********创**********翼******************************************
//*	函 数 名 : u16 VL53L0X_Get_Distance_Data()
//*	功能说明 : 读取VL53L0X数据
//*	形    参 ：无
//*	返 回 值 : 无
//* 作者    ：FFCY
//* 最后更改：CP_SYD 孙艺东 2019:7:15 14:54:36 
//**********************************(C)COPYRIGHT 2019 FFCY Team*********************************************/
u16 VL53L0X_Get_Distance_Data()
{
	u8 val = 0;
	//开始测距
	VL_IIC_Write_1Byte(VL53L0X_Address, VL53L0X_REG_SYSRANGE_START, 0x01);
	//等待测距完成
	while (!(val & 0x01))
		VL_IIC_Read_1Byte(VL53L0X_Address, VL53L0X_REG_RESULT_RANGE_STATUS, &val);
	//读取数据
	VL_IIC_Read_nByte(VL53L0X_Address, 0x14, 12, VL53L0X_Read_buff);

	//解压数据 并更新
	VL53L0X_Distance_data = makeuint16(VL53L0X_Read_buff[11], VL53L0X_Read_buff[10]);

	return VL53L0X_Distance_data;
}

//**************************飞**********凡*********创**********翼******************************************
//*	函 数 名 :  void VL53L0X_Get_Distance_Data()
//*	功能说明 : 读取LV53L1X测量数据
//*	形    参 ：无
//*	返 回 值 : 无
//* 作者    ：FFCY
//* 最后更改：CP_SYD 孙艺东 2019:7:15 14:15:00 
//**********************************(C)COPYRIGHT 2019 FFCY Team*********************************************/
u32 VL53L1X_Get_Distance_Data()
{
	//取寄存器的值
	u16 FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 = VL_IIC_Read_2ByteQ(RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0);
	// 转化
	VL53L0X_Distance_data = ((uint32_t)FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 * 2011 + 0x0400) / 0x0800;
	//清中断标志寄存器
	VL_IIC_Write_16address_Byte(VL53L1X_Address, SYSTEM__INTERRUPT_CLEAR, 0x01); // sys_interrupt_clear_rang

	return VL53L0X_Distance_data;
}




//**************************飞**********凡*********创**********翼******************************************
//*	函 数 名 :  void VL53LX_Data_clear()
//*	功能说明 :  清空变量
//*	形    参 ：无
//*	返 回 值 : 无
//* 作者    ：FFCY
//* 最后更改：CP_SYD 孙艺东 2019:7:15 15:13:57 
//**********************************(C)COPYRIGHT 2019 FFCY Team*********************************************/
void VL53LX_Data_clear()
{
	VL53Lx_Data.Sensor_type = 0;
	VL53Lx_Data.Is_OK = 0;
	VL53Lx_Data.Distance = 0;
}
//**************************飞**********凡*********创**********翼******************************************
//*	函 数 名 :  u8 VL53L0_init()
//*	功能说明 :  VL53L0初始化
//*	形    参 ：无
//*	返 回 值 : 无
//* 作者    ：FFCY
//* 最后更改：CP_SYD 孙艺东 2019:7:15 15:24:54 
//**********************************(C)COPYRIGHT 2019 FFCY Team*********************************************/
u8 VL53L0_init()
{
	VL53Lx_Data.Is_OK = 1;
}
//**************************飞**********凡*********创**********翼******************************************
//*	函 数 名 :  u8  VL53L1_init()
//*	功能说明 :  VL53L1初始化
//*	形    参 ：无
//*	返 回 值 : 无
//* 作者    ：FFCY
//* 最后更改：CP_SYD 孙艺东 2019:7:15 15:25:13 
//**********************************(C)COPYRIGHT 2019 FFCY Team*********************************************/
u8  VL53L1_init()
{
	
	//发送关机指令
	VL_IIC_Write_16address_Byte(VL53L1X_Address, SOFT_RESET, 0x00);
	delay_ms(50);
	//发送开机指令
	VL_IIC_Write_16address_Byte(VL53L1X_Address, SOFT_RESET, 0x01);
	delay_ms(50);
	//读系统时间
	 u16 osc_calibrate_val = VL_IIC_Read_2ByteQ(RESULT__OSC_CALIBRATE_VAL);
	//设置连续测量模式
     VL53L1X_startContinuous(50, osc_calibrate_val);

	 VL53Lx_Data.Is_OK = 1;
}

//**************************飞**********凡*********创**********翼******************************************
//*	函 数 名 :  void VL53LX_init()
//*	功能说明 :  总初始化
//*	形    参 ：无
//*	返 回 值 : 无
//* 作者    ：FFCY
//* 最后更改：CP_SYD 孙艺东 2019:7:15 15:25:39 
//**********************************(C)COPYRIGHT 2019 FFCY Team*********************************************/
u8 VL53LX_init()
{
	//初始化IIC 引脚
	VL53LX_I2C_init();
	//清空变量
	VL53LX_Data_clear();

	delay_ms(500);

	VL53Lx_Data.Sensor_type = VL53LX_Check_type();
    //VL53Lx_Data.Sensor_type = 2;
	if (VL53Lx_Data.Sensor_type == 0)   return 0;
	else if (VL53Lx_Data.Sensor_type == 1)  VL53L0_init();
	else if (VL53Lx_Data.Sensor_type == 2)  VL53L1_init();

	
	return 1;
}

u32 Distance_HIGH_Last = 0;
void VL53LX_Statemachine()
{
	u32 Distance_HIGH = 0;
	
	//if (VL53Lx_Data.Is_OK == 0)		return;							//检查激光测距是否在线
		
	if (VL53Lx_Data.Sensor_type == 1)
		VL53Lx_Data.Distance = (u32)VL53L0X_Get_Distance_Data();
	else if (VL53Lx_Data.Sensor_type == 2)
		VL53Lx_Data.Distance = VL53L1X_Get_Distance_Data();

	//去掉不正常的值
	if(VL53Lx_Data.Distance == 20)	VL53Lx_Data.Distance = Distance_HIGH_Last;
	if(VL53Lx_Data.Distance ==8190)	VL53Lx_Data.Distance = Distance_HIGH_Last;
	
	Distance_HIGH_Last = VL53Lx_Data.Distance;						//更新旧值
}
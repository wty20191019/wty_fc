//#include "Headfile.h"
//#include "sys.h"
//#include "string.h"
//#include "oled.h"
//#include "stdlib.h"
//#include "oledfont.h"
//#include "stdio.h"
//#include "delay.h"
//
//
//
////-----------------OLED端口定义---------------- 
//
//
//#define OLED_SCL_Clr() GPIO_ResetBits(GPIOC,GPIO_Pin_0)//SCL
//#define OLED_SCL_Set() GPIO_SetBits(GPIOC,GPIO_Pin_0)
//
//#define OLED_SDA_Clr() GPIO_ResetBits(GPIOC,GPIO_Pin_1)//DIN
//#define OLED_SDA_Set() GPIO_SetBits(GPIOC,GPIO_Pin_1)
//
//#define OLED_RES_Clr() GPIO_ResetBits(GPIOD,GPIO_Pin_4)//RES
//#define OLED_RES_Set() GPIO_SetBits(GPIOD,GPIO_Pin_4)
//
//
//#define OLED_CMD  0	//写命令
//#define OLED_DATA 1	//写数据
//
//u8 OLED_GRAM[144][8];
//
//
////延时
//void Oled_IIC_delay(void)
//{
//	u8 t = 10;
//	while (t--);
//}
//
////起始信号
//void Oled_I2C_Start(void)
//{
//	OLED_SDA_Set();
//	OLED_SCL_Set();
//	Oled_IIC_delay();
//	OLED_SDA_Clr();
//	Oled_IIC_delay();
//	OLED_SCL_Clr();
//	Oled_IIC_delay();
//}
//
////结束信号
//void Oled_I2C_Stop(void)
//{
//	OLED_SDA_Clr();
//	OLED_SCL_Set();
//	Oled_IIC_delay();
//	OLED_SDA_Set();
//}
//
////等待信号响应
//void Oled_I2C_WaitAck(void) //测数据信号的电平
//{
//	OLED_SDA_Set();
//	Oled_IIC_delay();
//	OLED_SCL_Set();
//	Oled_IIC_delay();
//	OLED_SCL_Clr();
//	Oled_IIC_delay();
//}
//
////写入一个字节
//void Oled_Send_Byte(u8 dat)
//{
//	u8 i;
//	for (i = 0; i < 8; i++)
//	{
//		if (dat & 0x80)//将dat的8位从最高位依次写入
//		{
//			OLED_SDA_Set();
//		}
//		else
//		{
//			OLED_SDA_Clr();
//		}
//		Oled_IIC_delay();
//		OLED_SCL_Set();
//		Oled_IIC_delay();
//		OLED_SCL_Clr();//将时钟信号设置为低电平
//		dat <<= 1;
//	}
//}
//
////发送一个字节
////mode:数据/命令标志 0,表示命令;1,表示数据;
//void OLED_WR_Byte(u8 dat, u8 mode)
//{
//	I2C_Start();
//	Oled_Send_Byte(0x78);
//	Oled_I2C_WaitAck();
//	if (mode) { Oled_Send_Byte(0x40); }
//	else { Oled_Send_Byte(0x00); }
//	Oled_I2C_WaitAck();
//	Oled_Send_Byte(dat);
//	Oled_I2C_WaitAck();
//	Oled_I2C_Stop();
//}
////反显函数
//void OLED_ColorTurn(u8 i)
//{
//	if (i == 0)
//	{
//		OLED_WR_Byte(0xA6, OLED_CMD);//正常显示
//	}
//	if (i == 1)
//	{
//		OLED_WR_Byte(0xA7, OLED_CMD);//反色显示
//	}
//}
//
////屏幕旋转180度
//void OLED_DisplayTurn(u8 i)
//{
//	if (i == 0)
//	{
//		OLED_WR_Byte(0xC8, OLED_CMD);//正常显示
//		OLED_WR_Byte(0xA1, OLED_CMD);
//	}
//	if (i == 1)
//	{
//		OLED_WR_Byte(0xC0, OLED_CMD);//反转显示
//		OLED_WR_Byte(0xA0, OLED_CMD);
//	}
//}
//
//
////开启OLED显示 
//void OLED_DisPlay_On(void)
//{
//	OLED_WR_Byte(0x8D, OLED_CMD);//电荷泵使能
//	OLED_WR_Byte(0x14, OLED_CMD);//开启电荷泵
//	OLED_WR_Byte(0xAF, OLED_CMD);//点亮屏幕
//}
//
////关闭OLED显示 
//void OLED_DisPlay_Off(void)
//{
//	OLED_WR_Byte(0x8D, OLED_CMD);//电荷泵使能
//	OLED_WR_Byte(0x10, OLED_CMD);//关闭电荷泵
//	OLED_WR_Byte(0xAE, OLED_CMD);//关闭屏幕
//}
//
////更新显存到OLED	
//void OLED_Refresh(void)
//{
//	u8 i, n;
//	for (i = 0; i < 8; i++)
//	{
//		OLED_WR_Byte(0xb0 + i, OLED_CMD); //设置行起始地址
//		OLED_WR_Byte(0x00, OLED_CMD);   //设置低列起始地址
//		OLED_WR_Byte(0x10, OLED_CMD);   //设置高列起始地址
//		I2C_Start();
//		Oled_Send_Byte(0x78);
//		Oled_I2C_WaitAck();
//		Oled_Send_Byte(0x40);
//		Oled_I2C_WaitAck();
//		for (n = 0; n < 128; n++)
//		{
//			Oled_Send_Byte(OLED_GRAM[n][i]);
//			Oled_I2C_WaitAck();
//		}
//		I2C_Stop();
//	}
//}
////清屏函数
//void OLED_Clear(void)
//{
//	u8 i, n;
//	for (i = 0; i < 8; i++)
//	{
//		for (n = 0; n < 128; n++)
//		{
//			OLED_GRAM[n][i] = 0;//清除所有数据
//		}
//	}
//	OLED_Refresh();//更新显示
//}
//
////画点 
////x:0~127
////y:0~63
////t:1 填充 0,清空	
//void OLED_DrawPoint(u8 x, u8 y, u8 t)
//{
//	u8 i, m, n;
//	i = y / 8;
//	m = y % 8;
//	n = 1 << m;
//	if (t) { OLED_GRAM[x][i] |= n; }
//	else
//	{
//		OLED_GRAM[x][i] = ~OLED_GRAM[x][i];
//		OLED_GRAM[x][i] |= n;
//		OLED_GRAM[x][i] = ~OLED_GRAM[x][i];
//	}
//}
//
////画线
////x1,y1:起点坐标
////x2,y2:结束坐标
//void OLED_DrawLine(u8 x1, u8 y1, u8 x2, u8 y2, u8 mode)
//{
//	u16 t;
//	int xerr = 0, yerr = 0, delta_x, delta_y, distance;
//	int incx, incy, uRow, uCol;
//	delta_x = x2 - x1; //计算坐标增量 
//	delta_y = y2 - y1;
//	uRow = x1;//画线起点坐标
//	uCol = y1;
//	if (delta_x>0)incx = 1; //设置单步方向 
//	else if (delta_x == 0)incx = 0;//垂直线 
//	else { incx = -1; delta_x = -delta_x; }
//	if (delta_y>0)incy = 1;
//	else if (delta_y == 0)incy = 0;//水平线 
//	else { incy = -1; delta_y = -delta_x; }
//	if (delta_x>delta_y)distance = delta_x; //选取基本增量坐标轴 
//	else distance = delta_y;
//	for (t = 0; t<distance + 1; t++)
//	{
//		OLED_DrawPoint(uRow, uCol, mode);//画点
//		xerr += delta_x;
//		yerr += delta_y;
//		if (xerr>distance)
//		{
//			xerr -= distance;
//			uRow += incx;
//		}
//		if (yerr>distance)
//		{
//			yerr -= distance;
//			uCol += incy;
//		}
//	}
//}
////x,y:圆心坐标
////r:圆的半径
//void OLED_DrawCircle(u8 x, u8 y, u8 r)
//{
//	int a, b, num;
//	a = 0;
//	b = r;
//	while (2 * b * b >= r * r)
//	{
//		OLED_DrawPoint(x + a, y - b, 1);
//		OLED_DrawPoint(x - a, y - b, 1);
//		OLED_DrawPoint(x - a, y + b, 1);
//		OLED_DrawPoint(x + a, y + b, 1);
//
//		OLED_DrawPoint(x + b, y + a, 1);
//		OLED_DrawPoint(x + b, y - a, 1);
//		OLED_DrawPoint(x - b, y - a, 1);
//		OLED_DrawPoint(x - b, y + a, 1);
//
//		a++;
//		num = (a * a + b * b) - r*r;//计算画的点离圆心的距离
//		if (num > 0)
//		{
//			b--;
//			a--;
//		}
//	}
//}
//
//
//
////在指定位置显示一个字符,包括部分字符
////x:0~127
////y:0~63
////size1:选择字体 6x8/6x12/8x16/12x24
////mode:0,反色显示;1,正常显示
//void OLED_ShowChar(u8 x, u8 y, u8 chr, u8 size1, u8 mode)
//{
//	u8 i, m, temp, size2, chr1;
//	u8 x0 = x, y0 = y;
//	if (size1 == 8)size2 = 6;
//	else size2 = (size1 / 8 + ((size1 % 8) ? 1 : 0))*(size1 / 2);  //得到字体一个字符对应点阵集所占的字节数
//	chr1 = chr - ' ';  //计算偏移后的值
//	for (i = 0; i<size2; i++)
//	{
//		if (size1 == 8)
//		{
//			temp = asc2_0806[chr1][i];
//		} //调用0806字体
//		else if (size1 == 12)
//		{
//			temp = asc2_1206[chr1][i];
//		} //调用1206字体
//		else if (size1 == 16)
//		{
//			temp = asc2_1608[chr1][i];
//		} //调用1608字体
//		else if (size1 == 24)
//		{
//			temp = asc2_2412[chr1][i];
//		} //调用2412字体
//		else return;
//		for (m = 0; m<8; m++)
//		{
//			if (temp & 0x01)OLED_DrawPoint(x, y, mode);
//			else OLED_DrawPoint(x, y, !mode);
//			temp >>= 1;
//			y++;
//		}
//		x++;
//		if ((size1 != 8) && ((x - x0) == size1 / 2))
//		{
//			x = x0; y0 = y0 + 8;
//		}
//		y = y0;
//	}
//}
//
//
////显示字符串
////x,y:起点坐标  
////size1:字体大小 
////*chr:字符串起始地址 
////mode:0,反色显示;1,正常显示
//void OLED_ShowString(u8 x, u8 y, u8 *chr, u8 size1, u8 mode)
//{
//	while ((*chr >= ' ') && (*chr <= '~'))//判断是不是非法字符!
//	{
//		OLED_ShowChar(x, y, *chr, size1, mode);
//		if (size1 == 8)x += 6;
//		else x += size1 / 2;
//		chr++;
//	}
//}
//
////m^n
//u32 OLED_Pow(u8 m, u8 n)
//{
//	u32 result = 1;
//	while (n--)
//	{
//		result *= m;
//	}
//	return result;
//}
//
////显示数字
////x,y :起点坐标
////num :要显示的数字
////len :数字的位数
////size:字体大小
////mode:0,反色显示;1,正常显示
//void OLED_ShowNum(u8 x, u8 y, u32 num, u8 len, u8 size1, u8 mode)
//{
//	u8 t, temp, m = 0;
//	if (size1 == 8)m = 2;
//	for (t = 0; t<len; t++)
//	{
//		temp = (num / OLED_Pow(10, len - t - 1)) % 10;
//		if (temp == 0)
//		{
//			OLED_ShowChar(x + (size1 / 2 + m)*t, y, '0', size1, mode);
//		}
//		else
//		{
//			OLED_ShowChar(x + (size1 / 2 + m)*t, y, temp + '0', size1, mode);
//		}
//	}
//}
//
////显示汉字
////x,y:起点坐标
////num:汉字对应的序号
////mode:0,反色显示;1,正常显示
//void OLED_ShowChinese(u8 x, u8 y, u8 num, u8 size1, u8 mode)
//{
//	u8 m, temp;
//	u8 x0 = x, y0 = y;
//	u16 i, size3 = (size1 / 8 + ((size1 % 8) ? 1 : 0))*size1;  //得到字体一个字符对应点阵集所占的字节数
//	for (i = 0; i<size3; i++)
//	{
//		if (size1 == 16)
//		{
//			temp = Hzk1[num][i];
//		}//调用16*16字体
//		else if (size1 == 24)
//		{
//			temp = Hzk2[num][i];
//		}//调用24*24字体
//		else if (size1 == 32)
//		{
//			temp = Hzk3[num][i];
//		}//调用32*32字体
//		else if (size1 == 64)
//		{
//			temp = Hzk4[num][i];
//		}//调用64*64字体
//		else return;
//		for (m = 0; m<8; m++)
//		{
//			if (temp & 0x01)OLED_DrawPoint(x, y, mode);
//			else OLED_DrawPoint(x, y, !mode);
//			temp >>= 1;
//			y++;
//		}
//		x++;
//		if ((x - x0) == size1)
//		{
//			x = x0; y0 = y0 + 8;
//		}
//		y = y0;
//	}
//}
//
////num 显示汉字的个数
////space 每一遍显示的间隔
////mode:0,反色显示;1,正常显示
//void OLED_ScrollDisplay(u8 num, u8 space, u8 mode)
//{
//	u8 i, n, t = 0, m = 0, r;
//	while (1)
//	{
//		if (m == 0)
//		{
//			OLED_ShowChinese(128, 24, t, 16, mode); //写入一个汉字保存在OLED_GRAM[][]数组中
//			t++;
//		}
//		if (t == num)
//		{
//			for (r = 0; r<16 * space; r++)      //显示间隔
//			{
//				for (i = 1; i<144; i++)
//				{
//					for (n = 0; n<8; n++)
//					{
//						OLED_GRAM[i - 1][n] = OLED_GRAM[i][n];
//					}
//				}
//				OLED_Refresh();
//			}
//			t = 0;
//		}
//		m++;
//		if (m == 16) { m = 0; }
//		for (i = 1; i<144; i++)   //实现左移
//		{
//			for (n = 0; n<8; n++)
//			{
//				OLED_GRAM[i - 1][n] = OLED_GRAM[i][n];
//			}
//		}
//		OLED_Refresh();
//	}
//}
//
////x,y：起点坐标
////sizex,sizey,图片长宽
////BMP[]：要写入的图片数组
////mode:0,反色显示;1,正常显示
//void OLED_ShowPicture(u8 x, u8 y, u8 sizex, u8 sizey, u8 BMP[], u8 mode)
//{
//	u16 j = 0;
//	u8 i, n, temp, m;
//	u8 x0 = x, y0 = y;
//	sizey = sizey / 8 + ((sizey % 8) ? 1 : 0);
//	for (n = 0; n<sizey; n++)
//	{
//		for (i = 0; i<sizex; i++)
//		{
//			temp = BMP[j];
//			j++;
//			for (m = 0; m<8; m++)
//			{
//				if (temp & 0x01)OLED_DrawPoint(x, y, mode);
//				else OLED_DrawPoint(x, y, !mode);
//				temp >>= 1;
//				y++;
//			}
//			x++;
//			if ((x - x0) == sizex)
//			{
//				x = x0;
//				y0 = y0 + 8;
//			}
//			y = y0;
//		}
//	}
//}
//
//u8 t = ' ';
///***************************************************
//函数名: void OLED_Init(void)
//说明:	OLED显示屏初始化
//入口:	无
//出口:	无
//备注:	上电初始化，运行一次
//****************************************************/
//void OLED_Init(void)
//{
//	GPIO_InitTypeDef  GPIO_InitStructure;
//
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOG, ENABLE);//使能PORTA~E,PORTG时钟
//
//																				//GPIO初始化设置
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;//推挽输出
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
//	GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化
//
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
//	GPIO_Init(GPIOD, &GPIO_InitStructure);//初始化
//
//										  //GPIO初始化设置
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;//推挽输出
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
//	GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化
//
//
//	delay_ms(200);
//
//
//	OLED_WR_Byte(0xAE, OLED_CMD);//--turn off oled panel
//	OLED_WR_Byte(0x00, OLED_CMD);//---set low column address
//	OLED_WR_Byte(0x10, OLED_CMD);//---set high column address
//	OLED_WR_Byte(0x40, OLED_CMD);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
//	OLED_WR_Byte(0x81, OLED_CMD);//--set contrast control register
//	OLED_WR_Byte(0xCF, OLED_CMD);// Set SEG Output Current Brightness
//	OLED_WR_Byte(0xA1, OLED_CMD);//--Set SEG/Column Mapping     0xa0左右反置 0xa1正常
//	OLED_WR_Byte(0xC8, OLED_CMD);//Set COM/Row Scan Direction   0xc0上下反置 0xc8正常
//	OLED_WR_Byte(0xA6, OLED_CMD);//--set normal display
//	OLED_WR_Byte(0xA8, OLED_CMD);//--set multiplex ratio(1 to 64)
//	OLED_WR_Byte(0x3f, OLED_CMD);//--1/64 duty
//	OLED_WR_Byte(0xD3, OLED_CMD);//-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
//	OLED_WR_Byte(0x00, OLED_CMD);//-not offset
//	OLED_WR_Byte(0xd5, OLED_CMD);//--set display clock divide ratio/oscillator frequency
//	OLED_WR_Byte(0x80, OLED_CMD);//--set divide ratio, Set Clock as 100 Frames/Sec
//	OLED_WR_Byte(0xD9, OLED_CMD);//--set pre-charge period
//	OLED_WR_Byte(0xF1, OLED_CMD);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
//	OLED_WR_Byte(0xDA, OLED_CMD);//--set com pins hardware configuration
//	OLED_WR_Byte(0x12, OLED_CMD);
//	OLED_WR_Byte(0xDB, OLED_CMD);//--set vcomh
//	OLED_WR_Byte(0x40, OLED_CMD);//Set VCOM Deselect Level
//	OLED_WR_Byte(0x20, OLED_CMD);//-Set Page Addressing Mode (0x00/0x01/0x02)
//	OLED_WR_Byte(0x02, OLED_CMD);//
//	OLED_WR_Byte(0x8D, OLED_CMD);//--set Charge Pump enable/disable
//	OLED_WR_Byte(0x14, OLED_CMD);//--set(0x10) disable
//	OLED_WR_Byte(0xA4, OLED_CMD);// Disable Entire Display On (0xa4/0xa5)
//	OLED_WR_Byte(0xA6, OLED_CMD);// Disable Inverse Display On (0xa6/a7) 
//	OLED_Clear();
//	OLED_WR_Byte(0xAF, OLED_CMD);
//
//	while (1)
//	{
//		//OLED_ShowPicture(0, 0, 128, 64, BMP1, 1);
//		OLED_Refresh();
//		delay_ms(500);
//		OLED_Clear();
//		OLED_ShowChinese(0, 0, 0, 16, 1);//中
//		OLED_ShowChinese(18, 0, 1, 16, 1);//景
//		OLED_ShowChinese(36, 0, 2, 16, 1);//园
//		OLED_ShowChinese(54, 0, 3, 16, 1);//电
//		OLED_ShowChinese(72, 0, 4, 16, 1);//子
//		OLED_ShowChinese(90, 0, 5, 16, 1);//技
//		OLED_ShowChinese(108, 0, 6, 16, 1);//术
//		OLED_ShowString(8, 16, "ZHONGJINGYUAN", 16, 1);
//		OLED_ShowString(20, 32, "2014/05/01", 16, 1);
//		OLED_ShowString(0, 48, "ASCII:", 16, 1);
//		OLED_ShowString(63, 48, "CODE:", 16, 1);
//		OLED_ShowChar(48, 48, t, 16, 1);//显示ASCII字符	   
//		t++;
//		if (t > '~')t = ' ';
//		OLED_ShowNum(103, 48, t, 3, 16, 1);
//		OLED_Refresh();
//		delay_ms(500);
//		OLED_Clear();
//		OLED_ShowChinese(0, 0, 0, 16, 1);  //16*16 中
//		OLED_ShowChinese(16, 0, 0, 24, 1); //24*24 中
//		OLED_ShowChinese(24, 20, 0, 32, 1);//32*32 中
//		OLED_ShowChinese(64, 0, 0, 64, 1); //64*64 中
//		OLED_Refresh();
//		delay_ms(500);
//		OLED_Clear();
//		OLED_ShowString(0, 0, "ABC", 8, 1);//6*8 “ABC”
//		OLED_ShowString(0, 8, "ABC", 12, 1);//6*12 “ABC”
//		OLED_ShowString(0, 20, "ABC", 16, 1);//8*16 “ABC”
//		OLED_ShowString(0, 36, "ABC", 24, 1);//12*24 “ABC”
//		OLED_Refresh();
//		delay_ms(500);
//		//OLED_ScrollDisplay(11, 4, 1);
//		LED1 = !LED1;
//	}
//
//}
//
///*********************OLED全屏************************************/
//void OLED_Fill(unsigned char bmp_dat)
//{
//
//}
///*********************OLED复位************************************/
//void OLED_CLS(void)
//{
//
//}
//
//
//void LCD_P8x16Str(unsigned char x, unsigned char  y, unsigned char ch[])
//{
//}
//void LCD_CLS(void)
//{
//}
///*------显示6X8一组标准的ASCII字符串，显示坐标为（x，y）------*/
//void LCD_P6x8Str(unsigned char x, unsigned char  y, unsigned char ch[])
//{
//
//}
////显示一个6X8的字符
//void LCD_P6x8Char(unsigned char x, unsigned char  y, unsigned char ucData)
//{
//
//}
//
//char Print_buff[20];
//
//void write_6_8_number(unsigned char x, unsigned char y, float number)
//{
//
//
//}
//
//
//void LCD_clear_L(unsigned char x, unsigned char y)
//{
//
//}
//
//

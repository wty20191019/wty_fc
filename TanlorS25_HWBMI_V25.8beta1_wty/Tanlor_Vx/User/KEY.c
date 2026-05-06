#include "Headfile.h"
#include "Key.h"

u8 AUTO_FLY_FLAG = 0;

int16_t Page_Number=0;//2
int My_page_number = 0;

/***************************************************
函数名: void Key_Init(void)
说明:	按键初始化
入口:	无
出口:	无
备注:	上电初始化，运行一次
****************************************************/
void Key_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB2PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_1 | GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOE2,3,4


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_5;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/***************************************************
函数名: void Key_Scan()
说明:	按键扫描程序，显示屏切换翻页
入口:	无
出口:	无
备注:	在主函数While(1)里面，利用中断空隙时间一直运行
****************************************************/
uint8_t Key_Right_Release=0;
extern  int change_time_flag ;
extern struct Key_input key;
int key_Fly_cnt = 0;

extern int My_page_number ;
int My_page_number_last = 0;
u8 Key_Scan(uint8_t release)
{
	if (change_time_flag == 1)
		return FALSE;

	key.key_del = 0;
	key.key_down = 0;
	key.Key_plus = 0;
	key.Key_up = 0;

	if (Key_Up == 1)
	{
		delay_ms(80);
		if (Key_Up == 1)
		{
			key.Key_up = 1;
		}
	}

	if (Key_Down == 1)
	{
		delay_ms(80);
		if (Key_Down == 1)
		{
			key.key_down = 1;
			key_Fly_cnt++;
		}
	}
	else
	{
		key_Fly_cnt = 0;
	}
	if (My_page_number_last == My_page_number)
	{
		key_Fly_cnt = 0;
	}
	My_page_number_last = My_page_number;

	if (key_Fly_cnt > 20)
	{
		AUTO_FLY_FLAG = 1;
	}
	if (Key_Plus == 1)
	{
		delay_ms(80);
		if (Key_Plus == 1)
		{
			key.Key_plus = 1;
		}
	}
	if (Key_Del == 1)
	{
		delay_ms(80);
		if (Key_Del == 1)
		{
 			key.key_del = 1;
		}
	}
	if (Key_Plus2 == 1)
	{
		delay_ms(80);
		if (Key_Plus2 == 1)
		{
			key.Key_plus = 1;
		}
	}
	return TRUE;
}


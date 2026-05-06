
#include "Headfile.h"
#include "Bling.h"

/***************************************************
函数名: void Bling_Init(void)
说明:	LED状态指示灯初始化
入口:	无
出口:	无
备注:	上电初始化，运行一次
****************************************************/
void Bling_Init()
{  
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//使能GPIOF时钟

	//状态灯
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 ;//状态指示灯
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIO
	GPIO_SetBits(GPIOC, GPIO_Pin_4);

	//蜂鸣器和激光点
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8| GPIO_Pin_9;//状态指示灯
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIO
	GPIO_ResetBits(GPIOC, GPIO_Pin_8 | GPIO_Pin_9);

	//LED灯
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;//状态指示灯
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIO
	GPIO_ResetBits(GPIOB, GPIO_Pin_4);

}

u8 LASER1_flag = 0; int LASER1_times = 0;
u8 BEEP_flag = 0; int BEEP_times = 0;
u8 Anm_Led_flag = 0; int Anm_Led_times = 0;
u8 tiaoshi = 1;
extern u8 T265_Confidence;

void Led_Statemachine()
{

	if (LASER1_flag == 1)
	{
		LASER1_times++;	LASER1 = 0;
		if (LASER1_times >= 80) { LASER1 = 1; LASER1_flag = 0; LASER1_times = 0; }
	}
	else if (LASER1_flag == 2)
	{
		LASER1 = 0;
	}
	else if (LASER1_flag == 3)
	{
		LASER1_times++;
		if (LASER1_times >= 20) 
		{ 
			LASER1 = !LASER1;
			LASER1_times = 0; 
		}
	}
	else
	{
		LASER1 = 1;
	}

	if (BEEP_flag == 1)
	{
		BEEP_times++;	BEEP = 1;
		if (BEEP_times >= 50) { BEEP = 0; BEEP_flag = 0; BEEP_times = 0; }
	}


	if (Anm_Led_flag == 1)
	{
		Anm_Led_times++;
		if (Anm_Led_times >= 30)
		{
			Anm_Led  = !Anm_Led;
			Anm_Led_times = 0;
		}
	}
	else
	{
		Anm_Led = 0;
	}
}

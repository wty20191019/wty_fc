
#include "stm32f4xx.h"
#include "ppm_input.h"
#include <string.h>


uint16 PPM_Sample_Cnt = 0;
uint16 PPM_Isr_Cnt = 0;
u32 Last_PPM_Time = 0;
u32 PPM_Time = 0;
u16 PPM_Time_Delta = 0;
u16 PPM_Time_Max = 0;
uint16 PPM_Start_Time = 0;
uint16 PPM_Finished_Time = 0;
uint16 PPM_Is_Okay = 0;
uint16 PPM_Databuf[10] = { 0 };
u32 TIME_ISR_CNT = 0;

/***************************************************
函数名: void PPM_GPIO_Init(void)
说明:    PPM输入引脚初始化
入口:    无
出口:    无
备注:    PA8/EXTI8，可按宏定义迁移到其他引脚
****************************************************/
void PPM_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(PPM_GPIO_CLK, ENABLE);
	//==================================
	//PPM输入引脚配置为上拉输入，使用外部中断捕获上升沿
	//==================================
	GPIO_InitStructure.GPIO_Pin = PPM_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(PPM_GPIO_PORT, &GPIO_InitStructure);
}

/***************************************************
函数名: void TIM4_Configuration_Cnt(void)
说明:    TIM4时间基准初始化
入口:    无
出口:    无
备注:    供PPM解析和其他时间测量共用
****************************************************/
void TIM4_Configuration_Cnt(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	TIM_DeInit(TIM4);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);
	//==================================
	//TIM4_CH3用于PPM时间测量，其他通道未使用
	//==================================
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure); //TIM4_CH3

	TIM_TimeBaseStructure.TIM_Period = 10000;
	TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM4, ENABLE);
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
}

/***************************************************
函数名: void TIM4_IRQHandler(void)
说明:    TIM4溢出中断
入口:    无
出口:    无
备注:    提供PPM采样所需的TIME_ISR_CNT
****************************************************/
void TIM4_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
		TIME_ISR_CNT++;
	}
}

/***************************************************
函数名: void PPM_Init(void)
说明:    PPM接收初始化
入口:    无
出口:    无
备注:    上电初始化，运行一次
****************************************************/
void PPM_Init()
{
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;

	PPM_GPIO_Init();
	TIM4_Configuration_Cnt();

	PPM_Sample_Cnt = 0;
	PPM_Isr_Cnt = 0;
	Last_PPM_Time = 0;
	PPM_Time = 0;
	PPM_Time_Delta = 0;
	PPM_Time_Max = 0;
	PPM_Start_Time = 0;
	PPM_Finished_Time = 0;
	PPM_Is_Okay = 0;
	TIME_ISR_CNT = 0;
	memset(PPM_Databuf, 0, sizeof(PPM_Databuf));

	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x04;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE); //使能SYSCFG时钟
	SYSCFG_EXTILineConfig(PPM_EXTI_PORTSOURCE, PPM_EXTI_PINSOURCE); //PA8连接到中断线8

	EXTI_InitStructure.EXTI_Line = PPM_EXTI_LINE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

}

/***************************************************
函数名: void EXTI9_5_IRQHandler(void)
说明:    PPM接收中断函数
入口:    无
出口:    无
备注:    程序初始化后、始终运行
****************************************************/
static uint16 PPM_buf[10] = { 0 };
void EXTI9_5_IRQHandler(void)
{
	if (EXTI_GetITStatus(PPM_EXTI_LINE) != RESET)
	{
		EXTI_ClearITPendingBit(PPM_EXTI_LINE);
		//系统运行时间获取，单位us
		Last_PPM_Time = PPM_Time;
		PPM_Time = 10000*TIME_ISR_CNT + TIM4->CNT;
		PPM_Time_Delta = PPM_Time - Last_PPM_Time;
		//PPM中断进入判断
		if (PPM_Isr_Cnt < 100)  PPM_Isr_Cnt++;
		//PPM解析开始
		if (PPM_Is_Okay == 1)
		{
			if (PPM_Time_Delta >= 800&&PPM_Time_Delta <= 2200)
			{
				PPM_Sample_Cnt++;
				//对应通道写入缓冲区
				PPM_buf[PPM_Sample_Cnt - 1] = PPM_Time_Delta;
				//单次解析结束
				if (PPM_Sample_Cnt >= 10)
				{
					memcpy(PPM_Databuf, PPM_buf, PPM_Sample_Cnt * sizeof(uint16));
					PPM_Is_Okay = 0;
				}
			}
			else
			{
				if (PPM_Time_Delta >= 3000)//帧结束电平至少2ms=2000us，由于部分老版本遥控器、
				  //接收机输出PPM信号不标准，当出现解析异常时，尝试改小此值，该情况仅出现一例：使用天地飞老版本遥控器
				{
					PPM_Is_Okay = 1;
					PPM_Sample_Cnt = 0;
				}
				else  PPM_Is_Okay = 0;
			}
		}
		else if (PPM_Time_Delta >= 2500)//帧结束电平至少2ms=2000us
		{
			PPM_Is_Okay = 1;
			PPM_Sample_Cnt = 0;
		}
	}
 
}




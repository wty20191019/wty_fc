/*----------------------------------------------------------------------------------------------------------------------/
*               本程序版权著作权属于中国民航大学飞凡创翼团队，
*               购买者要为飞凡创翼团队提供保护，未经作者许可，不得将源代码提供给他人，不得将源代码放到网上供他人免费下载，
*               更不能以此销售牟利，如发现上述行为，飞凡创翼团队将诉之以法律解决！！！
-----------------------------------------------------------------------------------------------------------------------/
*               修改日期:2023/5/6
*               版本：TanlorS V1.1
*               版权所有，盗版必究。
*               Copyright(C) 中国民航大学飞凡创翼团队 2017-2025
*               All rights reserved
----------------------------------------------------------------------------------------------------------------------*/
#include "Headfile.h"
#include "Time_Cnt.h"

#define Hour         3
#define Minute       2
#define Second       1
#define MicroSecond  0
uint16 Time_Sys[4] = { 0 };
uint16 Microsecond_Cnt = 0;
u32 TIME_ISR_CNT = 0;

//系统运行时间计时器
void TIM4_Configuration_Cnt(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  TIM_DeInit(TIM4);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);  	//TIM14时钟使能    



  GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);


  //初始化PF9

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);              //初始化PF9

  TIM_TimeBaseStructure.TIM_Period = 10000;//10ms
  TIM_TimeBaseStructure.TIM_Prescaler = 84-1; //1us
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性低

														   //TIM_OC1Init(TIM4, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC1
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC1

  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能TIM14在CCR1上的预装载寄存器

  TIM_ARRPreloadConfig(TIM4, ENABLE);//ARPE使能 

  TIM_Cmd(TIM4, ENABLE);  //使能TIM14



  TIM_ClearFlag(TIM4, TIM_FLAG_Update);
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIM4, ENABLE);
}

//每10ms进入一次中断
void TIM4_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM4, TIM_FLAG_Update);
		TIME_ISR_CNT++;
		Microsecond_Cnt++;
		if (Microsecond_Cnt >= 100)//1s
		{
			Microsecond_Cnt = 0;
			Time_Sys[Second]++;
			if (Time_Sys[Second] >= 60)//1min
			{
				Time_Sys[Second] = 0;
				Time_Sys[Minute]++;
				if (Time_Sys[Minute] >= 60)//1hour
				{
					Time_Sys[Minute] = 0;
					Time_Sys[Hour]++;
				}
			}
		}
		Time_Sys[MicroSecond] = Microsecond_Cnt;
	}
}


void Test_Period(Testime *Time_Lab)
{
  Time_Lab->Last_Time=Time_Lab->Now_Time;
  Time_Lab->Now_Time=(10000*TIME_ISR_CNT+TIM4->CNT)/1000.0;//单位ms
  Time_Lab->Time_Delta=Time_Lab->Now_Time-Time_Lab->Last_Time;
  Time_Lab->Time_Delta_INT=(uint16)(Time_Lab->Time_Delta);
}


void Cost_TimeStart(Testime *Time_Lab)
{
	Time_Lab->Now_Time = (10000 * TIME_ISR_CNT + TIM4->CNT) / 1000.0;
}

void Cost_TimeEnd(Testime *Time_Lab)
{
	Time_Lab->Last_Time = Time_Lab->Now_Time;
	Time_Lab->Now_Time = (10000 * TIME_ISR_CNT + TIM4->CNT) / 1000.0;//单位ms
	Time_Lab->Time_Delta = Time_Lab->Now_Time - Time_Lab->Last_Time;
	Time_Lab->Time_Delta_INT = (uint16)(Time_Lab->Time_Delta);
}

void Cost_read(Testime *Time_Lab)
{
	Time_Lab->Time_Delta = (10000 * TIME_ISR_CNT + TIM4->CNT) / 1000.0 - Time_Lab->Now_Time;
}
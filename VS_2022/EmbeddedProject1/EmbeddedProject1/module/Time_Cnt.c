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

#include "stm32f4xx.h"
#include "Time_Cnt.h"


u32 TIME_ISR_CNT = 0;

//系统运行时间计时器
void TIM4_Configuration_Cnt(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    TIM_DeInit(TIM4);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //TIM14时钟使能    



    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);


    TIM_TimeBaseStructure.TIM_Period = 10000; //10ms
    TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1; //1us
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性低

                                                             //TIM_OC1Init(TIM4, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC1
    TIM_OC3Init(TIM4, &TIM_OCInitStructure); //根据T指定的参数初始化外设TIM1 4OC1

    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable); //使能TIM14在CCR1上的预装载寄存器

    TIM_ARRPreloadConfig(TIM4, ENABLE); //ARPE使能 

    TIM_Cmd(TIM4, ENABLE); //使能TIM14



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
    }
}


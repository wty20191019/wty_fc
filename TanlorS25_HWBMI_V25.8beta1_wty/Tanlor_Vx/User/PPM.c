/*----------------------------------------------------------------------------------------------------------------------/
*               本程序只供购买者学习使用，版权著作权属于飞凡创翼团队，飞凡创翼团队将飞控程序源码提供给购买者，
*               购买者要为飞凡创翼团队提供保护，未经作者许可，不得将源代码提供给他人，不得将源代码放到网上供他人免费下载，
*               更不能以此销售牟利，如发现上述行为，飞凡创翼团队将诉之以法律解决！！！
-----------------------------------------------------------------------------------------------------------------------/
*               修改日期:2023/5/6
*               版本：TanlorS V1.0
*               版权所有，盗版必究。
*               Copyright(C) 中国民航大学飞凡创翼团队 2017-2025
*               All rights reserved
----------------------------------------------------------------------------------------------------------------------*/
#include "Headfile.h"
#include "PPM.h"

/***************************************************
函数名: void PPM_Init(void)
说明:	PPM接收初始化
入口:	无
出口:	无
备注:	上电初始化，运行一次
****************************************************/
void PPM_Init()
{

	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;

   RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟
   SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource8);//PE2 连接到中断线2

   EXTI_InitStructure.EXTI_Line = EXTI_Line8;//LINE0
   EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
   EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //上升沿触发 
   EXTI_InitStructure.EXTI_LineCmd = ENABLE;//使能LINE0
   EXTI_Init(&EXTI_InitStructure);//配置

}

uint16 PPM_Sample_Cnt=0;
uint16 PPM_Isr_Cnt=0;
u32 Last_PPM_Time=0;
u32 PPM_Time=0;
u16 PPM_Time_Delta=0;
u16 PPM_Time_Max=0;
uint16 PPM_Start_Time=0;
uint16 PPM_Finished_Time=0;
uint16 PPM_Is_Okay=0;
uint16 PPM_Databuf[10]={0};








/***************************************************
函数名: void EXTI9_5_IRQHandler(void)
说明:	PPM接收中断函数
入口:	无
出口:	无
备注:	程序初始化后、始终运行
****************************************************/
static uint16 PPM_buf[10]={0};
void EXTI9_5_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line8) != RESET)
  {
	  EXTI_ClearITPendingBit(EXTI_Line8);
    //系统运行时间获取，单位us
    Last_PPM_Time=PPM_Time;
    PPM_Time=10000*TIME_ISR_CNT+TIM4->CNT;
    PPM_Time_Delta=PPM_Time-Last_PPM_Time;
    //PPM中断进入判断
    if(PPM_Isr_Cnt<100)  PPM_Isr_Cnt++;
    //PPM解析开始
    if(PPM_Is_Okay==1)
    {
      if(PPM_Time_Delta>=800&&PPM_Time_Delta<=2200)
      {
        PPM_Sample_Cnt++;
        //对应通道写入缓冲区
        PPM_buf[PPM_Sample_Cnt-1]=PPM_Time_Delta;
		//单次解析结束
			if (PPM_Sample_Cnt >= 10)
			{
				memcpy(PPM_Databuf, PPM_buf, PPM_Sample_Cnt * sizeof(uint16));
				PPM_Is_Okay = 0;
			}
      }
      else
      {
        if(PPM_Time_Delta>=3000)//帧结束电平至少2ms=2000us，由于部分老版本遥控器、
          //接收机输出PPM信号不标准，当出现解析异常时，尝试改小此值，该情况仅出现一例：使用天地飞老版本遥控器
        {
          PPM_Is_Okay = 1;
          PPM_Sample_Cnt=0;
        }
        else  PPM_Is_Okay=0;
      }
    }
    else if(PPM_Time_Delta>=2500)//帧结束电平至少2ms=2000us
    {
      PPM_Is_Okay=1;
      PPM_Sample_Cnt=0;
    }
  }
 
}



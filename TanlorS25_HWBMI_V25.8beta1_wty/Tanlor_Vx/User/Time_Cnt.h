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
#ifndef _TIME_CNT_H_
#define _TIME_CNT_H_

#define Hour         3
#define Minute       2
#define Second       1
#define MicroSecond  0


void TIM4_Configuration_Cnt(void);

extern uint16 Time_Sys[4];

extern u32 TIME_ISR_CNT;


void Test_Period(Testime *Time_Lab);
void Cost_TimeStart(Testime *Time_Lab);//计时开始
void Cost_read(Testime *Time_Lab);
void Cost_TimeEnd(Testime *Time_Lab);  //计时结束

#endif


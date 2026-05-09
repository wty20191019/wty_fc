#ifndef __TIM2_SCHEDULER_H__
#define __TIM2_SCHEDULER_H__

#include "stm32f4xx.h"

typedef void(*TaskFunc)(void);

#define MAX_TASKS       (16U) //最多任务数量

#define PRIORITY_HIGH   (0U)  
#define PRIORITY_MAX    (14U) //最大优先级，0为最高优先级，数值越大优先级越低

void SCH_Init(void);
void SCH_AddTask(TaskFunc func, uint32_t period, uint8_t priority);
void SCH_Tick(void);
uint32_t SCH_GetTick(void);		//获取系统运行的总毫秒数
void SCH_Dispatch(void);		//在TIM2中断中执行到期任务
void SCH_Delay(uint32_t ms);	//阻塞式延时函数，单位毫秒

#endif





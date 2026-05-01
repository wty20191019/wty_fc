#ifndef __TIM2_SCHEDULER_H__
#define __TIM2_SCHEDULER_H__

#include "stm32f4xx.h"

typedef void (*TaskFunc)(void);

#define MAX_TASKS       (16U) // 最大任务数

#define PRIORITY_HIGH   (0U)  // 最高优先级
#define PRIORITY_MAX    (14U) // 最低优先级

void SCH_Init(void);
void SCH_AddTask(TaskFunc func, uint32_t period, uint8_t priority);// 添加任务到调度器
void SCH_Tick(void);
uint32_t SCH_GetTick(void);
void SCH_Dispatch(void);
void SCH_Delay(uint32_t ms); // 基于调度器的延时函数

#endif



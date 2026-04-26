#include "tim2_scheduler.h"
#include "misc.h"

typedef struct {
    TaskFunc func;
    uint32_t period;
    uint32_t next_run;
    uint8_t priority;
} Task_t;

static Task_t tasks[MAX_TASKS];
static uint8_t task_count = 0;
static volatile uint32_t tim2_tick = 0;

static uint32_t TIM2_GetClockHz(void)
{
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);

    if ((RCC->CFGR & RCC_CFGR_PPRE1) == RCC_CFGR_PPRE1_DIV1) {
        return clocks.PCLK1_Frequency;
    }
    return (clocks.PCLK1_Frequency * 2U);
}

void SCH_Init(void)
{
    TIM_TimeBaseInitTypeDef tim_base;
    NVIC_InitTypeDef nvic;
    uint32_t tim_clk_hz;
    uint16_t prescaler;

    for (uint32_t i = 0; i < MAX_TASKS; i++) {
        tasks[i].func = 0;
        tasks[i].period = 0;
        tasks[i].next_run = 0;
        tasks[i].priority = PRIORITY_MAX;
    }

    task_count = 0;
    tim2_tick = 0;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_Cmd(TIM2, DISABLE);
    TIM_DeInit(TIM2);

    tim_clk_hz = TIM2_GetClockHz();
    prescaler = (uint16_t)((tim_clk_hz / 1000000U) - 1U);

    tim_base.TIM_Prescaler = prescaler;
    tim_base.TIM_CounterMode = TIM_CounterMode_Up;
    tim_base.TIM_Period = 1000U - 1U;
    tim_base.TIM_ClockDivision = TIM_CKD_DIV1;
    tim_base.TIM_RepetitionCounter = 0U;
    TIM_TimeBaseInit(TIM2, &tim_base);

    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    nvic.NVIC_IRQChannel = TIM2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1U;
    nvic.NVIC_IRQChannelSubPriority = 0U;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    TIM_Cmd(TIM2, ENABLE);
}

void SCH_AddTask(TaskFunc func, uint32_t period, uint8_t priority)
{
    if ((task_count < MAX_TASKS) && (func != 0) && (period > 0U)) {
        if (priority > PRIORITY_MAX) {
            priority = PRIORITY_MAX;
        }

        tasks[task_count].func = func;
        tasks[task_count].period = period;
        tasks[task_count].next_run = tim2_tick + period;
        tasks[task_count].priority = priority;
        task_count++;
    }
}

void SCH_Tick(void)
{
    tim2_tick++;
}

uint32_t SCH_GetTick(void)
{
    return tim2_tick;
}

void SCH_Dispatch(void)
{
    static uint32_t last_check = 0U;
    uint32_t current_tick = tim2_tick;

    if (current_tick == last_check) {
        return;
    }
    last_check = current_tick;

    for (uint32_t prio = PRIORITY_HIGH; prio <= PRIORITY_MAX; prio++) {
        for (uint32_t i = 0; i < task_count; i++) {
            if ((tasks[i].priority == prio) && (tasks[i].func != 0)) {
                if ((int32_t)(current_tick - tasks[i].next_run) >= 0) {
                    tasks[i].next_run += tasks[i].period;

                    if ((int32_t)(current_tick - tasks[i].next_run) >= 0) {
                        tasks[i].next_run = current_tick + tasks[i].period;
                    }

                    tasks[i].func();
                }
            }
        }
    }
}

void SCH_Delay(uint32_t ms)
{
    uint32_t start_tick = tim2_tick;

    while ((tim2_tick - start_tick) < ms) {
        __WFI();
    }
}

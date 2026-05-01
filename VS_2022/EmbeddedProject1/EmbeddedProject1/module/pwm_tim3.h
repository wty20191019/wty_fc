#ifndef __PWM_TIM3_H__
#define __PWM_TIM3_H__

#include "stm32f4xx.h"

#define PWM_TIM3_PSC           (84U - 1U)
#define PWM_TIM3_ARR           (20000U - 1U)//ARR值设置为19999对应20ms
void PWM_Init(void);

void PWM_Set(   const uint16_t pwm1, 
                const uint16_t pwm2,
                const uint16_t pwm3, 
                const uint16_t pwm4);


#endif

#ifndef __PPM_H
#define __PPM_H

#include "stm32f4xx.h"

/* ========= 用户可修改 ========= */
#define PPM_GPIO_PORT        GPIOA
#define PPM_GPIO_PIN         GPIO_Pin_8
#define PPM_EXTI_LINE        EXTI_Line8
#define PPM_EXTI_IRQn        EXTI9_5_IRQn
#define PPM_EXTI_PORT_SRC    EXTI_PortSourceGPIOA
#define PPM_EXTI_PIN_SRC     EXTI_PinSource8
/* ============================= */

#define PPM_CHANNEL_NUM     10

typedef struct
{
	uint16_t channel[PPM_CHANNEL_NUM];
	uint8_t  ready;
} PPM_Data_t;

void PPM_Init(void);
void PPM_IRQHandler(void);

void TIM4_Init_1us(void);

extern PPM_Data_t ppm_data;


#endif


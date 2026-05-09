#ifndef _PPM_H
#define _PPM_H


void PPM_GPIO_Init(void);
void PPM_Init(void);
void TIM4_Configuration_Cnt(void);
extern u32 TIME_ISR_CNT;
extern uint16 PPM_Isr_Cnt;
extern uint16 PPM_Databuf[10];
extern u32 PPM_Time;




#endif


#ifndef _PPM_H
#define _PPM_H






#define PPM_GPIO_PORT       GPIOA
#define PPM_GPIO_CLK        RCC_AHB1Periph_GPIOA
#define PPM_GPIO_PIN        GPIO_Pin_8
#define PPM_GPIO_PINSOURCE  GPIO_PinSource8
#define PPM_EXTI_PORTSOURCE EXTI_PortSourceGPIOA
#define PPM_EXTI_PINSOURCE  EXTI_PinSource8
#define PPM_EXTI_LINE       EXTI_Line8



void PPM_GPIO_Init(void);
void PPM_Init(void);
void TIM4_Configuration_Cnt(void);
extern u32 TIME_ISR_CNT;
extern uint16 PPM_Isr_Cnt;
extern uint16 PPM_Databuf[10];
extern u32 PPM_Time;
#endif


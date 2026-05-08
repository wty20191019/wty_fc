#include "Headfile.h"
#include "My_Time.h"


extern void FFCY_Main_Processing();
void TIM2_Configuration(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	TIM_DeInit(TIM2);

	TIM_TimeBaseStructure.TIM_Period = 5000;//5ms
	TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1; //1us
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
}



void TIM2_IRQHandler(void)//5ms
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_FLAG_Update);
		//FFCY_Main_Processing();

        static uint8_t i = 0;
        if (++i >= 5) PA0_LED_Toggle(), i = 0;
		
	}
}






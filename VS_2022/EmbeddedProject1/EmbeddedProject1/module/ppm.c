#include "ppm.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
#include <string.h>
#include "stm32f4xx_tim.h"



static volatile uint32_t tim4_overflow = 0;

void TIM4_Init_1us(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	TIM_TimeBaseStructure.TIM_Period = 0xFFFF; // 16位最大值
	TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1; // 84MHz / 84 = 1MHz
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; // 低于TIM2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_Cmd(TIM4, ENABLE);
}

void TIM4_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
		tim4_overflow++;
	}
}

uint32_t PPM_GetMicros(void)
{
	uint32_t cnt;
	uint32_t overflow;

	do {
		overflow = tim4_overflow;
		cnt = TIM4->CNT;
	} while (overflow != tim4_overflow); // 防止读取时溢出

	return (overflow << 16) | cnt;
}













static uint8_t ppm_index = 0;
static uint32_t ppm_last_tick = 0;

PPM_Data_t ppm_data = { 0 };

/* ================= 中断解码 ================= */
void PPM_IRQHandler(void)
{
	uint32_t now = PPM_GetMicros();
	uint32_t delta = now - ppm_last_tick;
	ppm_last_tick = now;

	if (delta > 3000)   // 帧同步
	{
		ppm_index = 0;
		return;
	}

	if (delta < 800 || delta > 2200)
		return;

	if (ppm_index < PPM_CHANNEL_NUM)
	{
		ppm_data.channel[ppm_index++] = delta;
	}

	if (ppm_index >= PPM_CHANNEL_NUM)
	{
		ppm_data.ready = 1;
		ppm_index = 0;
	}
}

/* ================= 初始化 ================= */
void PPM_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* 1. 开启时钟 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/* 2. GPIO 输入上拉 */
	GPIO_InitStructure.GPIO_Pin  = PPM_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(PPM_GPIO_PORT, &GPIO_InitStructure);

	/* 3. SYSCFG 外部中断映射 */
	SYSCFG_EXTILineConfig(PPM_EXTI_PORT_SRC, PPM_EXTI_PIN_SRC);

	/* 4. EXTI 配置 */
	EXTI_InitStructure.EXTI_Line    = PPM_EXTI_LINE;
	EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* 5. NVIC */
	NVIC_InitStructure.NVIC_IRQChannel = PPM_EXTI_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}








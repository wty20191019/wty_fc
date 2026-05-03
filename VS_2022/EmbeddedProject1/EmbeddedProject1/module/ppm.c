#include "ppm.h"
#include "stm32f4xx_tim.h"
#include "misc.h"
#include <string.h>

extern volatile uint32_t g_ppmTimeOverflow;
extern uint16_t g_ppmChannels[10];

u32 PPM_GetTimeUs(void)
{
	uint32_t overflow1;
	uint32_t overflow2;
	uint32_t cnt;

	do
	{
		overflow1 = g_ppmTimeOverflow;
		cnt = TIM4->CNT;
		overflow2 = g_ppmTimeOverflow;
	} while (overflow1 != overflow2);

	return (overflow1 << 16) + cnt;
}

static u32(*s_get_time_us)(void) = NULL;
static void(*s_frame_ready_cb)(const u16*, u8) = NULL;

static u16 s_buf[10];
static u16 s_frame[10];
static u8 s_sample_cnt = 0;
static u8 s_is_okay = 0;
static u32 s_last_time = 0;
static u32 s_time = 0;
static u8 s_new_frame = 0;
static u16 s_isr_cnt = 0;

void PPM_TimeBase_Init(void)
{
	RCC_ClocksTypeDef clocks;
	TIM_TimeBaseInitTypeDef tim_base;
	NVIC_InitTypeDef nvic;
	uint32_t tim_clk_hz;
	uint16_t prescaler;

	RCC_GetClocksFreq(&clocks);
	tim_clk_hz = clocks.PCLK1_Frequency;
	if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
	{
		tim_clk_hz *= 2U;
	}

	prescaler = (uint16_t)((tim_clk_hz / 1000000U) - 1U);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	TIM_Cmd(TIM4, DISABLE);
	TIM_DeInit(TIM4);

	tim_base.TIM_Prescaler = prescaler;
	tim_base.TIM_CounterMode = TIM_CounterMode_Up;
	tim_base.TIM_Period = 0xFFFFU;
	tim_base.TIM_ClockDivision = TIM_CKD_DIV1;
	tim_base.TIM_RepetitionCounter = 0U;
	TIM_TimeBaseInit(TIM4, &tim_base);

	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

	nvic.NVIC_IRQChannel = TIM4_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 1U;
	nvic.NVIC_IRQChannelSubPriority = 1U;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	g_ppmTimeOverflow = 0U;
	TIM_Cmd(TIM4, ENABLE);
}

void PPM_FrameReadyCb(const u16* data, u8 len)
{
	u8 copy_len = (len < 10U) ? len : 10U;

	for (u8 i = 0U; i < copy_len; ++i)
	{
		g_ppmChannels[i] = data[i];
	}
}

void ppm_init(u32(*get_time_us)(void), void(*frame_ready_cb)(const u16* data, u8 len))
{
	s_get_time_us = get_time_us;
	s_frame_ready_cb = frame_ready_cb;
	s_sample_cnt = 0;
	s_is_okay = 0;
	s_last_time = 0;
	s_time = 0;
	s_new_frame = 0;
	s_isr_cnt = 0;
}

void ppm_exti_handler(void)
{
	if (s_get_time_us == NULL) return;
	s_last_time = s_time;
	s_time = s_get_time_us();
	u32 delta = s_time - s_last_time;

	if (s_isr_cnt < 100) s_isr_cnt++;

	if (s_is_okay)
	{
		if (delta >= 800 && delta <= 2200)
		{
			if (s_sample_cnt < 10)
			{
				s_buf[s_sample_cnt++] = (u16)delta;
				if (s_sample_cnt >= 10)
				{
					memcpy(s_frame, s_buf, sizeof(s_frame));
					s_new_frame = 1;
					s_is_okay = 0;
					if (s_frame_ready_cb) s_frame_ready_cb(s_frame, 10);
				}
			}
		}
		else
		{
			if (delta >= 3000)
			{
				/* 帧结束，开始新一帧解析 */
				s_is_okay = 1;
				s_sample_cnt = 0;
			}
			else
			{
				s_is_okay = 0;
			}
		}
	}
	else if (delta >= 2500)
	{
		/* 认为是帧起始的长脉冲 */
		s_is_okay = 1;
		s_sample_cnt = 0;
	}
}

u8 ppm_get_frame(u16 *out_buf, u8 max_len)
{
	if (!s_new_frame) return 0;
	u8 copy_len = (max_len < 10) ? max_len : 10;
	memcpy(out_buf, s_frame, copy_len * sizeof(u16));
	s_new_frame = 0;
	return copy_len;
}



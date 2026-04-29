#include "esc_calibration.h"

#include "board.h"
#include "pwm_tim3.h"

#define ESC_PULSE_MIN_US            (1000U)     //1000U对应1ms，电调常用的最小有效脉宽值，单位微秒
#define ESC_PULSE_MAX_US            (2000U)     //2000U对应2ms，电调常用的最大有效脉宽值，单位微秒

static uint16_t ESC_ClampPulse(uint16_t pulse_us)  //将输入的脉宽值限制在ESC支持的范围内，并且不超过PWM定时器的ARR值
{
    if (pulse_us < ESC_PULSE_MIN_US)
    {
        return ESC_PULSE_MIN_US;
    }

    if (pulse_us > ESC_PULSE_MAX_US)
    {
        return ESC_PULSE_MAX_US;
    }

    if (pulse_us > PWM_TIM3_ARR)
    {
        return PWM_TIM3_ARR;
    }

    return pulse_us;
}

void ESC_SetChannelsUs(uint16_t channel1_us,
                       uint16_t channel2_us,
                       uint16_t channel3_us,
                       uint16_t channel4_us)
{
    PWM_Set(ESC_ClampPulse(channel1_us),
            ESC_ClampPulse(channel2_us),
            ESC_ClampPulse(channel3_us),
            ESC_ClampPulse(channel4_us));
}

void ESC_SetAllUs(uint16_t pulse_us)//将所有通道设置为相同的脉宽值，适用于解锁和校准等场景
{
    pulse_us = ESC_ClampPulse(pulse_us);
    PWM_Set(pulse_us, pulse_us, pulse_us, pulse_us);
}

void ESC_Arm(void)
{
    ESC_SetAllUs(ESC_PULSE_MIN_US);
}

void ESC_Init(void)
{
    ESC_Arm();
}

void ESC_CalibrateSequence(uint32_t max_hold_ms, uint32_t min_hold_ms)
{
    ESC_SetAllUs(ESC_PULSE_MAX_US);
    systick_delay_ms(max_hold_ms);

    ESC_SetAllUs(ESC_PULSE_MIN_US);
    systick_delay_ms(min_hold_ms);

    ESC_Arm();
}



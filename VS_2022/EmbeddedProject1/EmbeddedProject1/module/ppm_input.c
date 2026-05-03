#include "ppm_input.h"

#include "misc.h"

#define PPM_SYNC_GAP_US         (3000U)
#define PPM_MIN_VALID_US        (750U)
#define PPM_MAX_VALID_US        (2250U)
#define PPM_CAPTURE_PERIOD_US   (0xFFFFU)

static volatile uint16_t g_ppm_captureLast = 0U;
static volatile uint16_t g_ppm_buildChannels[PPM_MAX_CHANNELS];
static volatile uint16_t g_ppm_frameChannels[PPM_MAX_CHANNELS];
static volatile uint8_t g_ppm_buildCount = 0U;
static volatile uint8_t g_ppm_frameCount = 0U;
static volatile uint8_t g_ppm_frameReady = 0U;

static uint32_t PPM_GetTim1ClockHz(void)
{
    RCC_ClocksTypeDef clocks;

    RCC_GetClocksFreq(&clocks);

    if ((RCC->CFGR & RCC_CFGR_PPRE2) == RCC_CFGR_PPRE2_DIV1)
    {
        return clocks.PCLK2_Frequency;
    }

    return (clocks.PCLK2_Frequency * 2U);
}


static void PPM_StoreFrame(void)
{
    uint8_t count = g_ppm_buildCount;

    if (count > PPM_MAX_CHANNELS)
    {
        count = PPM_MAX_CHANNELS;
    }

    for (uint8_t i = 0U; i < count; ++i)
    {
        g_ppm_frameChannels[i] = g_ppm_buildChannels[i];
    }

    g_ppm_frameCount = count;
    g_ppm_frameReady = 1U;
}

void PPM_Init(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef timBase;
    TIM_ICInitTypeDef timIc;
    NVIC_InitTypeDef nvic;
    uint32_t timClkHz;
    uint16_t prescaler;

    for (uint8_t i = 0U; i < PPM_MAX_CHANNELS; ++i)
    {
        g_ppm_buildChannels[i] = 0U;
        g_ppm_frameChannels[i] = 0U;
    }

    g_ppm_captureLast = 0U;
    g_ppm_buildCount = 0U;
    g_ppm_frameCount = 0U;
    g_ppm_frameReady = 0U;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);

    gpio.GPIO_Pin = GPIO_Pin_8;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &gpio);

    timClkHz = PPM_GetTim1ClockHz();
    prescaler = (uint16_t)((timClkHz / 1000000U) - 1U);

    TIM_Cmd(TIM1, DISABLE);
    TIM_DeInit(TIM1);

    timBase.TIM_Prescaler = prescaler;
    timBase.TIM_CounterMode = TIM_CounterMode_Up;
	timBase.TIM_Period = PPM_CAPTURE_PERIOD_US; //设置自动重装载寄存器的值为PPM_CAPTURE_PERIOD_US，这样计数器每PPM_CAPTURE_PERIOD_US微秒溢出一次
	timBase.TIM_ClockDivision = TIM_CKD_DIV1; //不分频，直接使用定时器时钟
    timBase.TIM_RepetitionCounter = 0U;
    TIM_TimeBaseInit(TIM1, &timBase);

	timIc.TIM_Channel = TIM_Channel_1; //使用定时器1的通道1进行输入捕获
	timIc.TIM_ICPolarity = TIM_ICPolarity_Rising; //捕获上升沿
	timIc.TIM_ICSelection = TIM_ICSelection_DirectTI; //直接连接到TI1输入
	timIc.TIM_ICPrescaler = TIM_ICPSC_DIV1; //捕获每个事件，不进行预分频
	timIc.TIM_ICFilter = 0U; //不使用输入滤波器
    TIM_ICInit(TIM1, &timIc);

    TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);
    TIM_ITConfig(TIM1, TIM_IT_CC1, ENABLE);

    nvic.NVIC_IRQChannel = TIM1_CC_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1U;
    nvic.NVIC_IRQChannelSubPriority = 0U;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    TIM_Cmd(TIM1, ENABLE);

    /* initialize last capture from current CCR1 to avoid huge first delta */
	
    g_ppm_captureLast = TIM_GetCapture1(TIM1);
}

void PPM_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_CC1) != RESET)
    {
        uint16_t capture;
        uint16_t delta;

        TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);

        capture = TIM_GetCapture1(TIM1);

        if (capture >= g_ppm_captureLast)
        {
            delta = (uint16_t)(capture - g_ppm_captureLast);
        }
        else
        {
            delta = (uint16_t)((PPM_CAPTURE_PERIOD_US - g_ppm_captureLast) + capture + 1U);
        }

        g_ppm_captureLast = capture;

        if (delta >= PPM_SYNC_GAP_US)
        {
            if (g_ppm_buildCount > 0U)
            {
                PPM_StoreFrame();
            }

            g_ppm_buildCount = 0U;
        }
        else if ((delta >= PPM_MIN_VALID_US) && (delta <= PPM_MAX_VALID_US))
        {
            if (g_ppm_buildCount < PPM_MAX_CHANNELS)
            {
                g_ppm_buildChannels[g_ppm_buildCount] = delta;
                g_ppm_buildCount++;
            }
        }
        else
        {
            /* ignore invalid pulse widths */
        }
    }
}

uint8_t PPM_HasFrame(void)
{
    return g_ppm_frameReady;
}

// Returns 1 if a frame was read successfully, 0 otherwise. If successful, the channel values are stored in the provided array and the channel count is updated.
uint8_t PPM_ReadFrame(uint16_t *channels, uint8_t maxChannels, uint8_t *channelCount)
{
    uint8_t count;

    if ((channels == 0) || (maxChannels == 0U))
    {
        return 0U;
    }

    __disable_irq();

    if (g_ppm_frameReady == 0U)
    {
        __enable_irq();
        return 0U;
    }

    count = g_ppm_frameCount;
    if (count > maxChannels)
    {
        count = maxChannels;
    }

    for (uint8_t i = 0U; i < count; ++i)
    {
        channels[i] = g_ppm_frameChannels[i];
    }

    g_ppm_frameReady = 0U;

    __enable_irq();

    if (channelCount != 0)
    {
        *channelCount = count;
    }

    return 1U;
}

uint16_t PPM_GetChannelUs(uint8_t channelIndex)
{
    uint16_t value = 0U;

    __disable_irq();

    if ((g_ppm_frameReady != 0U) && (channelIndex < g_ppm_frameCount))
    {
        value = g_ppm_frameChannels[channelIndex];
    }

    __enable_irq();

    return value;
}

uint8_t PPM_GetChannelCount(void)
{
    return g_ppm_frameCount;
}

#include "ppm_input.h"

#include "misc.h"
#include "tim2_scheduler.h" /* use SCH_GetTick() for ms base */
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"

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

    /* Configure PA8 as input with EXTI line
     * Use TIM2 counter (already running for scheduler) as microsecond time base
     */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_8;
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &gpio);

    /* connect EXTI8 line to PA8 */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource8);

    EXTI_InitTypeDef extiInit;
    extiInit.EXTI_Line = EXTI_Line8;
    extiInit.EXTI_Mode = EXTI_Mode_Interrupt;
    extiInit.EXTI_Trigger = EXTI_Trigger_Rising;
    extiInit.EXTI_LineCmd = ENABLE;
    EXTI_Init(&extiInit);

    nvic.NVIC_IRQChannel = EXTI9_5_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1U;
    nvic.NVIC_IRQChannelSubPriority = 0U;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    /* initialize last capture to current time to avoid large first delta */
    uint32_t ms;
    uint16_t cnt;
    do {
        ms = SCH_GetTick();
        cnt = TIM_GetCounter(TIM2);
    } while (ms != SCH_GetTick());
    g_ppm_captureLast = (uint16_t)((ms * 1000U + cnt) & 0xFFFFU);
}

/* Legacy TIM1 handler kept for compatibility */
void PPM_IRQHandler(void)
{
    /* not used in EXTI-based implementation */
}

/* EXTI-based PPM capture handler. Reads current time using TIM2 counter + ms tick. */
void PPM_EXTI_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line8) != RESET)
    {
        uint32_t ms1, ms2;
        uint16_t cnt;
        uint32_t now_us32;
        uint32_t last_us32;
        uint32_t delta_us32;

        EXTI_ClearITPendingBit(EXTI_Line8);

        /* read consistent millisecond tick and TIM2 counter */
        do {
            ms1 = SCH_GetTick();
            cnt = TIM_GetCounter(TIM2);
            ms2 = SCH_GetTick();
        } while (ms1 != ms2);

        now_us32 = ms2 * 1000U + (uint32_t)cnt;
        last_us32 = (uint32_t)g_ppm_captureLast; /* stored as lower 16 bits */

        /* extend last_us32 to 32-bit by assuming monotonic increasing; if wrap, handle */
        /* We stored only 16-bit previously; to be safe, keep a 32-bit last time internal */
        static uint32_t g_last_time_32 = 0U;

        if (g_last_time_32 == 0U)
        {
            g_last_time_32 = now_us32;
        }

        if (now_us32 >= g_last_time_32)
        {
            delta_us32 = now_us32 - g_last_time_32;
        }
        else
        {
            /* wrap-around handling */
            delta_us32 = (0xFFFFFFFFU - g_last_time_32) + now_us32 + 1U;
        }

        g_last_time_32 = now_us32;
        g_ppm_captureLast = (uint16_t)(now_us32 & 0xFFFFU);

        if (delta_us32 >= PPM_SYNC_GAP_US)
        {
            if (g_ppm_buildCount > 0U)
            {
                PPM_StoreFrame();
            }

            g_ppm_buildCount = 0U;
        }
        else if ((delta_us32 >= PPM_MIN_VALID_US) && (delta_us32 <= PPM_MAX_VALID_US))
        {
            if (g_ppm_buildCount < PPM_MAX_CHANNELS)
            {
                g_ppm_buildChannels[g_ppm_buildCount] = (uint16_t)delta_us32;
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

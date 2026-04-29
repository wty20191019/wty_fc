#include "pa0_LED_toggle.h"

void PA0_LED_Toggle_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_0;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    GPIO_ResetBits(GPIOA, GPIO_Pin_0);
}

void PA0_LED_Toggle(void)
{
    GPIO_ToggleBits(GPIOA, GPIO_Pin_0);
}


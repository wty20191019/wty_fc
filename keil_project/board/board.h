
#ifndef __SYSTICK_DELAY_H__
#define __SYSTICK_DELAY_H__

#include "stm32f4xx.h"

void board_init(void);
void systick_delay_us(uint32_t _us);
void systick_delay_ms(uint32_t _ms);
void systick_delay_1ms(uint32_t ms);
void systick_delay_1us(uint32_t us);

#endif

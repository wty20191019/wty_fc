#ifndef __ESC_CALIBRATION_H__
#define __ESC_CALIBRATION_H__

#include "stm32f4xx.h"

void ESC_Init(void);//电调初始化，默认调用ESC_Arm函数将所有通道设置为最小脉宽值以解锁电调
void ESC_SetAllUs(uint16_t pulse_us);//将所有通道设置为相同的脉宽值，适用于解锁和校准等场景
void ESC_SetChannelsUs(uint16_t channel1_us,//将每个通道设置为指定的脉宽值，单位微秒，适用于正常运行时的电调控制
                       uint16_t channel2_us,
                       uint16_t channel3_us,
                       uint16_t channel4_us);
void ESC_Arm(void);//电调解锁，通常是将所有通道设置为最小脉宽值（如1000us）以满足大多数电调的解锁条件
void ESC_CalibrateSequence(uint32_t max_hold_ms, uint32_t min_hold_ms);//电调校准序列，通过设置最大和最小脉宽值来完成校准

#endif

#ifndef __PPM_INPUT_H__
#define __PPM_INPUT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"

#define PPM_MAX_CHANNELS        (10U) //根据实际需要调整最大通道数量，注意不要超过定时器捕获的能力

	void PPM_Init(void); //初始化PPM输入模块，配置定时器和GPIO
	void PPM_IRQHandler(void); //定时器中断处理函数，捕获PPM信号并解析通道数据

	uint8_t PPM_HasFrame(void); //检查是否有新的PPM帧可用
	uint8_t PPM_ReadFrame(uint16_t *channels, uint8_t maxChannels, uint8_t *channelCount); //读取最新的PPM帧数据，返回通道值数组和通道数量
	uint16_t PPM_GetChannelUs(uint8_t channelIndex); //获取指定通道的值，单位为微秒
	uint8_t PPM_GetChannelCount(void); //获取当前PPM帧的通道数量

#ifdef __cplusplus
}
#endif

#endif

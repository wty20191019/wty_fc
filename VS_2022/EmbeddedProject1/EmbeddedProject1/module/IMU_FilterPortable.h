#ifndef IMU_FILTER_PORTABLE_H //IMU滤波器的头文件，包含了二阶低通滤波器的设计和处理函数
#define IMU_FILTER_PORTABLE_H

#include <stdint.h>
#include <math.h>

#define IMU_FILTER_PORTABLE_PI 3.14159265358979323846f

	typedef struct //二阶低通滤波器的系数结构体，包含了滤波器的五个系数：b0、b1、b2、a1和a2
{
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;
} IMU_FilterPortableBiquadCoeff;

typedef struct//二阶低通滤波器的状态结构体，包含了输入和输出的历史值以及预热计数器和预热限制
{
    float x1;
    float x2;
    float y1;
    float y2;
    uint16_t warmupCount;
    uint16_t warmupLimit;
} IMU_FilterPortableBiquadState;

static inline void IMU_FilterPortable_Reset(IMU_FilterPortableBiquadState *state)
{
    state->x1 = 0.0f;
    state->x2 = 0.0f;
    state->y1 = 0.0f;
    state->y2 = 0.0f;
    state->warmupCount = 0U;
}

static inline void IMU_FilterPortable_DesignLP2(float sampleHz, float cutoffHz,
                                                 IMU_FilterPortableBiquadCoeff *coeff)
{
    if (cutoffHz <= 0.0f || sampleHz <= 0.0f)
    {
        coeff->b0 = 1.0f;
        coeff->b1 = 0.0f;
        coeff->b2 = 0.0f;
        coeff->a1 = 0.0f;
        coeff->a2 = 0.0f;
        return;
    }

    {
        const float fr = sampleHz / cutoffHz;
        const float ohm = tanf(IMU_FILTER_PORTABLE_PI / fr);
        const float cosTerm = cosf(IMU_FILTER_PORTABLE_PI / 4.0f);
        const float c = 1.0f + 2.0f * cosTerm * ohm + ohm * ohm;
        const float b0 = (ohm * ohm) / c;

        coeff->b0 = b0;
        coeff->b1 = 2.0f * b0;
        coeff->b2 = b0;
        coeff->a1 = 2.0f * (ohm * ohm - 1.0f) / c;
        coeff->a2 = (1.0f - 2.0f * cosTerm * ohm + ohm * ohm) / c;
    }
}

static inline void IMU_FilterPortable_Init(IMU_FilterPortableBiquadState *state,
                                           uint16_t warmupLimit)
{
    IMU_FilterPortable_Reset(state);
    state->warmupLimit = warmupLimit;
}
	//二阶低通滤波器的处理函数，输入当前采样值、滤波器状态和滤波器系数，输出滤波后的值
static inline float IMU_FilterPortable_Process(float input,
                                               IMU_FilterPortableBiquadState *state,
                                               const IMU_FilterPortableBiquadCoeff *coeff)
{
    float y;

    if (state->warmupCount < state->warmupLimit)
    {
        y = input;
        state->warmupCount++;
    }
    else
    {
        y = coeff->b0 * input
          + coeff->b1 * state->x1
          + coeff->b2 * state->x2
          - coeff->a1 * state->y1
          - coeff->a2 * state->y2;
    }

    state->x2 = state->x1;
    state->x1 = input;
    state->y2 = state->y1;
    state->y1 = y;

    return y;
}



#endif

#include "KalmanFilter.h"


//===========================================================================
//卡尔曼滤波器实现
//===========================================================================
// processNoise         //过程噪声协方差，表示系统模型的不确定性，较大的值会使滤波器更依赖测量值，较小的值会使滤波器更依赖预测值
// measurementNoise     //测量噪声协方差，表示测量值的不确定性，较大的值会使滤波器更依赖预测值，较小的值会使滤波器更依赖测量值
// initialP             //初始估计误差协方差，表示初始状态的不确定性，较大的值会使滤波器在初始阶段更依赖测量值，较小的值会使滤波器在初始阶段更依赖预测值
// initialX             //初始状态估计值，表示滤波器在没有任何测量数据时的初始猜测
//==========================================================================
void KalmanFilter_Init(KalmanFilterState *k, float processNoise, float measurementNoise, float initialP, float initialX)
{
    if (k == 0) return;
    k->q = processNoise;
    k->r = measurementNoise;
    k->p = initialP;
    k->x = initialX;
    k->initialized = 1;
}

float KalmanFilter_Update(KalmanFilterState *k, float measurement)
{
    if (k == 0) return measurement;
    if (!k->initialized)
    {
        k->x = measurement;
        k->p = 1.0f;
        k->initialized = 1;
        return k->x;
    }

    /* Prediction step: p = p + q */
    k->p = k->p + k->q;

    /* Kalman gain: k_gain = p / (p + r) */
    float k_gain = k->p / (k->p + k->r);

    /* Update estimate with measurement */
    k->x = k->x + k_gain * (measurement - k->x);

    /* Update error covariance: p = (1 - k) * p */
    k->p = (1.0f - k_gain) * k->p;

    return k->x;
}

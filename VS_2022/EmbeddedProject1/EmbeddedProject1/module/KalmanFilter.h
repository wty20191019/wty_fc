#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#ifdef __cplusplus
extern "C" {
#endif
	///卡尔曼滤波器的头文件，包含了卡尔曼滤波器的状态结构体定义和处理函数声明
typedef struct
{
    float q; /* process noise covariance */
    float r; /* measurement noise covariance */
    float p; /* estimation error covariance */
    float x; /* value */
    unsigned char initialized;
} KalmanFilterState;

void KalmanFilter_Init(KalmanFilterState *k, float processNoise, float measurementNoise, float initialP, float initialX);
float KalmanFilter_Update(KalmanFilterState *k, float measurement);

#ifdef __cplusplus
}
#endif

#endif /* KALMANFILTER_H */

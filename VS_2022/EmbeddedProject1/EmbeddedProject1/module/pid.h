
#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
    float kp; //比例增益
    float ki; //积分增益
    float kd; //微分增益

    float integrator; //积分器状态
    float prevError; //前一个误差值
    float prevMeasurement; //前一个测量值
    float prevDerivative; //前一个微分值（用于滤波）

    float outMin; //输出最小值
    float outMax; //输出最大值

    float integratorMin; //积分器最小值（防止积分风up）
    float integratorMax; //积分器最大值（防止积分风up）

    float derivativeFilterAlpha; //微分滤波系数（0..1），1表示不使用滤波
} PID_Handle_t;

void PID_Init(PID_Handle_t *pid, float kp, float ki, float kd, float outMin, float outMax); //初始化PID控制器参数
void PID_Reset(PID_Handle_t *pid); //重置PID控制器状态（积分器和前一个误差等）
float PID_Update(PID_Handle_t *pid, float setpoint, float measurement, float dt); //计算PID控制输出，输入为设定值、测量值和时间间隔，输出为控制量
void PID_SetOutputLimits(PID_Handle_t *pid, float outMin, float outMax); //设置PID控制输出的最小值和最大值
void PID_SetIntegratorLimits(PID_Handle_t *pid, float iMin, float iMax); //设置PID积分器的最小值和最大值，防止积分风up
void PID_SetDerivativeFilterAlpha(PID_Handle_t *pid, float alpha); //设置PID微分滤波系数，范围0..1，值越小滤波效果越强，值为1表示不使用滤波

#endif 






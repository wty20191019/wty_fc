
//======================================================================
#include "pid.h"
#include <stddef.h>

void PID_Init(PID_Handle_t *pid, float kp, float ki, float kd, float outMin, float outMax)
{
    if (pid == NULL) return;

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->integrator = 0.0f;     
    pid->prevError = 0.0f;      
    pid->prevMeasurement = 0.0f;
    pid->prevDerivative = 0.0f;

    pid->outMin = outMin;
    pid->outMax = outMax;

    pid->integratorMin = outMin;
    pid->integratorMax = outMax;

    pid->derivativeFilterAlpha = 1.0f; //默认不使用滤波，直接使用当前的导数值
}

void PID_Reset(PID_Handle_t *pid)
{
    if (pid == NULL) return;
    pid->integrator = 0.0f;
    pid->prevError = 0.0f;
    pid->prevMeasurement = 0.0f;
    pid->prevDerivative = 0.0f;
}

//brief 计算PID控制输出
//PID_Handle_t *pid: PID控制器句柄，包含PID参数和状态
//float setpoint: 目标值
//float measurement: 当前测量值
//float dt: 上次调用到现在的时间间隔，单位秒
float PID_Update(PID_Handle_t *pid, float setpoint, float measurement, float dt)
{
    if (pid == NULL || dt <= 0.0f) return 0.0f;

    float error = setpoint - measurement;

    /* Proportional term */
    float P = pid->kp * error;

    /* Integrator with anti-windup */
    pid->integrator += error * dt * pid->ki;
    if (pid->integrator > pid->integratorMax) pid->integrator = pid->integratorMax;
    if (pid->integrator < pid->integratorMin) pid->integrator = pid->integratorMin;
    float I = pid->integrator;

    /* Derivative (on measurement) to avoid derivative kick on setpoint changes */
    float derivative = 0.0f;
    float measurementDiff = measurement - pid->prevMeasurement;
    derivative = -pid->kd * (measurementDiff / dt);

    /* derivative low-pass filter (simple first-order) */
    derivative = pid->derivativeFilterAlpha * derivative + (1.0f - pid->derivativeFilterAlpha) * pid->prevDerivative;
    pid->prevDerivative = derivative;

    float output = P + I + derivative;

    /* Output saturation */
    if (output > pid->outMax) output = pid->outMax;
    if (output < pid->outMin) output = pid->outMin;

    /* Anti-windup: if output saturated, optionally back-calculate integrator (simple clamping used above) */

    pid->prevError = error;
    pid->prevMeasurement = measurement;

    return output;
}

void PID_SetOutputLimits(PID_Handle_t *pid, float outMin, float outMax)
{
    if (pid == NULL) return;
    pid->outMin = outMin;
    pid->outMax = outMax;

    if (pid->integratorMin < outMin) pid->integratorMin = outMin;
    if (pid->integratorMax > outMax) pid->integratorMax = outMax;
}

void PID_SetIntegratorLimits(PID_Handle_t *pid, float iMin, float iMax)
{
    if (pid == NULL) return;
    pid->integratorMin = iMin;
    pid->integratorMax = iMax;
    if (pid->integrator > pid->integratorMax) pid->integrator = pid->integratorMax;
    if (pid->integrator < pid->integratorMin) pid->integrator = pid->integratorMin;
}

void PID_SetDerivativeFilterAlpha(PID_Handle_t *pid, float alpha)
{
    if (pid == NULL) return;
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    pid->derivativeFilterAlpha = alpha;
}

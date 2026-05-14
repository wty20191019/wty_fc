#ifndef ALL_CONTROL_H
#define ALL_CONTROL_H

#include <stdint.h>

void ALL_Control_Init(void);
void ALL_Control_Task(void);
uint8_t ALL_Control_TunePidBySlider(uint32_t sliderId, float value);

typedef enum
{
    ALL_PID_ROLL_ANGLE = 0,
    ALL_PID_PITCH_ANGLE,
    ALL_PID_YAW_ANGLE,
    ALL_PID_ROLL_RATE,
    ALL_PID_PITCH_RATE,
    ALL_PID_YAW_RATE,
    ALL_PID_COUNT
} ALL_PidIndex_t;

typedef enum
{
    ALL_PID_PARAM_KP = 0,
    ALL_PID_PARAM_KI,
    ALL_PID_PARAM_KD,
    ALL_PID_PARAM_COUNT
} ALL_PidParam_t;



/**

    使用方式（上位机发送）
    发送格式保持你现有的协议：[slider,ID,VALUE]
    •	VALUE 为整数：
    •	0~1000：按滑动条位置自动缩放为增益（kp 默认 0~10，ki 默认 0~5，kd 默认 0~2）
    •	>1000：按 fixed-point 解析，gain = VALUE / 1000.0（例：2500→2.5）
    •	VALUE 也支持浮点：[slider,10,0.35]（直接作为增益）
    
    Slider ID → PID 参数映射
    
    1-3  ：Roll角度环 kp/ki/kd
    4-6  ：Pitch角度环 kp/ki/kd
    7-9  ：Yaw角度环 kp/ki/kd
    10-12：Roll角速度环 kp/ki/kd
    13-15：Pitch角速度环 kp/ki/kd
    16-18：Yaw角速度环 kp/ki/kd
    
    例如：
    •	[slider,1,500]：设置 Roll角度环 kp（按 0~10 缩放）
    •	[slider,10,2500]：设置 Roll角速度环 kp=2.5

 **/





#endif

#include "all_control.h"
#include "pid.h"
#include "esc_calibration.h"
#include "ppm_input.h"
#include "FFCY_NEW_ICM42688.h"

#include "stm32f4xx.h"

#include <stddef.h>


//PPM_Databuf[0] roll
//PPM_Databuf[1] pitch
//PPM_Databuf[2] throttle
//PPM_Databuf[3] yaw

uint16_t yaw_thresh_low;
uint16_t yaw_thresh_high;
uint16_t yaw_ppm;

uint16_t throttle;
float roll_set;
float pitch_set;
float yaw_rate_set; //角速度设定值由偏航角PID输出得到，因此命名为yaw_rate_set更合适
float roll_rate_set; //角速度设定值由角度PID输出得到，因此命名为roll_rate_set更合适
float pitch_rate_set; // 同上，命名为pitch_rate_set更合适

float roll_out;
float pitch_out;
float yaw_out;

float yaw_error;

float roll_rate;
float pitch_rate;
float yaw_rate;






extern float pitchDeg;
extern float rollDeg;
extern float yawDeg;

#define PPM_MIN_US            (1000U)   //PPM输入的最小脉宽，单位微秒，通常为1000us
#define PPM_MAX_US            (2000U)   //PPM输入的最大脉宽，单位微秒，通常为2000us
#define PPM_MID_US            (1500U)   //PPM输入的中立脉宽，单位微秒，通常为1500us
#define PPM_DEADBAND_US       (20U)     //遥控器死区范围
#define THROTTLE_ARM_US       (1100U)   //油门解锁的脉宽阈值，单位微秒
#define CONTROL_DT_SEC        (0.005f)  //控制循环的时间间隔 5ms

#define ROLL_ANGLE_MAX_DEG    (10.0f)   //最大横滚角度，单位度
#define PITCH_ANGLE_MAX_DEG   (10.0f)   //最大俯仰角度，单位度
#define YAW_RATE_MAX_DPS      (45.0f)   //最大偏航角速度，单位度每秒

#define ROLL_RATE_MAX_DPS     (160.0f)  //最大横滚角速度，单位度每秒
#define PITCH_RATE_MAX_DPS    (160.0f)  //最大俯仰角速度，单位度每秒

#define ICM42688_GYRO_DPS_PER_LSB    (1.0f / 16.4f) //ICM42688陀螺仪每个LSB对应的角速度，单位度每秒

#define PID_OUTPUT_MIN        (-200.0f) //PID输出最小值
#define PID_OUTPUT_MAX        (200.0f)  //PID输出最大值

static PID_Handle_t g_pid_roll_angle;//
static PID_Handle_t g_pid_pitch_angle;
static PID_Handle_t g_pid_yaw_angle;

static PID_Handle_t g_pid_roll_rate;
static PID_Handle_t g_pid_pitch_rate;
static PID_Handle_t g_pid_yaw_rate;

static float g_yaw_target = 0.0f;
static uint8_t g_control_ready = 0U;

//================================
#define SCALE_PERCENT_MAX     (0.8f)    //用于计算摇杆（解锁/锁定）阈值的比例，0.9表示需要达到摇杆范围的90%才能触发（解锁/锁定）动作
#define LOCK_COUNT_THRESHOLD  (100U)    //用于计算摇杆（解锁/锁定）阈值的计数阈值 1s
static uint16_t g_lock_count = 0U;      //锁定计数器
static uint16_t g_unlock_count = 0U;    //解锁计数器
static uint8_t  ESC_lock = 0U;          //解锁状态标志



//根据 PID 索引获取对应的 PID 句柄
static PID_Handle_t *ALL_Control_GetPidHandle(ALL_PidIndex_t index)
{
    switch (index)
    {
    case ALL_PID_ROLL_ANGLE:
        return &g_pid_roll_angle;
    case ALL_PID_PITCH_ANGLE:
        return &g_pid_pitch_angle;
    case ALL_PID_YAW_ANGLE:
        return &g_pid_yaw_angle;
    case ALL_PID_ROLL_RATE:
        return &g_pid_roll_rate;
    case ALL_PID_PITCH_RATE:
        return &g_pid_pitch_rate;
    case ALL_PID_YAW_RATE:
        return &g_pid_yaw_rate;
    default:
        return NULL;
    }
}

// 通过上位机滑动条在线调参：sliderId 映射到某个 PID 的某个参数(kp/ki/kd)
uint8_t ALL_Control_TunePidBySlider(uint32_t sliderId, float value)
{
    PID_Handle_t *pid = NULL;
    ALL_PidParam_t param = ALL_PID_PARAM_KP;

    switch (sliderId)
    {
    case 1U:  pid = ALL_Control_GetPidHandle(ALL_PID_ROLL_ANGLE);  param = ALL_PID_PARAM_KP; break;
    case 2U:  pid = ALL_Control_GetPidHandle(ALL_PID_ROLL_ANGLE);  param = ALL_PID_PARAM_KI; break;
    case 3U:  pid = ALL_Control_GetPidHandle(ALL_PID_ROLL_ANGLE);  param = ALL_PID_PARAM_KD; break;
    case 4U:  pid = ALL_Control_GetPidHandle(ALL_PID_PITCH_ANGLE); param = ALL_PID_PARAM_KP; break;
    case 5U:  pid = ALL_Control_GetPidHandle(ALL_PID_PITCH_ANGLE); param = ALL_PID_PARAM_KI; break;
    case 6U:  pid = ALL_Control_GetPidHandle(ALL_PID_PITCH_ANGLE); param = ALL_PID_PARAM_KD; break;
    case 7U:  pid = ALL_Control_GetPidHandle(ALL_PID_YAW_ANGLE);   param = ALL_PID_PARAM_KP; break;
    case 8U:  pid = ALL_Control_GetPidHandle(ALL_PID_YAW_ANGLE);   param = ALL_PID_PARAM_KI; break;
    case 9U:  pid = ALL_Control_GetPidHandle(ALL_PID_YAW_ANGLE);   param = ALL_PID_PARAM_KD; break;
    case 10U: pid = ALL_Control_GetPidHandle(ALL_PID_ROLL_RATE);   param = ALL_PID_PARAM_KP; break;
    case 11U: pid = ALL_Control_GetPidHandle(ALL_PID_ROLL_RATE);   param = ALL_PID_PARAM_KI; break;
    case 12U: pid = ALL_Control_GetPidHandle(ALL_PID_ROLL_RATE);   param = ALL_PID_PARAM_KD; break;
    case 13U: pid = ALL_Control_GetPidHandle(ALL_PID_PITCH_RATE);  param = ALL_PID_PARAM_KP; break;
    case 14U: pid = ALL_Control_GetPidHandle(ALL_PID_PITCH_RATE);  param = ALL_PID_PARAM_KI; break;
    case 15U: pid = ALL_Control_GetPidHandle(ALL_PID_PITCH_RATE);  param = ALL_PID_PARAM_KD; break;
    case 16U: pid = ALL_Control_GetPidHandle(ALL_PID_YAW_RATE);    param = ALL_PID_PARAM_KP; break;
    case 17U: pid = ALL_Control_GetPidHandle(ALL_PID_YAW_RATE);    param = ALL_PID_PARAM_KI; break;
    case 18U: pid = ALL_Control_GetPidHandle(ALL_PID_YAW_RATE);    param = ALL_PID_PARAM_KD; break;
    default:
        pid = NULL;
        break;
    }

    if (pid == NULL)
    {
        return 0U;
    }

    if (value < 0.0f)
    {
        value = 0.0f;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    switch (param)
    {
    case ALL_PID_PARAM_KP:
        pid->kp = value;
        break;
    case ALL_PID_PARAM_KI:
        pid->ki = value;
        break;
    case ALL_PID_PARAM_KD:
        pid->kd = value;
        break;
    default:
        break;
    }

    // 在线调参后重置 PID 状态，避免积分/微分状态突变
    PID_Reset(pid);

    if (primask == 0U)
    {
        __enable_irq();
    }

    return 1U;
}











//将输入值限制在指定范围内
static uint16_t ClampPulse(uint16_t value, uint16_t min_value, uint16_t max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

//将输入值限制在指定范围内
static float ClampFloat(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

//将PPM输入值映射到指定的输出范围内
static float MapPpmToFloat(uint16_t input, float out_min, float out_max)
{
    float clamped = (float)ClampPulse(input, PPM_MIN_US, PPM_MAX_US);
    float span_in = (float)(PPM_MAX_US - PPM_MIN_US);
    float span_out = out_max - out_min;

    return out_min + ((clamped - (float)PPM_MIN_US) * span_out / span_in);
}

//应用PPM死区，避免在中立附近产生小的输入值导致飞控系统产生不必要的响应
static uint16_t ApplyPpmDeadband(uint16_t input)
{
    uint16_t clamped = ClampPulse(input, PPM_MIN_US, PPM_MAX_US);
    if (clamped > PPM_MID_US)
    {
        if ((clamped - PPM_MID_US) <= PPM_DEADBAND_US)
        {
            return PPM_MID_US;
        }
    }
    else
    {
        if ((PPM_MID_US - clamped) <= PPM_DEADBAND_US)
        {
            return PPM_MID_US;
        }
    }

    return clamped;
}

//初始化PID控制器参数和状态
void ALL_Control_Init(void)
{
    //角度环PID参数
    PID_Init(&g_pid_roll_angle  , 2.5f  , 0.0f  , 0.0f  , -ROLL_RATE_MAX_DPS , ROLL_RATE_MAX_DPS  );
    PID_Init(&g_pid_pitch_angle , 2.5f  , 0.0f  , 0.0f  , -PITCH_RATE_MAX_DPS, PITCH_RATE_MAX_DPS);
    PID_Init(&g_pid_yaw_angle   , 1.0f  , 0.0f  , 0.0f  , -YAW_RATE_MAX_DPS, YAW_RATE_MAX_DPS);

    //角速度环PID参数
    PID_Init(&g_pid_roll_rate   , 0.3f , 0.0f  , 0.0f  , PID_OUTPUT_MIN     , PID_OUTPUT_MAX     );
    PID_Init(&g_pid_pitch_rate  , 0.3f , 0.0f  , 0.0f  , PID_OUTPUT_MIN     , PID_OUTPUT_MAX     );
    PID_Init(&g_pid_yaw_rate    , 0.2f , 0.0f  , 0.0f  , PID_OUTPUT_MIN     , PID_OUTPUT_MAX     );

    
    //设置PID微分滤波系数，值越小滤波效果越强，值为1表示不使用滤波
    PID_SetDerivativeFilterAlpha(&g_pid_roll_rate   , 0.8f      );
    PID_SetDerivativeFilterAlpha(&g_pid_pitch_rate  , 0.8f      );
    PID_SetDerivativeFilterAlpha(&g_pid_yaw_rate    , 0.8f      );

    g_yaw_target = yawDeg;  //将当前偏航角作为初始目标偏航角，避免启动时产生大的偏航误差
    g_control_ready = 1U;   //标记飞控算法准备就绪，可以开始控制循环
}

void ALL_Control_Task(void)
{


    if (g_control_ready == 0U)
    {
        ALL_Control_Init();
    }

    throttle = ClampPulse(PPM_Databuf[2], PPM_MIN_US, PPM_MAX_US);

    
    {
        yaw_ppm = ClampPulse(PPM_Databuf[3], PPM_MIN_US, PPM_MAX_US);
        yaw_thresh_high = (uint16_t)(PPM_MID_US + (uint16_t)((PPM_MAX_US - PPM_MID_US) * SCALE_PERCENT_MAX));
        yaw_thresh_low  = (uint16_t)(PPM_MID_US - (uint16_t)((PPM_MAX_US - PPM_MID_US) * SCALE_PERCENT_MAX));

        if (ESC_lock == 0U)
        {   
            //解锁条件：油门在解锁阈值以下，且偏航摇杆向左（小于低阈值）持续达到计数阈值
            if (throttle <= THROTTLE_ARM_US && yaw_ppm <= yaw_thresh_low)
            {
                g_unlock_count++;
            }
            else
            {
                g_unlock_count = 0U;
            }

            //锁定条件：油门在解锁阈值以下，且偏航摇杆向右（大于高阈值）持续达到计数阈值
            if (throttle <= THROTTLE_ARM_US && yaw_ppm >= yaw_thresh_high)
            {
                g_lock_count++;
            }
            else
            {
                g_lock_count = 0U;
            }

            if (g_unlock_count > LOCK_COUNT_THRESHOLD)
            {
                ESC_lock = 1U;
                g_lock_count = 0U;
                g_unlock_count = 0U;
                
                //解锁时重置PID状态，避免由于长时间锁定导致的积分累积和微分突变问题
                PID_Reset(&g_pid_roll_rate);
                PID_Reset(&g_pid_pitch_rate);
                PID_Reset(&g_pid_yaw_rate);
                g_yaw_target = yawDeg;
            }
            else
            {
                
                ESC_SetChannelsUs(PPM_MIN_US, PPM_MIN_US, PPM_MIN_US, PPM_MIN_US);
                return;
            }
        }
        else 
        {   
            
            //锁定条件：油门在解锁阈值以下，且偏航摇杆向右（大于高阈值）持续达到计数阈值
            if (throttle <= THROTTLE_ARM_US && yaw_ppm >= yaw_thresh_high)
            {
                g_lock_count++;
            }
            else
            {
                g_lock_count = 0U;
            }

            if (g_lock_count > LOCK_COUNT_THRESHOLD)
            {
                ESC_lock = 0U;
                g_lock_count = 0U;
                g_unlock_count = 0U;
                PID_Reset(&g_pid_roll_angle);
                PID_Reset(&g_pid_pitch_angle);
                PID_Reset(&g_pid_yaw_angle);
                PID_Reset(&g_pid_roll_rate);
                PID_Reset(&g_pid_pitch_rate);
                PID_Reset(&g_pid_yaw_rate);
                g_yaw_target = yawDeg;
                ESC_SetChannelsUs(PPM_MIN_US, PPM_MIN_US, PPM_MIN_US, PPM_MIN_US);
                return;
            }
        }
    }
    
    
    // ---------------------------------------------------------------

    roll_set        = MapPpmToFloat(ApplyPpmDeadband(PPM_Databuf[0])    , -ROLL_ANGLE_MAX_DEG   , ROLL_ANGLE_MAX_DEG    );
    pitch_set       = MapPpmToFloat(ApplyPpmDeadband(PPM_Databuf[1])    , -PITCH_ANGLE_MAX_DEG  , PITCH_ANGLE_MAX_DEG   );
    yaw_rate_set    = MapPpmToFloat(ApplyPpmDeadband(PPM_Databuf[3])    , -YAW_RATE_MAX_DPS     , YAW_RATE_MAX_DPS      );

    g_yaw_target += yaw_rate_set * CONTROL_DT_SEC;
    yaw_error = g_yaw_target - yawDeg;

    if (yaw_error > 180.0f)
    {
        g_yaw_target -= 360.0f;
    }
    else if (yaw_error < -180.0f)
    {
        g_yaw_target += 360.0f;
    }

    //角度PID计算得到角速度设定值
    roll_rate_set   = PID_Update(&g_pid_roll_angle   , roll_set      , rollDeg   , CONTROL_DT_SEC    );
    pitch_rate_set  = PID_Update(&g_pid_pitch_angle  , pitch_set     , pitchDeg  , CONTROL_DT_SEC    );
    yaw_rate_set    = PID_Update(&g_pid_yaw_angle    , g_yaw_target  , yawDeg    , CONTROL_DT_SEC    );

    roll_rate       = (-MPU_FilteredData.GyroY) * ICM42688_GYRO_DPS_PER_LSB;
    pitch_rate      = (MPU_FilteredData.GyroX)  * ICM42688_GYRO_DPS_PER_LSB;
    yaw_rate        = (MPU_FilteredData.GyroZ)  * ICM42688_GYRO_DPS_PER_LSB;

    //角速度PID计算得到最终的控制输出
    roll_out        = PID_Update(&g_pid_roll_rate   , roll_rate_set     , roll_rate     , CONTROL_DT_SEC    );
    pitch_out       = PID_Update(&g_pid_pitch_rate  , pitch_rate_set    , pitch_rate    , CONTROL_DT_SEC    );
    yaw_out         = PID_Update(&g_pid_yaw_rate    , yaw_rate_set      , yaw_rate      , CONTROL_DT_SEC    );

    roll_out        = ClampFloat(roll_out   , PID_OUTPUT_MIN    , PID_OUTPUT_MAX    );
    pitch_out       = ClampFloat(pitch_out  , PID_OUTPUT_MIN    , PID_OUTPUT_MAX    );
    yaw_out         = ClampFloat(yaw_out    , PID_OUTPUT_MIN    , PID_OUTPUT_MAX    );

    
    
    
    {
        float motor1 = (float)throttle + pitch_out - roll_out - yaw_out;
        float motor2 = (float)throttle - pitch_out - roll_out + yaw_out;
        float motor3 = (float)throttle - pitch_out + roll_out - yaw_out;
        float motor4 = (float)throttle + pitch_out + roll_out + yaw_out;

        ESC_SetChannelsUs(ClampPulse((uint16_t)motor1, PPM_MIN_US, PPM_MAX_US),
                          ClampPulse((uint16_t)motor2, PPM_MIN_US, PPM_MAX_US),
                          ClampPulse((uint16_t)motor3, PPM_MIN_US, PPM_MAX_US),
                          ClampPulse((uint16_t)motor4, PPM_MIN_US, PPM_MAX_US));
    }
    
    
}



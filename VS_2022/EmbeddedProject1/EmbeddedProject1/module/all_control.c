#include "all_control.h"
#include "pid.h"
#include "pid_flash_store.h"
#include "esc_calibration.h"
#include "ppm_input.h"
#include "FFCY_NEW_ICM42688.h"

#include "stm32f4xx.h"

#include <stddef.h>


#define IS_ALL_Control_ApplyPidGains 1      // flash_PID，1=应用 0=不应用（仅用于调试）

//PPM_Databuf[0] roll
//PPM_Databuf[1] pitch
//PPM_Databuf[2] throttle
//PPM_Databuf[3] yaw

uint16_t yaw_thresh_low; //偏航角阈值低值，单位微秒
uint16_t yaw_thresh_high; //偏航角阈值高值，单位微秒
uint16_t yaw_ppm; //偏航角PPM输入值，单位微秒

uint16_t throttle;
uint16_t throttle_run;

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
#define PPM_D                 (100U)
#define PPM_MIN_RIN_US        (1050U)   //PPM输入的最小有效脉宽，单位微秒，低于此值视为无效输入
#define PPM_MID_US            (1500U)   //PPM输入的中立脉宽，单位微秒，通常为1500us
#define PPM_DEADBAND_US       (20U)     //遥控器死区范围
#define THROTTLE_ARM_US       (1100U)   //油门解锁的脉宽阈值，单位微秒
#define PPM_5_MID             (1500U)   //LOCK
#define CONTROL_DT_SEC        (0.005f)  //控制循环的时间间隔 5ms

#define ROLL_ANGLE_MAX_DEG    (10.0f)   //最大横滚角度，单位度
#define PITCH_ANGLE_MAX_DEG   (10.0f)   //最大俯仰角度，单位度

#define YAW_RATE_MAX_DPS      (30.0f)   //最大偏航角速度，单位度每秒
#define ROLL_RATE_MAX_DPS     (270.0f)  //最大横滚角速度，单位度每秒
#define PITCH_RATE_MAX_DPS    (270.0f)  //最大俯仰角速度，单位度每秒

#define ICM42688_GYRO_DPS_PER_LSB    (1.0f / 16.4f) //ICM42688陀螺仪每个LSB对应的角速度，单位度每秒

#define PID_OUTPUT_MIN        (-400.0f) //PID输出最小值
#define PID_OUTPUT_MAX        (400.0f)  //PID输出最大值

#define YAW_PID_OUTPUT_MAX    ( 400.0f)
#define YAW_PID_OUTPUT_MIX    (-400.0f)

// 通过 PPM_Databuf[6] 切换控制模式：
// <1090  角速度 + 角度(自稳)
// >1900  角速度(手动/ACRO)
#define MODE_SWITCH_LOW_US    (1000U)
#define MODE_SWITCH_MID_US    (1500U)
#define MODE_SWITCH_HIGH_US   (2000U)
#define MODE_SWITCH_D         (100U)

typedef enum
{
    ALL_CONTROL_MODE_ANGLE_RATE = 0U,
    ALL_CONTROL_MODE_RATE_ONLY  = 1U
} ALL_ControlMode_t;

static PID_Handle_t g_pid_roll_angle;//
static PID_Handle_t g_pid_pitch_angle;
static PID_Handle_t g_pid_yaw_angle;

static PID_Handle_t g_pid_roll_rate;
static PID_Handle_t g_pid_pitch_rate;
static PID_Handle_t g_pid_yaw_rate;

static float g_yaw_target = 0.0f;//偏航目标角度
static uint8_t g_control_ready = 0U;

// PID增益修改标志和计时器，用于延迟保存到Flash，避免频繁写入影响性能和Flash寿命
static uint8_t g_pidFlashDirty = 0U;        // 0=干净，1=有修改未保存
static uint16_t g_pidFlashDirtyTicks = 0U;  // 以控制循环周期为单位的计时器，达到一定值后才允许写入Flash

#define PID_FLASH_SAVE_DELAY_TICKS (400U)   // 400 * 5ms = 2s// 修改PID增益后需要等待2秒（400个控制周期）才会保存到Flash，这期间如果再次修改会重置计时器

static ALL_ControlMode_t g_control_mode = ALL_CONTROL_MODE_ANGLE_RATE;

//================================
#define SCALE_PERCENT_MAX     (0.8f)    //用于计算摇杆（解锁/锁定）阈值的比例，0.9表示需要达到摇杆范围的90%才能触发（解锁/锁定）动作
#define LOCK_COUNT_THRESHOLD  (20U)    //用于计算摇杆（解锁/锁定）阈值的计数阈值 0.2s
static uint16_t g_lock_count = 0U;      //锁定计数器
static uint16_t g_unlock_count = 0U;    //解锁计数器

static uint8_t  ESC_lock = 1U;          // ESC_lock: 1=锁定(电机停转)    0=解锁(允许输出)

// 计算摇杆（解锁/锁定）阈值，基于PPM输入范围和设定的比例
static void ALL_Control_CollectPidGains(PID_FlashGains_t gains[ALL_PID_COUNT])// 从当前PID句柄中收集增益值，准备保存到Flash
{
    gains[ALL_PID_ROLL_ANGLE].kp = g_pid_roll_angle.kp;
    gains[ALL_PID_ROLL_ANGLE].ki = g_pid_roll_angle.ki;
    gains[ALL_PID_ROLL_ANGLE].kd = g_pid_roll_angle.kd;

    // Roll/Pitch 作为一组调参：Flash 中也使用同一套增益，避免读写后不一致
    gains[ALL_PID_PITCH_ANGLE].kp = g_pid_roll_angle.kp;
    gains[ALL_PID_PITCH_ANGLE].ki = g_pid_roll_angle.ki;
    gains[ALL_PID_PITCH_ANGLE].kd = g_pid_roll_angle.kd;

    gains[ALL_PID_YAW_ANGLE].kp = g_pid_yaw_angle.kp;
    gains[ALL_PID_YAW_ANGLE].ki = g_pid_yaw_angle.ki;
    gains[ALL_PID_YAW_ANGLE].kd = g_pid_yaw_angle.kd;

    gains[ALL_PID_ROLL_RATE].kp = g_pid_roll_rate.kp;
    gains[ALL_PID_ROLL_RATE].ki = g_pid_roll_rate.ki;
    gains[ALL_PID_ROLL_RATE].kd = g_pid_roll_rate.kd;

    gains[ALL_PID_PITCH_RATE].kp = g_pid_roll_rate.kp;
    gains[ALL_PID_PITCH_RATE].ki = g_pid_roll_rate.ki;
    gains[ALL_PID_PITCH_RATE].kd = g_pid_roll_rate.kd;

    gains[ALL_PID_YAW_RATE].kp = g_pid_yaw_rate.kp;
    gains[ALL_PID_YAW_RATE].ki = g_pid_yaw_rate.ki;
    gains[ALL_PID_YAW_RATE].kd = g_pid_yaw_rate.kd;
}

// 将从Flash加载的增益应用到当前的PID句柄，并重置PID状态，确保新增益立即生效且不会受到之前积分状态的影响
static void ALL_Control_ApplyPidGains(const PID_FlashGains_t gains[ALL_PID_COUNT])
{
    g_pid_roll_angle.kp = gains[ALL_PID_ROLL_ANGLE].kp;
    g_pid_roll_angle.ki = gains[ALL_PID_ROLL_ANGLE].ki;
    g_pid_roll_angle.kd = gains[ALL_PID_ROLL_ANGLE].kd;

    // Roll/Pitch 作为一组调参：使用同一套（Roll）增益应用到 Pitch
    g_pid_pitch_angle.kp = gains[ALL_PID_ROLL_ANGLE].kp;
    g_pid_pitch_angle.ki = gains[ALL_PID_ROLL_ANGLE].ki;
    g_pid_pitch_angle.kd = gains[ALL_PID_ROLL_ANGLE].kd;

    g_pid_yaw_angle.kp = gains[ALL_PID_YAW_ANGLE].kp;
    g_pid_yaw_angle.ki = gains[ALL_PID_YAW_ANGLE].ki;
    g_pid_yaw_angle.kd = gains[ALL_PID_YAW_ANGLE].kd;

    g_pid_roll_rate.kp = gains[ALL_PID_ROLL_RATE].kp;
    g_pid_roll_rate.ki = gains[ALL_PID_ROLL_RATE].ki;
    g_pid_roll_rate.kd = gains[ALL_PID_ROLL_RATE].kd;

    g_pid_pitch_rate.kp = gains[ALL_PID_ROLL_RATE].kp;
    g_pid_pitch_rate.ki = gains[ALL_PID_ROLL_RATE].ki;
    g_pid_pitch_rate.kd = gains[ALL_PID_ROLL_RATE].kd;

    g_pid_yaw_rate.kp = gains[ALL_PID_YAW_RATE].kp;
    g_pid_yaw_rate.ki = gains[ALL_PID_YAW_RATE].ki;
    g_pid_yaw_rate.kd = gains[ALL_PID_YAW_RATE].kd;

    PID_Reset(&g_pid_roll_angle);
    PID_Reset(&g_pid_pitch_angle);
    PID_Reset(&g_pid_yaw_angle);
    PID_Reset(&g_pid_roll_rate);
    PID_Reset(&g_pid_pitch_rate);
    PID_Reset(&g_pid_yaw_rate);
}


// PID增益保存到Flash的服务函数，定期检查是否需要保存
static void ALL_Control_PidFlashService(void)
{
    if (g_pidFlashDirty == 0U)
    {
        return;
    }

    if (g_pidFlashDirtyTicks < PID_FLASH_SAVE_DELAY_TICKS)
    {
        g_pidFlashDirtyTicks++;
        return;
    }

    // Only write flash when motors are locked to avoid disturbing control loop timing.
    if (ESC_lock != 1U)
    {
        return;
    }

    PID_FlashGains_t gains[ALL_PID_COUNT];
    ALL_Control_CollectPidGains(gains);

    if (PID_FlashStore_Save(gains) != 0U)
    {
        g_pidFlashDirty = 0U;
        g_pidFlashDirtyTicks = 0U;
    }
}




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

// 通过上位机滑动条在线调参：sliderId → PID 参数映射
//  1-3   ：Roll 和 Pitch 角度环 kp/ki/kd
//  4-6   ：Yaw 角度环 kp/ki/kd
//  7-9   ：Roll 和 Pitch 角速度环 kp/ki/kd
//  10-12 ：Yaw 角速度环 kp/ki/kd
uint8_t ALL_Control_TunePidBySlider(uint32_t sliderId, float value)
{
    PID_Handle_t *pid1 = NULL;
    PID_Handle_t *pid2 = NULL;
    ALL_PidParam_t param = ALL_PID_PARAM_KP;

    switch (sliderId)
    {
    // 1-3：Roll & Pitch angle
    case 1U: param = ALL_PID_PARAM_KP; pid1 = ALL_Control_GetPidHandle(ALL_PID_ROLL_ANGLE);  pid2 = ALL_Control_GetPidHandle(ALL_PID_PITCH_ANGLE); break;
    case 2U: param = ALL_PID_PARAM_KI; pid1 = ALL_Control_GetPidHandle(ALL_PID_ROLL_ANGLE);  pid2 = ALL_Control_GetPidHandle(ALL_PID_PITCH_ANGLE); break;
    case 3U: param = ALL_PID_PARAM_KD; pid1 = ALL_Control_GetPidHandle(ALL_PID_ROLL_ANGLE);  pid2 = ALL_Control_GetPidHandle(ALL_PID_PITCH_ANGLE); break;

    // 4-6：Yaw angle
    case 4U: param = ALL_PID_PARAM_KP; pid1 = ALL_Control_GetPidHandle(ALL_PID_YAW_ANGLE); break;
    case 5U: param = ALL_PID_PARAM_KI; pid1 = ALL_Control_GetPidHandle(ALL_PID_YAW_ANGLE); break;
    case 6U: param = ALL_PID_PARAM_KD; pid1 = ALL_Control_GetPidHandle(ALL_PID_YAW_ANGLE); break;

    // 7-9：Roll & Pitch rate
    case 7U: param = ALL_PID_PARAM_KP; pid1 = ALL_Control_GetPidHandle(ALL_PID_ROLL_RATE);   pid2 = ALL_Control_GetPidHandle(ALL_PID_PITCH_RATE); break;
    case 8U: param = ALL_PID_PARAM_KI; pid1 = ALL_Control_GetPidHandle(ALL_PID_ROLL_RATE);   pid2 = ALL_Control_GetPidHandle(ALL_PID_PITCH_RATE); break;
    case 9U: param = ALL_PID_PARAM_KD; pid1 = ALL_Control_GetPidHandle(ALL_PID_ROLL_RATE);   pid2 = ALL_Control_GetPidHandle(ALL_PID_PITCH_RATE); break;

    // 10-12：Yaw rate
    case 10U: param = ALL_PID_PARAM_KP; pid1 = ALL_Control_GetPidHandle(ALL_PID_YAW_RATE); break;
    case 11U: param = ALL_PID_PARAM_KI; pid1 = ALL_Control_GetPidHandle(ALL_PID_YAW_RATE); break;
    case 12U: param = ALL_PID_PARAM_KD; pid1 = ALL_Control_GetPidHandle(ALL_PID_YAW_RATE); break;

    default:
        break;
    }

    if (pid1 == NULL)
    {
        return 0U;
    }

    if (value < 0.0f)
    {
        value = 0.0f;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    {
        PID_Handle_t *pids[2] = { pid1, pid2 };
        for (uint32_t i = 0U; i < 2U; i++)
        {
            PID_Handle_t *pid = pids[i];
            if (pid == NULL)
            {
                continue;
            }

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
        }
    }

    
    // 标记需要保存到 Flash（延迟保存，避免频繁擦写）
    g_pidFlashDirty = 1U;
    g_pidFlashDirtyTicks = 0U;

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
    PID_Init(&g_pid_roll_angle  , 4.89f , 0.92f, 0.26f, -ROLL_RATE_MAX_DPS, ROLL_RATE_MAX_DPS);
    PID_Init(&g_pid_pitch_angle , 4.89f , 0.92f, 0.26f, -PITCH_RATE_MAX_DPS, PITCH_RATE_MAX_DPS);
    PID_Init(&g_pid_yaw_angle   , 3.50f , 0.0f  ,1.00f  , -YAW_RATE_MAX_DPS, YAW_RATE_MAX_DPS);

    //角速度环PID参数
    PID_Init(&g_pid_roll_rate   , 0.40f  , 0.47f     , 0.03f     , PID_OUTPUT_MIN        , PID_OUTPUT_MAX       );
    PID_Init(&g_pid_pitch_rate  , 0.40f  , 0.47f     , 0.03f     , PID_OUTPUT_MIN        , PID_OUTPUT_MAX       );
    PID_Init(&g_pid_yaw_rate    , 1.50f  , 3.00f     , 0.00f     , YAW_PID_OUTPUT_MIX    , YAW_PID_OUTPUT_MAX   );

    
    //设置PID微分滤波系数，值越小滤波效果越强，值为1表示不使用滤波
    PID_SetDerivativeFilterAlpha(&g_pid_roll_rate   , 1.0f      );
    PID_SetDerivativeFilterAlpha(&g_pid_pitch_rate  , 1.0f      );
    PID_SetDerivativeFilterAlpha(&g_pid_yaw_rate    , 1.0f      );

    
    // 从 Flash 加载上一次保存的 PID 参数（若无有效数据则使用默认值）
    {
        PID_FlashGains_t gains[ALL_PID_COUNT];
        if (PID_FlashStore_Load(gains) != 0U && IS_ALL_Control_ApplyPidGains==1 )
        {
            ALL_Control_ApplyPidGains(gains);
        }
    }

    g_yaw_target = yawDeg;  //将当前偏航角作为初始目标偏航角，避免启动时产生大的偏航误差
    g_control_ready = 1U;   //标记飞控算法准备就绪，可以开始控制循环
}

void ALL_Control_Task(void)
{


    if (g_control_ready == 0U)
    {
        ALL_Control_Init();
    }
    
    #include "oled.h"
    if (PPM_Databuf[0] == 0)
    {
        OLED_Printf(80, 0, 8, 1, "NO_PPM");
        ESC_lock = 1;
        return;
    }


    throttle = ClampPulse(PPM_Databuf[2], PPM_MIN_US, PPM_MAX_US);

    // 模式切换：中间区域保持当前模式(避免抖动)
    {
        uint16_t mode_ppm = ClampPulse(PPM_Databuf[4], PPM_MIN_US, PPM_MAX_US);
        ALL_ControlMode_t prevMode = g_control_mode;

        if (   ((MODE_SWITCH_LOW_US + MODE_SWITCH_D) > mode_ppm) && (mode_ppm > (MODE_SWITCH_LOW_US - MODE_SWITCH_D))   )
        {
            g_control_mode = ALL_CONTROL_MODE_RATE_ONLY;
        }
        else if (   ((MODE_SWITCH_MID_US + MODE_SWITCH_D) > mode_ppm) && (mode_ppm > (MODE_SWITCH_MID_US - MODE_SWITCH_D))   )
        {
            g_control_mode = ALL_CONTROL_MODE_ANGLE_RATE;
        }
        else if (   ((MODE_SWITCH_HIGH_US + MODE_SWITCH_D) > mode_ppm) && (mode_ppm > (MODE_SWITCH_HIGH_US - MODE_SWITCH_D))   )
        {
            g_control_mode = ALL_CONTROL_MODE_ANGLE_RATE;
        }
        
        if (g_control_mode != prevMode)
        {
            // 切换模式时重置 PID，避免状态突变
            PID_Reset(&g_pid_roll_angle);
            PID_Reset(&g_pid_pitch_angle);
            PID_Reset(&g_pid_yaw_angle);
            PID_Reset(&g_pid_roll_rate);
            PID_Reset(&g_pid_pitch_rate);
            PID_Reset(&g_pid_yaw_rate);
            g_yaw_target = yawDeg;
        }
    }

    
    {
        yaw_ppm = ClampPulse(PPM_Databuf[3], PPM_MIN_US, PPM_MAX_US);
        yaw_thresh_high = (uint16_t)(PPM_MID_US + (uint16_t)((PPM_MAX_US - PPM_MID_US) * SCALE_PERCENT_MAX));
        yaw_thresh_low  = (uint16_t)(PPM_MID_US - (uint16_t)((PPM_MAX_US - PPM_MID_US) * SCALE_PERCENT_MAX));

        
        if (ESC_lock == 1U) // 当前处于锁定状态
        {
            // 解锁条件
            if ( PPM_Databuf[5] < PPM_5_MID )
            {                 
                
                if ( PPM_Databuf[2] <  (PPM_MIN_US + PPM_D) )
                {
                    g_unlock_count++;
                    
                }
                
            }
            else
            {
                g_unlock_count = 0U;
            }
            g_lock_count = 0U;

            if (g_unlock_count > LOCK_COUNT_THRESHOLD)
            {
                ESC_lock = 0U;
                g_unlock_count = 0U;

                // 解锁时重置 PID 状态，避免积分累积/微分突变
                PID_Reset(&g_pid_roll_angle);
                PID_Reset(&g_pid_pitch_angle);
                PID_Reset(&g_pid_yaw_angle);
                PID_Reset(&g_pid_roll_rate);
                PID_Reset(&g_pid_pitch_rate);
                PID_Reset(&g_pid_yaw_rate);
                g_yaw_target = yawDeg;
            }
        
            // 锁定时强制电机最小，并直接返回
            ESC_SetChannelsUs(PPM_MIN_US, PPM_MIN_US, PPM_MIN_US, PPM_MIN_US);
            throttle_run = PPM_MIN_US;
            
        }
        else if(ESC_lock == 0) // ESC_lock == 0U 当前处于解锁状态
        {
            throttle_run = throttle;

            // 锁定条件：油门低 + 偏航右
            if (PPM_Databuf[5] > PPM_5_MID)
            {
                g_lock_count++;
            }
            else
            {
                g_lock_count = 0U;
            }
            g_unlock_count = 0U;

            if (g_lock_count > LOCK_COUNT_THRESHOLD)
            {
                ESC_lock = 1U;
                g_lock_count = 0U;

                PID_Reset(&g_pid_roll_angle);
                PID_Reset(&g_pid_pitch_angle);
                PID_Reset(&g_pid_yaw_angle);
                PID_Reset(&g_pid_roll_rate);
                PID_Reset(&g_pid_pitch_rate);
                PID_Reset(&g_pid_yaw_rate);
                g_yaw_target = yawDeg;
                ESC_SetChannelsUs(PPM_MIN_US, PPM_MIN_US, PPM_MIN_US, PPM_MIN_US);
                throttle_run = PPM_MIN_US;
                
            }
        }
    }
    
    // ---------------------------------------------------------------
    if (g_control_mode == ALL_CONTROL_MODE_ANGLE_RATE)//自稳模式
    {
        // 自稳：摇杆 -> 角度目标 -> 角速度目标
        roll_set  = - MapPpmToFloat(ApplyPpmDeadband(PPM_Databuf[0]), -ROLL_ANGLE_MAX_DEG,  ROLL_ANGLE_MAX_DEG);
        pitch_set = MapPpmToFloat(ApplyPpmDeadband(PPM_Databuf[1]), -PITCH_ANGLE_MAX_DEG, PITCH_ANGLE_MAX_DEG);

        // 偏航：摇杆给“角速度”，积分成角度目标(偏航保持)
        {
            float yaw_rate_cmd = MapPpmToFloat(ApplyPpmDeadband(PPM_Databuf[3]), -YAW_RATE_MAX_DPS, YAW_RATE_MAX_DPS);
            g_yaw_target += yaw_rate_cmd * CONTROL_DT_SEC;
        }

        
        yaw_error = g_yaw_target - yawDeg;
        if (yaw_error > 180.0f)
        {
            g_yaw_target -= 360.0f;
        }
        else if (yaw_error < -180.0f)
        {
            g_yaw_target += 360.0f;
        }

        // 角度 PID 计算得到角速度设定值
        roll_rate_set  = PID_Update(&g_pid_roll_angle,  roll_set,     rollDeg,  CONTROL_DT_SEC);
        pitch_rate_set = PID_Update(&g_pid_pitch_angle, pitch_set,    pitchDeg, CONTROL_DT_SEC);
        yaw_rate_set   = PID_Update(&g_pid_yaw_angle,   g_yaw_target, yawDeg,   CONTROL_DT_SEC);
        //yaw_rate_set   = MapPpmToFloat(ApplyPpmDeadband(PPM_Databuf[3]), -YAW_RATE_MAX_DPS, YAW_RATE_MAX_DPS);
    }
    else if(g_control_mode == ALL_CONTROL_MODE_RATE_ONLY)// ALL_CONTROL_MODE_RATE_ONLY 手动/ACRO模式
    {
        // 角速度：摇杆直接给角速度
        roll_rate_set  = - MapPpmToFloat(ApplyPpmDeadband(PPM_Databuf[0]), -ROLL_RATE_MAX_DPS,  ROLL_RATE_MAX_DPS);
        pitch_rate_set = MapPpmToFloat(ApplyPpmDeadband(PPM_Databuf[1]), -PITCH_RATE_MAX_DPS, PITCH_RATE_MAX_DPS);
        yaw_rate_set   = MapPpmToFloat(ApplyPpmDeadband(PPM_Databuf[3]), -YAW_RATE_MAX_DPS,   YAW_RATE_MAX_DPS);

        // 角速度模式下保持 yaw_target 跟随当前 yaw，方便切回自稳不跳变
        g_yaw_target = yawDeg;
    }

    
    roll_rate       = (-CH_FT_MPU_FilteredData.GyroY) * ICM42688_GYRO_DPS_PER_LSB;
    pitch_rate      = (CH_FT_MPU_FilteredData.GyroX)  * ICM42688_GYRO_DPS_PER_LSB;
    yaw_rate        = (CH_FT_MPU_FilteredData.GyroZ)  * ICM42688_GYRO_DPS_PER_LSB;

        //角速度PID计算得到最终的控制输出
        roll_out        = PID_Update(&g_pid_roll_rate, roll_rate_set, roll_rate, CONTROL_DT_SEC);
        pitch_out       = PID_Update(&g_pid_pitch_rate, pitch_rate_set, pitch_rate, CONTROL_DT_SEC);
        yaw_out         = PID_Update(&g_pid_yaw_rate, yaw_rate_set, yaw_rate, CONTROL_DT_SEC);

        roll_out        = ClampFloat(roll_out, PID_OUTPUT_MIN, PID_OUTPUT_MAX);
        pitch_out       = ClampFloat(pitch_out, PID_OUTPUT_MIN, PID_OUTPUT_MAX);
        yaw_out         = ClampFloat(yaw_out, PID_OUTPUT_MIN, PID_OUTPUT_MAX);

    
    
        if (ESC_lock == 1U) // 当前处于锁定状态
        {
            ESC_SetChannelsUs(PPM_MIN_US, PPM_MIN_US, PPM_MIN_US, PPM_MIN_US);
        }
        else // ESC_lock == 0U 当前处于解锁状态
        {
            float motor1 = (float)throttle_run + pitch_out - roll_out + yaw_out;
            float motor2 = (float)throttle_run - pitch_out - roll_out - yaw_out;
            float motor3 = (float)throttle_run - pitch_out + roll_out + yaw_out;
            float motor4 = (float)throttle_run + pitch_out + roll_out - yaw_out;

            ESC_SetChannelsUs(  ClampPulse((uint16_t)motor1, PPM_MIN_RIN_US, PPM_MAX_US),
                                ClampPulse((uint16_t)motor2, PPM_MIN_RIN_US, PPM_MAX_US),
                                ClampPulse((uint16_t)motor3, PPM_MIN_RIN_US, PPM_MAX_US),
                                ClampPulse((uint16_t)motor4, PPM_MIN_RIN_US, PPM_MAX_US));
        }

    //在控制循环中调用PID参数保存服务函数，检查是否需要将修改后的PID参数保存到Flash中
    ALL_Control_PidFlashService(); 
    
    
    
    
}



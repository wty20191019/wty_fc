#include "board.h"
#include "bsp_uart.h"
#include "i2c1.h"
#include "FFCY_NEW_ICM42688.h"
#include "Attitude6Axis.h"
#include <stdio.h>
#include "oled.h"
#include "pwm_tim3.h"
#include "tim2_scheduler.h"
#include "pa0_LED_toggle.h"

#define IMU_TASK_PERIOD_MS           (100U)
#define ATT6_DT_SEC                  ((float)IMU_TASK_PERIOD_MS / 1000.0f)
#define ICM42688_ACC_G_PER_LSB       (1.0f / 8192.0f)
#define ICM42688_GYRO_DPS_PER_LSB    (1.0f / 16.4f)

static Attitude6AxisState g_attitude;

static void FormatAngleLine(char *line, size_t lineSize, const char *name, float angleDeg)
{
    int32_t scaled = (int32_t)(angleDeg * 10.0f);
    int32_t absScaled = (scaled < 0) ? (-scaled) : scaled;

    snprintf(line, lineSize, "%s:%c%3ld.%1ld", name, (scaled < 0) ? '-' : '+', (long)(absScaled / 10), (long)(absScaled % 10));
}

static void Task_ImuOledUpdate(void)
{
    char line[22];
    float pitchDeg;
    float rollDeg;
    float yawDeg;

    ImuSensor_ReadReg_BuffAll();

    Attitude6Axis_UpdateRaw(
        &g_attitude,
        -MPU_Data.AccY,
        MPU_Data.AccX,
        MPU_Data.AccZ,
        -MPU_Data.GyroY,
        MPU_Data.GyroX,
        MPU_Data.GyroZ,
        ICM42688_ACC_G_PER_LSB,
        ICM42688_GYRO_DPS_PER_LSB,
        ATT6_DT_SEC);

    Attitude6Axis_GetEulerDeg(&g_attitude, &pitchDeg, &rollDeg, &yawDeg);

    OLED_Clear();

    snprintf(line, sizeof(line), "ATTITUDE 6AX");
    OLED_ShowString(0, 0, (uint8_t *)line, 8, 1);

    FormatAngleLine(line, sizeof(line), "P", pitchDeg);
    OLED_ShowString(0, 8, (uint8_t *)line, 8, 1);

    FormatAngleLine(line, sizeof(line), "R", rollDeg);
    OLED_ShowString(0, 16, (uint8_t *)line, 8, 1);

    FormatAngleLine(line, sizeof(line), "Y", yawDeg);
    OLED_ShowString(0, 24, (uint8_t *)line, 8, 1);

    snprintf(line, sizeof(line), "AX:%6d", MPU_Data.AccX);
    OLED_ShowString(0, 32, (uint8_t *)line, 8, 1);

    snprintf(line, sizeof(line), "AY:%6d", MPU_Data.AccY);
    OLED_ShowString(0, 40, (uint8_t *)line, 8, 1);

    snprintf(line, sizeof(line), "AZ:%6d", MPU_Data.AccZ);
    OLED_ShowString(0, 48, (uint8_t *)line, 8, 1);

    snprintf(line, sizeof(line), "T :%6d", MPU_Data.Temp);
    OLED_ShowString(0, 56, (uint8_t *)line, 8, 1);

    OLED_Refresh();
}

int main(void)
{
    board_init();   //初始化系统时钟和SysTick

    PWM_Init();     //初始化TIM3的PWM输出

    uart1_init(115200U);

    ImuSensor_Init();  //初始化ICM42688
    Attitude6Axis_Init(&g_attitude, 2.0f, 0.02f);

    OLED_Init();     //初始化OLED显示屏
    OLED_Clear();

    PA0_LED_Toggle_Init(); // 初始化PA0引脚用于LED闪烁

    SCH_Init();
	//调度器====================================================================
    SCH_AddTask(PA0_LED_Toggle			, 500U		, PRIORITY_HIGH	);
    SCH_AddTask(Task_ImuOledUpdate	    , IMU_TASK_PERIOD_MS		, 1U			);
    //==========================================================================
    while (1)
    {
        SCH_Dispatch();
    }


}




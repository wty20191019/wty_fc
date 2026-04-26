#include "board.h"
#include "bsp_uart.h"
#include "i2c1.h"
#include "FFCY_NEW_ICM42688.h"
#include <stdio.h>
#include "oled.h"
#include "pwm_tim3.h"
#include "tim2_scheduler.h"
#include "pa0_LED_toggle.h"

static void Task_ImuOledUpdate(void)
{
    char line[22];

    ImuSensor_ReadReg_BuffAll();
    OLED_Clear();

    snprintf(line, sizeof(line), "ICM42688 RAW");
    OLED_ShowString(0, 0, (uint8_t *)line, 8, 1);

    snprintf(line, sizeof(line), "AX:%6d", MPU_Data.AccX);
    OLED_ShowString(0, 8, (uint8_t *)line, 8, 1);

    snprintf(line, sizeof(line), "AY:%6d", MPU_Data.AccY);
    OLED_ShowString(0, 16, (uint8_t *)line, 8, 1);

    snprintf(line, sizeof(line), "AZ:%6d", MPU_Data.AccZ);
    OLED_ShowString(0, 24, (uint8_t *)line, 8, 1);

    snprintf(line, sizeof(line), "GX:%6d", MPU_Data.GyroX);
    OLED_ShowString(0, 32, (uint8_t *)line, 8, 1);

    snprintf(line, sizeof(line), "GY:%6d", MPU_Data.GyroY);
    OLED_ShowString(0, 40, (uint8_t *)line, 8, 1);

    snprintf(line, sizeof(line), "GZ:%6d", MPU_Data.GyroZ);
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

    OLED_Init();     //初始化OLED显示屏
    OLED_Clear();

    PA0_LED_Toggle_Init(); // 初始化PA0引脚用于LED闪烁

    SCH_Init();
	//调度器====================================================================
    SCH_AddTask(PA0_LED_Toggle			, 500U		, PRIORITY_HIGH	);
    SCH_AddTask(Task_ImuOledUpdate	, 100U		, 1U			);
    //==========================================================================
    while (1)
    {
        SCH_Dispatch();
    }


}




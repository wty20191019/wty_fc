#include "board.h"

#include "DMA_UART1.h"
#include "DMA_UART2.h"
#include "DMA_UART3.h"
#include "DMA_UART4.h"
#include "DMA_UART5.h"
#include "DMA_UART6.h"

#include "i2c1.h"
#include "FFCY_NEW_ICM42688.h"
#include "Attitude6Axis.h"
#include <stdio.h>
#include "oled.h"
#include "pwm_tim3.h"
#include "tim2_scheduler.h"
#include "pa0_LED_toggle.h"

#define IMU_TASK_PERIOD_MS           (100U)                                 //IMU数据读取和OLED显示更新的周期，单位毫秒    
#define ATT6_DT_SEC                  ((float)IMU_TASK_PERIOD_MS / 1000.0f)  //姿态算法更新周期，单位秒
#define ICM42688_ACC_G_PER_LSB       (1.0f / 8192.0f)                       //ICM42688加速度计每LSB对应的重力加速度值，单位g
#define ICM42688_GYRO_DPS_PER_LSB    (1.0f / 16.4f)                         //ICM42688陀螺仪每LSB对应的角速度值，单位度每秒

static Attitude6AxisState g_attitude; //全局姿态算法状态变量


//==========================================================================
////IMU数据读取和OLED显示更新任务
//==========================================================================
static void Task_ImuOledUpdate(void)  
{
    float pitchDeg;
    float rollDeg;
    float yawDeg;


    ImuSensor_ReadReg_BuffAll();//读取ICM42688的所有相关寄存器数据到全局变量MPU_Data中

    Attitude6Axis_UpdateRaw(    //更新姿态算法状态
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

    Attitude6Axis_GetEulerDeg(&g_attitude, &pitchDeg, &rollDeg, &yawDeg);//获取欧拉角度



    OLED_Clear();//清屏====================================================

    OLED_Printf(0, 0, 8, 1, "ATTITUDE 6AX");
    OLED_Printf(0, 8, 8, 1,  "P:%+3.2f", pitchDeg);
    OLED_Printf(0, 16, 8, 1, "R:%+3.2f", rollDeg);
    OLED_Printf(0, 24, 8, 1, "Y:%+3.2f", yawDeg);
    OLED_Printf(0, 32, 8, 1, "AX:%+6d", MPU_Data.AccX);
    OLED_Printf(0, 40, 8, 1, "AY:%+6d", MPU_Data.AccY);
    OLED_Printf(0, 48, 8, 1, "AZ:%+6d", MPU_Data.AccZ);
    OLED_Printf(0, 56, 8, 1, "T :%+6d", MPU_Data.Temp);

    OLED_Refresh();//更新显示=============================================

    //Serial1_Printf("ax=%d ay=%d az=%d\r\n", MPU_Data.AccX, MPU_Data.AccY, MPU_Data.AccZ);
    Serial1_Printf("[plot,%2.2f,%2.2f,%2.2f]\r\n", pitchDeg, rollDeg, yawDeg);

}

int main(void)
{
    board_init();//初始化系统时钟和SysTick

    PWM_Init();//初始化TIM3的PWM输出

    PA0_LED_Toggle_Init();// 初始化PA0引脚用于LED闪烁

    DMA_USART1_Init(9600U);//初始化USART1用于串口调试输出，波特率9600

    
    systick_delay_ms(2000);//等待IMU稳定
    ImuSensor_Init();  //初始化ICM42688
    Attitude6Axis_Init(&g_attitude, 2.0f, 0.02f);
    


    OLED_Init();     //初始化OLED显示屏
    OLED_Clear();

    

    SCH_Init();
	//调度器==========================================================================
    SCH_AddTask(PA0_LED_Toggle			, 50U		                , 14U			);
    SCH_AddTask(Task_ImuOledUpdate	    , IMU_TASK_PERIOD_MS		, 1U			);
    //================================================================================
    while (1)
    {
        SCH_Dispatch();//调度器分发任务
    }


}




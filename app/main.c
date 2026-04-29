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
#include "esc_calibration.h"
#include "tim2_scheduler.h"
#include "pa0_LED_toggle.h"

#include <string.h>

#define ESC_AUTO_CALIBRATION         (1U)                                   //首次使用或更换电调时建议打开，按文章流程自动完成解锁/行程校准

#define IMU_TASK_PERIOD_MS           (5U)                                   //IMU数据读取和OLED显示更新的周期，单位毫秒    
#define ATT6_DT_SEC                  ((float)IMU_TASK_PERIOD_MS / 1000.0f)  //姿态算法更新周期，单位秒
#define ICM42688_ACC_G_PER_LSB       (1.0f / 8192.0f)                       //ICM42688加速度计每LSB对应的重力加速度值，单位g
#define ICM42688_GYRO_DPS_PER_LSB    (1.0f / 16.4f)                         //ICM42688陀螺仪每LSB对应的角速度值，单位度每秒
#define ATT_ZERO_CALIB_SAMPLES       (1000U)                                //启动后静止采样次数(1000*5ms=5s)
#define UART1_ECHO_BUF_SIZE          (64U)                                  //串口1回显缓冲区大小
#define UART1_PACKET_BUF_SIZE        (64U)                                  //串口1解析包缓冲区大小

static Attitude6AxisState g_attitude; //全局姿态算法状态变量

float pitchDeg;
float rollDeg;
float yawDeg;

static float g_pitch_zero = 0.0f;
static float g_roll_zero = 0.0f;
static float g_yaw_zero = 0.0f;
static float g_pitch_sum = 0.0f;
static float g_roll_sum = 0.0f;
static float g_yaw_sum = 0.0f;
static uint16_t g_zero_count = 0U;
static uint8_t g_zero_ready = 0U;
static uint8_t g_uart1EchoBuf[UART1_ECHO_BUF_SIZE];   //串口1回显缓冲区

static char g_uart1PacketBuf[UART1_PACKET_BUF_SIZE];
static uint16_t g_uart1PacketLen = 0U;
static uint8_t g_uart1PacketActive = 0U;



//==========================================================================
////IMU数据读取
//==========================================================================
static void Task_ImuOledUpdate(void)  
{

    float pitch_raw;
    float roll_raw;
    float yaw_raw;



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

    Attitude6Axis_GetEulerDeg(&g_attitude, &pitch_raw, &roll_raw, &yaw_raw);//获取欧拉角度

    if (g_zero_ready == 0U)
    {
        g_pitch_sum += pitch_raw;
        g_roll_sum += roll_raw;
        g_yaw_sum += yaw_raw;
        g_zero_count++;

        if (g_zero_count >= ATT_ZERO_CALIB_SAMPLES)
        {
            g_pitch_zero = g_pitch_sum / (float)g_zero_count;
            g_roll_zero = g_roll_sum / (float)g_zero_count;
            g_yaw_zero = g_yaw_sum / (float)g_zero_count;
            g_zero_ready = 1U;
        }
    }

    pitchDeg = pitch_raw - g_pitch_zero;//减去零偏得到最终的姿态角度值
    rollDeg = roll_raw - g_roll_zero;//减去零偏得到最终的姿态角度值
    yawDeg = yaw_raw - g_yaw_zero;//减去零偏得到最终的姿态角度值
}
//==========================================================================
//串口1收到的数据原样回发
//==========================================================================
static void Task_Uart1Echo(void)
{
    uint16_t rxLen = DMA_USART1_Read(g_uart1EchoBuf, UART1_ECHO_BUF_SIZE);

    if (rxLen > 0U)
    {
        DMA_USART1_Send(g_uart1EchoBuf, rxLen);

        for (uint16_t i = 0U; i < rxLen; ++i)
        {
            uint8_t ch = g_uart1EchoBuf[i];

            if (ch == (uint8_t)'[')
            {
                g_uart1PacketActive = 1U;
                g_uart1PacketLen = 0U;
                continue;
            }

            if (g_uart1PacketActive == 0U)
            {
                continue;
            }

            if (ch == (uint8_t)']')
            {
                char packetCopy[UART1_PACKET_BUF_SIZE];
                unsigned int sliderId;
                unsigned int sliderValue;

                g_uart1PacketActive = 0U;

                if (g_uart1PacketLen >= UART1_PACKET_BUF_SIZE)
                {
                    g_uart1PacketLen = UART1_PACKET_BUF_SIZE - 1U;
                }

                memcpy(packetCopy, g_uart1PacketBuf, g_uart1PacketLen);
                packetCopy[g_uart1PacketLen] = '\0';

                if (sscanf(packetCopy, "slider,%u,%u", &sliderId, &sliderValue) == 2)//尝试解析滑动条数据包，格式为[slider,ID,VALUE]
                {
                    Serial1_Printf("[uart1] slider=%u value=%u\r\n", sliderId, sliderValue);

                    if (sliderId == 1)
                    {
                        ESC_SetChannelsUs(sliderValue, sliderValue, sliderValue, sliderValue);
                    }
                    
                }
                else
                {
                    Serial1_Printf("[uart1] packet=%s\r\n", packetCopy);
                }

                g_uart1PacketLen = 0U;
                continue;
            }

            if (g_uart1PacketLen < (UART1_PACKET_BUF_SIZE - 1U))
            {
                g_uart1PacketBuf[g_uart1PacketLen] = (char)ch;
                g_uart1PacketLen++;
            }
            else
            {
                g_uart1PacketActive = 0U;
                g_uart1PacketLen = 0U;
            }
        }
    }
    
}

void Task_OledUpdate(void)  //OLED显示更新的任务函数声明
{

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


void Task_ESC_Control(void)  //电调控制任务
{
    static uint8_t  first_run = 1U;
    static uint16_t cunt = 0U;

    if(first_run==1U)
    {
        cunt = 0U;
        first_run=0U;
        
    }
    else
    {
        cunt++;

        ESC_SetChannelsUs(1500+500*0.1f,1500+500*0.1f,1500+500*0.1f,1500+500*0.1f);
        
        if (cunt >= 50U)//20*50ms 
        {
            ESC_SetChannelsUs(1500U,1500U,1500U,1500U);
        }
        



    }
}






//==========================================================================
//主函数
//==========================================================================
int main(void)
{
    board_init();//初始化系统时钟和SysTick

    PWM_Init();//初始化TIM3的PWM输出

    if (ESC_AUTO_CALIBRATION != 0U)
    {
        ESC_CalibrateSequence(5000U, 3000U);//自动完成电调解锁和行程校准
    }
    else
    {
        ESC_Init();
    }




    PA0_LED_Toggle_Init();// 初始化PA0引脚用于LED闪烁

    DMA_USART1_Init(115200);//初始化USART1用于串口调试输出，波特率115200

    
    systick_delay_ms(2000);//等待IMU稳定
    ImuSensor_Init();  //初始化ICM42688
    Attitude6Axis_Init(&g_attitude, 2.0f, 0.02f);
    


    OLED_Init();     //初始化OLED显示屏
    OLED_Clear();

    ESC_SetChannelsUs(1050U,1050U,1050U,1050U);



    SCH_Init();
	//调度器==========================================================================
    SCH_AddTask(Task_ImuOledUpdate      , IMU_TASK_PERIOD_MS		, 7            );
    SCH_AddTask(PA0_LED_Toggle          , 50U                       ,14            );
    //SCH_AddTask(Task_ESC_Control        , 20U                       ,15            );

    //================================================================================
    while (1)
    {
        Task_Uart1Echo(); //处理串口1回显任务
        Task_OledUpdate();//处理OLED显示更新任务
    }


}




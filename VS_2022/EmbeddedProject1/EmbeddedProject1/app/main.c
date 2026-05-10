
//==========================================================================
#include "board.h"
#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>



//==========================================================================

#include "i2c1.h"
#include "FFCY_NEW_ICM42688.h"
#include "Attitude6Axis.h"
#include "oled.h"
#include "pwm_tim3.h"
#include "ppm_input.h"
#include "esc_calibration.h"
#include "tim2_scheduler.h"
#include "pa0_LED_toggle.h"
#include "DMA_UART1.h"





#define ESC_AUTO_CALIBRATION         (0)                                    //首次使用或更换电调时建议打开，按文章流程自动完成解锁/行程校准
#define IMU_TASK_PERIOD_MS           (5)                                    //IMU数据读取和OLED显示更新的周期，单位毫秒    
#define ATT6_DT_SEC                  ((float)IMU_TASK_PERIOD_MS / 1000.0f)  //姿态算法更新周期，单位秒
#define ICM42688_ACC_G_PER_LSB       (1.0f / 8192.0f)                       //ICM42688加速度计每LSB对应的重力加速度值，单位g
#define ICM42688_GYRO_DPS_PER_LSB    (1.0f / 16.4f)                         //ICM42688陀螺仪每LSB对应的角速度值，单位度每秒
#define ATT_ZERO_CALIB_SAMPLES       (1000U)                                //启动后静止采样次数(1000*5ms=5s)
#define UART1_ECHO_BUF_SIZE          (64)                                   //串口1回显缓冲区大小
#define UART1_PACKET_BUF_SIZE        (64)                                   //串口1解析包缓冲区大小

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
static void Task_ImuUpdate(void)  
{
    
    float pitch_raw;
    float roll_raw;
    float yaw_raw;
    
    ImuSensor_ReadReg_BuffAll();//读取ICM42688的原始数据
    ImuSensor_ProcessData(); //处理原始数据得到滤波后的加速度计和陀螺仪数据，并存储在MPU_FilteredData中
    
    Attitude6Axis_UpdateRaw(    //更新姿态算法状态
        &g_attitude,
        -MPU_FilteredData.AccY,
        MPU_FilteredData.AccX,
        MPU_FilteredData.AccZ,
        -MPU_FilteredData.GyroY,
        MPU_FilteredData.GyroX,
        MPU_FilteredData.GyroZ,
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

        if (g_zero_count >= ATT_ZERO_CALIB_SAMPLES)//当累计的样本数量达到预设的校准样本数量时，计算零偏并标记零偏准备就绪
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

//==========================================================================
//OLED显示更新任务
//==========================================================================
void Task_OledUpdate(void)  //OLED显示更新的任务函数声明
{

    OLED_Clear();//清屏====================================================

    OLED_Printf(0, 0,  8, 1, "ATTITUDE 6AX");
    OLED_Printf(0, 8,  8, 1, "P:");             OLED_ShowFloatNum(12, 8 , pitchDeg, 2, 2, 8, 1);
    OLED_Printf(0, 16, 8, 1, "R:");             OLED_ShowFloatNum(12, 16, rollDeg,  2, 2, 8, 1);
    OLED_Printf(0, 24, 8, 1, "Y:");             OLED_ShowFloatNum(12, 24, yawDeg,   2, 2, 8, 1);
    OLED_Printf(0, 32, 8, 1, "AX:%+6d", MPU_FilteredData.AccX);
    OLED_Printf(0, 40, 8, 1, "AY:%+6d", MPU_FilteredData.AccY);
    OLED_Printf(0, 48, 8, 1, "AZ:%+6d", MPU_FilteredData.AccZ);
    OLED_Printf(0, 56, 8, 1, "T :%+6d", MPU_FilteredData.Temp);
    
    
    
    OLED_Refresh();//更新显示=============================================


    {
        int pitchScaled = (int)(pitchDeg * 100.0f);
        int rollScaled = (int)(rollDeg * 100.0f);
        int yawScaled = (int)(yawDeg * 100.0f);

        int pitchInt = pitchScaled / 100;
        int rollInt = rollScaled / 100;
        int yawInt = yawScaled / 100;

        int pitchFrac = pitchScaled % 100;
        int rollFrac = rollScaled % 100;
        int yawFrac = yawScaled % 100;

        if (pitchFrac < 0) { pitchFrac = -pitchFrac; }
        if (rollFrac < 0)  { rollFrac = -rollFrac; }
        if (yawFrac < 0)   { yawFrac = -yawFrac; }

        
    }

}

//==========================================================================
//NVIC中断优先级配置
//==========================================================================
void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;                            //定义NVIC初始化结构体
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);                 //优先级组2


    //飞计数定时器
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;                 //定时器4中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;    //抢占优先级0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;           //子优先级2
    NVIC_Init(&NVIC_InitStructure);

    //PPM接收机
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;              //外部中断0
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;    //抢占优先级0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x04;           //子优先级4
    NVIC_Init(&NVIC_InitStructure);

    //DMA中断优先级
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;         //DMA2 Stream7中断通道（USART1）
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;    //抢占优先级1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;           //子优先级0
    NVIC_Init(&NVIC_InitStructure);


    //飞控任务调度定时器
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;                 //定时器2中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;    //抢占优先级2
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;           //子优先级2
    NVIC_Init(&NVIC_InitStructure);

}




//==========================================================================
//主函数
//==========================================================================
int main(void)
{
    
    NVIC_Configuration();                       //配置NVIC中断优先级
    
    board_init();                               //初始化系统时钟和SysTick

    PWM_Init();                                 //初始化TIM3的PWM输出
	
	PPM_Init();                                 //初始化PPM输入

    if (ESC_AUTO_CALIBRATION != 0U)             //电调自动校准功能
    {
        ESC_CalibrateSequence(1000U, 1000U);    
    }
    else
    {
        ESC_Init();
    }

    PA0_LED_Toggle_Init();                      // 初始化PA0_LED闪烁_最低优先级任务

    //DMA_USART1_Init(115200);                  //初始化USART1用于串口调试输出，波特率115200

	systick_delay_ms(3000);
    
    ImuSensor_Init();                           //初始化ICM42688
    Attitude6Axis_Init(&g_attitude, 2.0f, 0.02f);
    
    OLED_Init();                                //初始化OLED显示屏
    OLED_Clear();

    ESC_SetChannelsUs(1050U,1050U,1050U,1050U);



    SCH_Init();
    //调度器==========================================================================
    SCH_AddTask(Task_ImuUpdate          , IMU_TASK_PERIOD_MS        , 7             );
    SCH_AddTask(PA0_LED_Toggle          , 20U                       ,14             );
    
    
    //================================================================================
    while (1)
    {
        Task_OledUpdate();
    }
}


























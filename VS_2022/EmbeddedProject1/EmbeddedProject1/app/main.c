

//==========================================================================
#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>
#include <misc.h>


//==========================================================================

#include "systick_delay.h"
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
#include "all_control.h"





#define ESC_AUTO_CALIBRATION         (0)                                    //首次使用或更换电调时建议打开，按文章流程自动完成解锁/行程校准

#define IMU_TASK_PERIOD_MS           (5)                                    //IMU数据读取和OLED显示更新的周期ms 
#define ATT6_DT_SEC                  ((float)IMU_TASK_PERIOD_MS / 1000.0f)  //姿态算法更新周期，单位秒
#define ICM42688_ACC_G_PER_LSB       (1.0f / 8192.0f)                       //ICM42688加速度计每LSB对应的重力加速度值，单位g
#define ICM42688_GYRO_DPS_PER_LSB    (1.0f / 16.4f)                         //ICM42688陀螺仪每LSB对应的角速度值，单位度每秒
#define UART1_ECHO_BUF_SIZE          (64)                                   //串口1回显缓冲区大小
#define UART1_PACKET_BUF_SIZE        (64)                                   //串口1解析包缓冲区大小

static Attitude6AxisState g_attitude; //全局姿态算法状态变量

float pitchDeg;
float rollDeg;
float yawDeg;

static uint8_t g_uart1EchoBuf[UART1_ECHO_BUF_SIZE];     //串口1回显缓冲区
static char g_uart1PacketBuf[UART1_PACKET_BUF_SIZE];    //串口1解析包缓冲区
static uint16_t g_uart1PacketLen = 0U;                  //当前解析包的长度
static uint8_t g_uart1PacketActive = 0U;                //当前是否正在解析包的标志

//将滑动条原始值转换为PID增益，支持两种格式：
static float SliderRawToPidGain(uint32_t sliderId, uint32_t raw)
{
    return ((float)raw / 1000.0f) ;
}


//==========================================================================
////IMU数据读取
//==========================================================================
static void Task_ImuUpdate(void)  
{
    
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
    
    Attitude6Axis_GetEulerDeg(&g_attitude, &pitchDeg, &rollDeg, &yawDeg); //获取欧拉角度
    
}


//==============================================================================
// brief 从串口接收缓冲区读取数据，回显并解析 [slider,id,gain] 命令
//==============================================================================
void Task_Uart1Echo_Process(void)
{
    uint8_t recvBuf[512]; // 临时缓冲区，大小可根据实际调整
    uint16_t len;

    // 循环读取，直到缓冲区中没有新数据为止（防止多条命令堆积）
    while ((len = DMA_USART1_Read(recvBuf, sizeof(recvBuf) - 1)) > 0)
    {
        recvBuf[len] = '\0'; // 确保字符串结束

        // ---------- 回发（echo）----------
        DMA_USART1_SendString("Echo: ");
        DMA_USART1_Send(recvBuf, len);
        DMA_USART1_SendString("\r\n");

        // ---------- 解析命令 ----------
        // 查找 '[' 和 ']'
        char *pStart = strchr((char *)recvBuf, '[');
        char *pEnd   = strchr((char *)recvBuf, ']');

        if (pStart && pEnd && pEnd > pStart)
        {
            // 提取方括号内的内容
            size_t innerLen = pEnd - pStart - 1;
            if (innerLen > 0 && innerLen < sizeof(recvBuf))
            {
                char inner[64];
                memcpy(inner, pStart + 1, innerLen);
                inner[innerLen] = '\0';

                // 用逗号分割三个字段
                char *token = strtok(inner, ",");
                if (token && strcmp(token, "slider") == 0)
                {
                    token = strtok(NULL, ",");
                    if (token)
                    {
                        int sliderId = atoi(token);
                        token = strtok(NULL, ",");
                        if (token)
                        {
                            int gain = atoi(token);
                            // 调用 PID 调节接口
                            ALL_Control_TunePidBySlider((uint32_t)sliderId, (float)(gain/1000.0f));
                        }
                    }
                }
            }
        }
    }
    //DMA_USART1_SendString("123,ABC\r\n");
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
    OLED_Printf(0, 32, 8, 1, "AX:%+6d GX:%+6d", MPU_FilteredData.AccX, MPU_FilteredData.GyroX);
    OLED_Printf(0, 40, 8, 1, "AY:%+6d GY:%+6d", MPU_FilteredData.AccY, MPU_FilteredData.GyroY);
    OLED_Printf(0, 48, 8, 1, "AZ:%+6d GZ:%+6d", MPU_FilteredData.AccZ, MPU_FilteredData.GyroZ);
    OLED_Printf(0, 56, 8, 1, "T :%+6d", MPU_FilteredData.Temp);
    
    
    
    OLED_Refresh();//更新显示=============================================
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
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;    //抢占优先级0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;           //子优先级2
    NVIC_Init(&NVIC_InitStructure);

    //PPM接收机
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;              //外部中断0
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;    //抢占优先级0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;           //子优先级4
    NVIC_Init(&NVIC_InitStructure);

    //DMA中断优先级
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;         //DMA2 Stream7中断通道（USART1）
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;    //抢占优先级1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;           //子优先级0
    NVIC_Init(&NVIC_InitStructure);


    //飞控任务调度定时器
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;                 //定时器2中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;    //抢占优先级2
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;           //子优先级2
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

    
    
    
    
    PA0_LED_Toggle_Init();                          // 初始化PA0_LED闪烁_最低优先级任务

    DMA_USART1_Init(115200);                        //初始化USART1用于串口调试输出，波特率115200

    systick_delay_ms(3000);                         //启动后延时3秒
    ImuSensor_Init();                               //初始化ICM42688
    Attitude6Axis_Init(&g_attitude, 2.0f, 0.02f);   //初始化姿态算法状态，参数为陀螺仪滤波系数和加速度计权重
    
    
    ALL_Control_Init();                             //初始化飞控算法状态
    
    OLED_Init();                                    //初始化OLED显示屏
    OLED_Clear();

    ESC_SetChannelsUs(1000U,1000U,1000U,1000U);

    
    
    
    SCH_Init();
    //调度器==========================================================================
    SCH_AddTask(Task_ImuUpdate          , IMU_TASK_PERIOD_MS        , 6             );
    SCH_AddTask(ALL_Control_Task        , IMU_TASK_PERIOD_MS        , 7             );
    SCH_AddTask(PA0_LED_Toggle          , 20U                       ,14             );
    
    
    //================================================================================
    while (1)
    {
        Task_OledUpdate();
        Task_Uart1Echo_Process();
        systick_delay_ms(1);
    }
}


























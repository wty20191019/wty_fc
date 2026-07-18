#ifndef __FFCY_NEW_ICM42688_H__
#define __FFCY_NEW_ICM42688_H__

#include "stm32f4xx.h"
//==========================================================================
//ICM42688寄存器地址和配置
//==========================================================================
#define ICM42688_ADDRESS_DEFAULT        0x68
#define ICM42688_ADDRESS_ALT            0x69
#define ICM42688_WHO_AM_I               0x75
#define ICM42688_WHOAMI_VALUE           0x47
#define ICM42688_REG_BANK_SEL           0x76
#define ICM42688_PWR_MGMT0              0x4E
#define ICM42688_ACCEL_CONFIG0          0x50
#define ICM42688_GYRO_CONFIG0           0x4F
#define ICM42688_GYRO_ACCEL_CONFIG0     0x52
#define ICM42688_INT_CONFIG             0x14
#define ICM42688_TEMP_DATA1             0x1D

#define ICM42688_GYRO_ODR               0x06
#define ICM42688_GYRO_RNG               0x00
#define ICM42688_ACC_ODR                0x06
#define ICM42688_ACC_RNG                0x02


#define ICM42688_ACCEL_XOUT    0x1F//加速度计X轴数据寄存器地址
#define ICM42688_ACCEL_YOUT    0x21//加速度计Y轴数据寄存器地址
#define ICM42688_ACCEL_ZOUT    0x23//加速度计Z轴数据寄存器地址
#define ICM42688_GYRO_XOUT     0x25//陀螺仪X轴数据寄存器地址
#define ICM42688_GYRO_YOUT     0x27//陀螺仪Y轴数据寄存器地址
#define ICM42688_GYRO_ZOUT     0x29//陀螺仪Z轴数据寄存器地址

//=====================================================================
//硬件低通滤波器配置
//=====================================================================
//可选的滤波档位，单位Hz，实际截止频率可能略有偏差，具体数值请参考ICM42688数据手册

#define ICM42688_FILTER_BW_180HZ     0x00U  
#define ICM42688_FILTER_BW_121HZ     0x01U
#define ICM42688_FILTER_BW_73HZ      0x02U
#define ICM42688_FILTER_BW_53HZ      0x03U
#define ICM42688_FILTER_BW_34HZ      0x04U
#define ICM42688_FILTER_BW_25HZ      0x05U
#define ICM42688_FILTER_BW_16HZ      0x06U
#define ICM42688_FILTER_BW_8HZ       0x07U

#define ICM42688_ACCEL_UI_FILT_BW    ICM42688_FILTER_BW_8HZ       //加速度计的数字低通滤波档位
#define ICM42688_GYRO_UI_FILT_BW     ICM42688_FILTER_BW_8HZ        //陀螺仪的数字低通滤波档位       
#define ICM42688_HW_FILTER_CFG       ((uint8_t)(((ICM42688_ACCEL_UI_FILT_BW & 0x0FU) << 4) | (ICM42688_GYRO_UI_FILT_BW & 0x0FU)))

//==========================================================================
//软件低通滤波器
//==========================================================================
#define ICM42688_SOFT_FILTER_ENABLE             1           //是否启用软件低通滤波器

#define ICM42688_SOFT_FILTER_SAMPLE_HZ          200.0f      //软件低通滤波器的采样频率，单位Hz，应与实际读取IMU数据的频率相匹配
#define ICM42688_acc_SOFT_FILTER_CUTOFF_HZ      20.0f        //acc 软件低通滤波器的截止频率，单位Hz
#define ICM42688_gyro_SOFT_FILTER_CUTOFF_HZ     20.0f        //gyro软件低通滤波器的截止频率，单位Hz
#define ICM42688_SOFT_FILTER_WARMUP_COUNT       400U        //软件低通滤波器的预热采样次数，滤波器在预热期间不输出有效数据//建议设置为采样频率的1-2秒的采样数量，例如100Hz采样频率可以设置为100-200


//==========================================================================
//卡尔曼滤波器
//==========================================================================
#define ICM42688_KALMAN_ENABLE              0       //是否启用卡尔曼滤波器



//==========================================================================
//陀螺仪偏置校准//gyro
//==========================================================================
#define ICM42688_GYRO_BIAS_ENABLE          1        //是否启用陀螺仪偏置校准和跟踪功能

#define ICM42688_GYRO_BIAS_CAL_SAMPLES     1000U    //陀螺仪偏置校准时的采样数量
#define ICM42688_GYRO_BIAS_MAX_ABS_LSB     2500     //陀螺仪偏置校准时的最大绝对值阈值(LSB)


//==========================================================================
//加速度计偏置校准//ACC
//==========================================================================
#define ICM42688_ACC_BIAS_ENABLE           1        //是否启用加速度计偏置校准（静止、水平放置时效果最佳）

#define ICM42688_ACC_BIAS_CAL_SAMPLES      1000U    //加速度计偏置校准采样数量
#define ICM42688_ACC_BIAS_MAX_ABS_XY_LSB   2500     //校准期间   X/Y        最大绝对值阈值（LSB），用于判断是否接近水平静止
#define ICM42688_ACC_BIAS_MAX_1G_DEV_LSB   2500     //校准期间  | |Az|-1g | 最大偏  差阈值（LSB）

/* 期望的静止重力向量（单位：LSB）。
 * 默认假设水平放置，Z轴读数约为 +1g。
 * 如你的板子方向相反（静止时Az约为-1g）
 * 可将Z期望改为(-ICM42688_ACC_1G_LSB)。 */

#define ICM42688_ACC_BIAS_EXPECT_X_LSB     0
#define ICM42688_ACC_BIAS_EXPECT_Y_LSB     0
#define ICM42688_ACC_BIAS_EXPECT_Z_LSB     (ICM42688_ACC_1G_LSB)


//==========================================================================
//跟踪功能
//==========================================================================
#define ICM42688_GYRO_BIAS_TRACK_ENABLE    1       //是否启用陀螺仪偏置跟踪功能，启用后在正常运行过程中会持续跟踪陀螺仪偏置的变化并进行补偿，以进一步抑制陀螺仪漂移，特别是对于航向（偏航）的漂移抑制非常有效

#define ICM42688_GYRO_BIAS_TRACK_ALPHA     0.001f   //陀螺仪偏置跟踪的指数移动平均滤波系数，范围0.0-1.0，值越小跟踪越慢但更稳定，值越大跟踪越快但可能引入噪声，建议设置为0.001-0.01
#define ICM42688_GYRO_STILL_THRESH_LSB     20       //陀螺仪静止状态检测的阈值，单位LSB，当陀螺仪输出的绝对值都低于此阈值时认为IMU处于静止状态，建议设置为20-50LSB（约1-3dps）以适应不同的IMU安装环境
#define ICM42688_ACC_1G_LSB                8192     //ICM42688加速度计1g对应的LSB值，基于±4g量程设置
#define ICM42688_ACC_NORM_MIN_RATIO        0.80f    //加速度计归一化最小比率
#define ICM42688_ACC_NORM_MAX_RATIO        1.20f    //加速度计归一化最大比率




typedef enum
{
    ICM42688_OK = 0,
    ICM42688_TIMEOUT = 1,
    ICM42688_ERROR = 2,
} ICM42688_Status;

typedef struct
{
    int16_t AccX;
    int16_t AccY;
    int16_t AccZ;
    int16_t GyroX;
    int16_t GyroY;
    int16_t GyroZ;
    int16_t Temp;
} ICM42688_RawData;

typedef ICM42688_RawData MPU6050_Data;

extern ICM42688_RawData MPU_Data;
extern ICM42688_RawData MPU_FilteredData;
extern MPU6050_Data FT_MPU_FilteredData;
extern MPU6050_Data CH_FT_MPU_FilteredData;
extern u32 IIC_Timeout_Cnt;
extern u32 IIC_Timeout_Cnt_noTimesClear;

void ImuSensor_Init(void);
ICM42688_Status ICM42688_Init(void);
void ImuSensor_ReadReg_BuffAll(void);
void ImuSensor_ProcessData(void);
ICM42688_Status ICM42688_ReadWhoAmI(uint8_t *who_am_i);
void ICM42688_WriteReg(uint8_t DevAddress, uint8_t RegAddress, uint8_t Data);
uint8_t ICM42688_ReadReg(uint8_t DevAddress, uint8_t RegAddress);
int16_t GetData_Gyro(uint8_t REG_Address);
int16_t GetData_Acc(uint8_t REG_Address);
void GET_MPU_DATA(void);

#endif

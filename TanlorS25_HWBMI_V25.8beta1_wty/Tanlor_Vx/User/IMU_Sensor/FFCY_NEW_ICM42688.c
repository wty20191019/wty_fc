#include "Headfile.h"
#include "FFCY_NEW_ICM42688.h"
#include "stm32f4xx_spi.h"

/* forward declare SPI helpers implemented at end of this file */
static uint8_t SPI_TransmitReceiveByte(uint8_t data);
static uint8_t SPI_ReadReg(uint8_t reg);
static void SPI_ReadBuffer(uint8_t reg, uint8_t* data, uint16_t len);
static void SPI_WriteReg(uint8_t reg, uint8_t data);

#if IMU_SENSOR == ICM42688

MPU6050_Data MPU_Data;
Vector3f Accel, Gyro;
float X_Origion, Y_Origion, Z_Origion;                 // 经过椭圆校正后的三轴加速度量
int16_t Acce_Correct[3] = { 0 };                       // 用于矫正加速度量，截至频率很低
Vector3f accel_filter, gyro_filter_QU;
Vector3f gyro_nofilter;

float IMU_K[3] = { 1.0f, 1.0f, 1.0f };                 // 默认标度(量程)误差
float IMU_B[3] = { 0.0f, 0.0f, 0.0f };                 // 默认零位误差

Axis3f Gyro_feedback;

Butter_Parameter Accel_Parameter, Butter_1HZ_Parameter_Acce, Gyro_Parameter, Gyro_Parameter_QU, Ins_Accel_Parameter;
Butter_BufferData Butter_Buffer_SINS[3], Butter_Buffer_Correct[3], gyro_filter_buf[3], gyro_filter_QU_buf[3], accel_filter_buf[3];

u32 IIC_Timeout_Cnt = 0;
u32 IIC_Timeout_Cnt_noTimesClear = 0;

/* SPI register access (ICM42688 uses MSB=1 for read, MSB=0 for write) */
void ICM42688_WriteReg(uint8_t DevAddress, uint8_t RegAddress, uint8_t Data)
{
    (void)DevAddress;
    SPI_WriteReg(RegAddress, Data);
}

uint8_t ICM42688_ReadReg(uint8_t DevAddress, uint8_t RegAddress)
{
    (void)DevAddress;
    return SPI_ReadReg(RegAddress);
}

int16_t GetData_Gyro(uint8_t REG_Address)
{
    uint8_t hd = SPI_ReadReg(REG_Address);
    uint8_t ld = SPI_ReadReg((uint8_t)(REG_Address + 1));
    return (int16_t)(((uint16_t)hd << 8) | ld);
}

int16_t GetData_Acc(uint8_t REG_Address)
{
    uint8_t hd = SPI_ReadReg(REG_Address);
    uint8_t ld = SPI_ReadReg((uint8_t)(REG_Address + 1));
    return (int16_t)(((uint16_t)hd << 8) | ld);
}

void ImuSensor_ReadReg_BuffAll(void)
{
    uint8_t buf[14];
    IIC_Timeout_Cnt = 0;

    /* Burst read: TEMP(0x1D..0x1E) + ACC(0x1F..0x24) + GYRO(0x25..0x2A) */
    SPI_ReadBuffer(ICM42688_TEMP_DATA1, buf, (uint16_t)sizeof(buf));

    MPU_Data.Temp = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
    MPU_Data.AccX = (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);
    MPU_Data.AccY = (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);
    MPU_Data.AccZ = (int16_t)(((uint16_t)buf[6] << 8) | buf[7]);
    MPU_Data.GyroX = (int16_t)(((uint16_t)buf[8] << 8) | buf[9]);
    MPU_Data.GyroY = (int16_t)(((uint16_t)buf[10] << 8) | buf[11]);
    MPU_Data.GyroZ = (int16_t)(((uint16_t)buf[12] << 8) | buf[13]);
}

static uint8_t ICM42688_CheckWhoAmI(void)
{
    return (SPI_ReadReg(ICM42688_WHO_AM_I) == ICM42688_WHOAMI_VALUE) ? 1U : 0U;
}

static void ICM42688_Config(void)
{
    /* Bank 0 */
    SPI_WriteReg(ICM42688_REG_BANK_SEL, 0x00);

    /* Enable accel + gyro in low-noise mode */
    SPI_WriteReg(ICM42688_PWR_MGMT0, 0x0F);

    /* Configure accel + gyro ODR / FS */
    SPI_WriteReg(ICM42688_ACCEL_CONFIG0, (uint8_t)((ICM42688_ACC_RNG << 5) | ICM42688_ACC_ODR));
    SPI_WriteReg(ICM42688_GYRO_CONFIG0, (uint8_t)((ICM42688_GYRO_RNG << 5) | ICM42688_GYRO_ODR));

    /* Configure UI filters */
    SPI_WriteReg(ICM42688_GYRO_ACCEL_CONFIG0, ICM42688_HW_FILTER_CFG);

    /* INT pin config (same as existing wty I2C preset) */
    SPI_WriteReg(ICM42688_INT_CONFIG, 0x1B);
}

void ImuSensor_Init(void)
{
    /* Inline SPI PB12-15 init (previously in SPI_PB12_15_Init) */
    {
        GPIO_InitTypeDef GPIO_InitStructure;
        SPI_InitTypeDef  SPI_InitStructure;

        /* Enable clocks */
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

        /* Configure PB13/PB14/PB15 as AF for SPI2 */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_Init(GPIOB, &GPIO_InitStructure);

        GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);

        /* Configure PB12 as GPIO output for CS */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_Init(GPIOB, &GPIO_InitStructure);
        GPIO_SetBits(GPIOB, GPIO_Pin_12); /* CS high (inactive) */

        /* SPI2 configuration */
        SPI_I2S_DeInit(SPI2);
        SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
        SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
        SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
        SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
        SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
        SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
        SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; /* adjust as needed */
        SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
        SPI_InitStructure.SPI_CRCPolynomial = 7;
        SPI_Init(SPI2, &SPI_InitStructure);
        SPI_Cmd(SPI2, ENABLE);
    }

    /* Probe device */
    while (ICM42688_CheckWhoAmI() == 0U)
    {
        IIC_Timeout_Cnt++;
        IIC_Timeout_Cnt_noTimesClear++;
    }

    ICM42688_Config();

    for (u8 i = 0; i < 3; i++) // 初始化加速计和陀螺二阶低通滤波
    {
        lpf2pInit(&gyroLpf[i], 200, GYRO_LPF_CUTOFF_FREQ);
        lpf2pInit(&accLpf[i], 200, ACCEL_LPF_CUTOFF_FREQ);
    }

    Set_Cutoff_Frequency(Sampling_Freq, 10, &Ins_Accel_Parameter);        // 惯性导航加速度
    Set_Cutoff_Frequency(Sampling_Freq, 1, &Butter_1HZ_Parameter_Acce);   // 传感器校准加计滤波值
    Set_Cutoff_Frequency(Sampling_Freq, 10, &Accel_Parameter);            // 姿态解算加计修正滤波值
    Set_Cutoff_Frequency(Sampling_Freq, 10, &Gyro_Parameter_QU);          // 四元数使用的角速度
    Set_Cutoff_Frequency(Sampling_Freq, 8, &Gyro_Parameter);              // 角速度反馈滤波参数
}

void GET_MPU_DATA(void)
{
    ImuSensor_ReadReg_BuffAll();

    int16_t ay = MPU_Data.AccX;
    int16_t ax = MPU_Data.AccY;
    int16_t az = MPU_Data.AccZ;

    int16_t gy = MPU_Data.GyroX;
    int16_t gx = MPU_Data.GyroY;
    int16_t gz = MPU_Data.GyroZ;

    sensors.gyro.x = -(gx - X_w_off) * SENSORS_DEG_PER_LSB_CFG; /* 单位 °/s */
    sensors.gyro.y = (gy - Y_w_off) * SENSORS_DEG_PER_LSB_CFG;
    sensors.gyro.z = (gz - Z_w_off) * SENSORS_DEG_PER_LSB_CFG;

    sensors.acc.x = -(ax) * SENSORS_G_PER_LSB_CFG; /* 单位 g */
    sensors.acc.y = (ay) * SENSORS_G_PER_LSB_CFG;
    sensors.acc.z = (az) * SENSORS_G_PER_LSB_CFG;

    applyAxis3fLpf(gyroLpf, &sensors.gyro);
    applyAxis3fLpf(accLpf, &sensors.acc);

    sensors1.gyro.x = sensors.gyro.x;
    sensors1.gyro.y = sensors.gyro.y;
    sensors1.gyro.z = sensors.gyro.z;

    sensors1.acc.x = sensors.acc.x;
    sensors1.acc.y = sensors.acc.y;
    sensors1.acc.z = sensors.acc.z;

    MPitch_Gyro = sensors.gyro.x;
    MRoll_Gyro = sensors.gyro.y;
    MYaw_Gyro = sensors.gyro.z;

    Accel.y = MPU_Data.AccX;
    Accel.x = -MPU_Data.AccY;
    Accel.z = MPU_Data.AccZ;
    Gyro.y = MPU_Data.GyroX - X_w_off;
    Gyro.x = -(MPU_Data.GyroY - Y_w_off);
    Gyro.z = MPU_Data.GyroZ - Z_w_off;

    /* Acce_Correct_Filter */
    Acce_Correct[0] = (int16_t)(LPButterworth(Accel.x, &Butter_Buffer_Correct[0], &Butter_1HZ_Parameter_Acce));
    Acce_Correct[1] = (int16_t)(LPButterworth(Accel.y, &Butter_Buffer_Correct[1], &Butter_1HZ_Parameter_Acce));
    Acce_Correct[2] = (int16_t)(LPButterworth(Accel.z, &Butter_Buffer_Correct[2], &Butter_1HZ_Parameter_Acce));

    /* 椭球矫正 */
    X_Origion = IMU_K[0] * Accel.x - IMU_B[0] * One_G_TO_Accel;
    Y_Origion = IMU_K[1] * Accel.y - IMU_B[1] * One_G_TO_Accel;
    Z_Origion = IMU_K[2] * Accel.z - IMU_B[2] * One_G_TO_Accel;

    accel_filter.x = LPButterworth(X_Origion, &accel_filter_buf[0], &Accel_Parameter);
    accel_filter.y = LPButterworth(Y_Origion, &accel_filter_buf[1], &Accel_Parameter);
    accel_filter.z = LPButterworth(Z_Origion, &accel_filter_buf[2], &Accel_Parameter);

    Body_Frame.x = LPButterworth(X_Origion, &Butter_Buffer_SINS[0], &Ins_Accel_Parameter);
    Body_Frame.y = LPButterworth(Y_Origion, &Butter_Buffer_SINS[1], &Ins_Accel_Parameter);
    Body_Frame.z = LPButterworth(Z_Origion, &Butter_Buffer_SINS[2], &Ins_Accel_Parameter);

    MBody_Frame.x = sensors.acc.x / SENSORS_G_PER_LSB_CFG;
    MBody_Frame.y = sensors.acc.y / SENSORS_G_PER_LSB_CFG;
    MBody_Frame.z = sensors.acc.z / SENSORS_G_PER_LSB_CFG;

    gyro_nofilter.x = Gyro.x * GYRO_CALIBRATION_COFF;
    gyro_nofilter.y = Gyro.y * GYRO_CALIBRATION_COFF;
    gyro_nofilter.z = Gyro.z * GYRO_CALIBRATION_COFF;
}

/* SPI2片选引脚定义 */
#define ICM42688_CS_GPIO      GPIOB
#define ICM42688_CS_PIN       GPIO_Pin_12

/* 片选拉低 */
static void ICM42688_CS_Low(void)
{
    GPIO_ResetBits(ICM42688_CS_GPIO, ICM42688_CS_PIN);
}

/* 片选拉高 */
static void ICM42688_CS_High(void)
{
    GPIO_SetBits(ICM42688_CS_GPIO, ICM42688_CS_PIN);
}

/* SPI写寄存器函数实现 */
static void SPI_WriteReg(uint8_t reg, uint8_t data)
{
    ICM42688_CS_Low();
    /* write opcode: MSB=0 */
    SPI_TransmitReceiveByte(reg & 0x7F);
    SPI_TransmitReceiveByte(data);
    ICM42688_CS_High();
}

/* SPI读取单寄存器 */
static uint8_t SPI_ReadReg(uint8_t reg)
{
    uint8_t val;
    ICM42688_CS_Low();
    /* read opcode: set MSB */
    SPI_TransmitReceiveByte(reg | 0x80);
    val = SPI_TransmitReceiveByte(0xFF);
    ICM42688_CS_High();
    return val;
}

/* SPI读取连续缓冲区 */
static void SPI_ReadBuffer(uint8_t reg, uint8_t* data, uint16_t len)
{
    ICM42688_CS_Low();
    SPI_TransmitReceiveByte(reg | 0x80);
    for (uint16_t i = 0; i < len; i++)
    {
        data[i] = SPI_TransmitReceiveByte(0xFF);
    }
    ICM42688_CS_High();
}

/* 基本的 SPI 发送/接收字节 */
static uint8_t SPI_TransmitReceiveByte(uint8_t data)
{
    /* Wait until TXE set */
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) {}
    SPI_I2S_SendData(SPI2, data);
    /* Wait for RXNE */
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET) {}
    return (uint8_t)SPI_I2S_ReceiveData(SPI2);
}

#endif

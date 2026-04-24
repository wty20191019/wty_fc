#include "stm32f4xx_i2c.h"
#include "FFCY_NEW_ICM42688.h"
#include "FFCY_NEW_ICM42688_compat.h"

#define ICM42688_ENABLE_FULL_PIPELINE 1

#ifndef IMU_SENSOR
#define IMU_SENSOR ICM42688
#endif

#ifndef ICM42688_ENABLE_FULL_PIPELINE
#define ICM42688_ENABLE_FULL_PIPELINE 0
#endif

#if IMU_SENSOR == ICM42688

MPU6050_Data MPU_Data;

#if ICM42688_ENABLE_FULL_PIPELINE
Vector3f Accel, Gyro;
float X_Origion, Y_Origion, Z_Origion;
int16_t Acce_Correct[3] = {0};
Vector3f accel_filter, gyro_filter_QU;
Vector3f gyro_nofilter;

float IMU_K[3] = {1.0f, 1.0f, 1.0f};
float IMU_B[3] = {0.0f, 0.0f, 0.0f};

Axis3f Gyro_feedback;

Butter_Parameter Accel_Parameter, Butter_1HZ_Parameter_Acce, Gyro_Parameter, Gyro_Parameter_QU, Ins_Accel_Parameter;
Butter_BufferData Butter_Buffer_SINS[3], Butter_Buffer_Correct[3], gyro_filter_buf[3], gyro_filter_QU_buf[3], accel_filter_buf[3];
#endif

u32 IIC_Timeout_Cnt = 0;
u32 IIC_Timeout_Cnt_noTimesClear = 0;

static uint8_t s_device_address = ICM42688_ADDRESS_DEFAULT;

static uint8_t ICM42688_WaitEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT)
{
    uint32_t Timeout = 2000U;

    while (I2C_CheckEvent(I2Cx, I2C_EVENT) != SUCCESS)
    {
        Timeout--;
        if (Timeout == 0U)
        {
            IIC_Timeout_Cnt++;
            IIC_Timeout_Cnt_noTimesClear++;
            return 0U;
        }
    }

    return 1U;
}

static uint8_t icm42688_normalize_address(uint8_t dev_address)
{
    if (dev_address > 0x7FU)
    {
        return (uint8_t)(dev_address >> 1);
    }

    return dev_address;
}

static void ICM42688_I2C_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef I2C_InitStructure;

    I2C_DeInit(I2C1);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1);

    I2C_StructInit(&I2C_InitStructure);
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_ClockSpeed = 400000U;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_Init(I2C1, &I2C_InitStructure);
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    I2C_Cmd(I2C1, ENABLE);
}

static ICM42688_Status icm42688_probe_address(uint8_t addr)
{
    uint8_t who_am_i = 0;

    s_device_address = addr;
    if (ICM42688_ReadWhoAmI(&who_am_i) != ICM42688_OK)
    {
        return ICM42688_TIMEOUT;
    }

    return (who_am_i == ICM42688_WHOAMI_VALUE) ? ICM42688_OK : ICM42688_ERROR;
}

static ICM42688_Status icm42688_write_reg_current(uint8_t reg_addr, uint8_t data)
{
    I2C_GenerateSTART(I2C1, ENABLE);
    if (ICM42688_WaitEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) == 0U)
    {
        I2C_GenerateSTOP(I2C1, ENABLE);
        return ICM42688_TIMEOUT;
    }

    I2C_Send7bitAddress(I2C1, s_device_address, I2C_Direction_Transmitter);
    if (ICM42688_WaitEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == 0U)
    {
        I2C_GenerateSTOP(I2C1, ENABLE);
        return ICM42688_TIMEOUT;
    }

    I2C_SendData(I2C1, reg_addr);
    if (ICM42688_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING) == 0U)
    {
        I2C_GenerateSTOP(I2C1, ENABLE);
        return ICM42688_TIMEOUT;
    }

    I2C_SendData(I2C1, data);
    if (ICM42688_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == 0U)
    {
        I2C_GenerateSTOP(I2C1, ENABLE);
        return ICM42688_TIMEOUT;
    }

    I2C_GenerateSTOP(I2C1, ENABLE);
    return ICM42688_OK;
}

static ICM42688_Status icm42688_read_reg_current(uint8_t reg_addr, uint8_t *data)
{
    if (data == 0)
    {
        return ICM42688_ERROR;
    }

    I2C_GenerateSTART(I2C1, ENABLE);
    if (ICM42688_WaitEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) == 0U)
    {
        I2C_GenerateSTOP(I2C1, ENABLE);
        return ICM42688_TIMEOUT;
    }

    I2C_Send7bitAddress(I2C1, s_device_address, I2C_Direction_Transmitter);
    if (ICM42688_WaitEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == 0U)
    {
        I2C_GenerateSTOP(I2C1, ENABLE);
        return ICM42688_TIMEOUT;
    }

    I2C_SendData(I2C1, reg_addr);
    if (ICM42688_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == 0U)
    {
        I2C_GenerateSTOP(I2C1, ENABLE);
        return ICM42688_TIMEOUT;
    }

    I2C_GenerateSTART(I2C1, ENABLE);
    if (ICM42688_WaitEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) == 0U)
    {
        I2C_GenerateSTOP(I2C1, ENABLE);
        return ICM42688_TIMEOUT;
    }

    I2C_Send7bitAddress(I2C1, s_device_address, I2C_Direction_Receiver);
    if (ICM42688_WaitEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) == 0U)
    {
        I2C_GenerateSTOP(I2C1, ENABLE);
        return ICM42688_TIMEOUT;
    }

    I2C_AcknowledgeConfig(I2C1, DISABLE);
    I2C_GenerateSTOP(I2C1, ENABLE);
    if (ICM42688_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) == 0U)
    {
        I2C_AcknowledgeConfig(I2C1, ENABLE);
        return ICM42688_TIMEOUT;
    }

    *data = I2C_ReceiveData(I2C1);
    I2C_AcknowledgeConfig(I2C1, ENABLE);

    return ICM42688_OK;
}

ICM42688_Status ICM42688_ReadWhoAmI(uint8_t *who_am_i)
{
    if (who_am_i == 0)
    {
        return ICM42688_ERROR;
    }

    return icm42688_read_reg_current(ICM42688_WHO_AM_I, who_am_i);
}

void ICM42688_WriteReg(uint8_t DevAddress, uint8_t RegAddress, uint8_t Data)
{
    s_device_address = icm42688_normalize_address(DevAddress);
    (void)icm42688_write_reg_current(RegAddress, Data);
}

uint8_t ICM42688_ReadReg(uint8_t DevAddress, uint8_t RegAddress)
{
    uint8_t data = 0;
    s_device_address = icm42688_normalize_address(DevAddress);
    (void)icm42688_read_reg_current(RegAddress, &data);

    return data;
}

int16_t GetData_Gyro(uint8_t REG_Address)
{
    uint8_t hd = ICM42688_ReadReg(s_device_address, REG_Address);
    uint8_t ld = ICM42688_ReadReg(s_device_address, (uint8_t)(REG_Address + 1U));
    return (int16_t)(((uint16_t)hd << 8) | ld);
}

int16_t GetData_Acc(uint8_t REG_Address)
{
    uint8_t hd = ICM42688_ReadReg(s_device_address, REG_Address);
    uint8_t ld = ICM42688_ReadReg(s_device_address, (uint8_t)(REG_Address + 1U));
    return (int16_t)(((uint16_t)hd << 8) | ld);
}

void ImuSensor_ReadReg_BuffAll(void)
{
    int16_t ImuSensor_buff[6];

    IIC_Timeout_Cnt = 0;

    ImuSensor_buff[0] = GetData_Gyro(ICM42688_GYRO_XOUT);
    ImuSensor_buff[1] = GetData_Gyro(ICM42688_GYRO_YOUT);
    ImuSensor_buff[2] = GetData_Gyro(ICM42688_GYRO_ZOUT);
    ImuSensor_buff[3] = GetData_Acc(ICM42688_ACCEL_XOUT);
    ImuSensor_buff[4] = GetData_Acc(ICM42688_ACCEL_YOUT);
    ImuSensor_buff[5] = GetData_Acc(ICM42688_ACCEL_ZOUT);

    if (IIC_Timeout_Cnt == 0)
    {
        MPU_Data.GyroX = ImuSensor_buff[0];
        MPU_Data.GyroY = ImuSensor_buff[1];
        MPU_Data.GyroZ = ImuSensor_buff[2];
        MPU_Data.AccX = ImuSensor_buff[3];
        MPU_Data.AccY = ImuSensor_buff[4];
        MPU_Data.AccZ = ImuSensor_buff[5];
        MPU_Data.Temp = (int16_t)(((uint16_t)ICM42688_ReadReg(s_device_address, ICM42688_TEMP_DATA1) << 8) |
                                  ICM42688_ReadReg(s_device_address, (uint8_t)(ICM42688_TEMP_DATA1 + 1U)));
    }
}

ICM42688_Status ICM42688_Init(void)
{
    ICM42688_I2C_Init();
    IIC_Timeout_Cnt = 0;
    IIC_Timeout_Cnt_noTimesClear = 0;

    if (icm42688_probe_address(ICM42688_ADDRESS_DEFAULT) != ICM42688_OK)
    {
        if (icm42688_probe_address(ICM42688_ADDRESS_ALT) != ICM42688_OK)
        {
            return ICM42688_TIMEOUT;
        }
    }

    if (icm42688_write_reg_current(ICM42688_REG_BANK_SEL, 0x00) != ICM42688_OK)
    {
        return ICM42688_TIMEOUT;
    }

    if (icm42688_write_reg_current(ICM42688_PWR_MGMT0, 0x0F) != ICM42688_OK)
    {
        return ICM42688_TIMEOUT;
    }

    if (icm42688_write_reg_current(ICM42688_ACCEL_CONFIG0, (uint8_t)((ICM42688_ACC_RNG << 5) | ICM42688_ACC_ODR)) != ICM42688_OK)
    {
        return ICM42688_TIMEOUT;
    }

    if (icm42688_write_reg_current(ICM42688_GYRO_CONFIG0, (uint8_t)((ICM42688_GYRO_RNG << 5) | ICM42688_GYRO_ODR)) != ICM42688_OK)
    {
        return ICM42688_TIMEOUT;
    }

    if (icm42688_write_reg_current(ICM42688_GYRO_ACCEL_CONFIG0, 0x44) != ICM42688_OK)
    {
        return ICM42688_TIMEOUT;
    }

    if (icm42688_write_reg_current(ICM42688_INT_CONFIG, 0x1B) != ICM42688_OK)
    {
        return ICM42688_TIMEOUT;
    }

    return ICM42688_OK;
}

void ImuSensor_Init(void)
{
    while (ICM42688_Init() != ICM42688_OK)
    {
    }

#if ICM42688_ENABLE_FULL_PIPELINE
    for (u8 i = 0; i < 3; i++)
    {
        lpf2pInit(&gyroLpf[i], 200, GYRO_LPF_CUTOFF_FREQ);
        lpf2pInit(&accLpf[i], 200, ACCEL_LPF_CUTOFF_FREQ);
    }

    Set_Cutoff_Frequency(Sampling_Freq, 10, &Ins_Accel_Parameter);
    Set_Cutoff_Frequency(Sampling_Freq, 1, &Butter_1HZ_Parameter_Acce);
    Set_Cutoff_Frequency(Sampling_Freq, 10, &Accel_Parameter);
    Set_Cutoff_Frequency(Sampling_Freq, 10, &Gyro_Parameter_QU);
    Set_Cutoff_Frequency(Sampling_Freq, 8, &Gyro_Parameter);
#endif
}

void GET_MPU_DATA(void)
{
    ImuSensor_ReadReg_BuffAll();

#if ICM42688_ENABLE_FULL_PIPELINE
    int16_t ay = MPU_Data.AccX;
    int16_t ax = MPU_Data.AccY;
    int16_t az = MPU_Data.AccZ;

    int16_t gy = MPU_Data.GyroX;
    int16_t gx = MPU_Data.GyroY;
    int16_t gz = MPU_Data.GyroZ;

    sensors.gyro.x = -(gx - X_w_off) * SENSORS_DEG_PER_LSB_CFG;
    sensors.gyro.y = (gy - Y_w_off) * SENSORS_DEG_PER_LSB_CFG;
    sensors.gyro.z = (gz - Z_w_off) * SENSORS_DEG_PER_LSB_CFG;

    sensors.acc.x = -(ax) * SENSORS_G_PER_LSB_CFG;
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

    Acce_Correct[0] = (int16_t)(LPButterworth(Accel.x, &Butter_Buffer_Correct[0], &Butter_1HZ_Parameter_Acce));
    Acce_Correct[1] = (int16_t)(LPButterworth(Accel.y, &Butter_Buffer_Correct[1], &Butter_1HZ_Parameter_Acce));
    Acce_Correct[2] = (int16_t)(LPButterworth(Accel.z, &Butter_Buffer_Correct[2], &Butter_1HZ_Parameter_Acce));

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
#endif
}

#endif

#include "FFCY_NEW_ICM42688.h"
#include "i2c1.h"


#if IMU_SENSOR == ICM42688

ICM42688_RawData MPU_Data;
u32 IIC_Timeout_Cnt = 0;
u32 IIC_Timeout_Cnt_noTimesClear = 0;

static uint8_t s_device_address = ICM42688_ADDRESS_DEFAULT;

static uint8_t icm42688_normalize_address(uint8_t dev_address)
{
    if (dev_address > 0x7FU)
    {
        return (uint8_t)(dev_address >> 1);
    }

    return dev_address;
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
    I2C1_Status status = i2c1_mem_write(s_device_address, reg_addr, &data, 1U, 20000U);
    return (status == I2C1_OK) ? ICM42688_OK : ((status == I2C1_TIMEOUT) ? ICM42688_TIMEOUT : ICM42688_ERROR);
}

static ICM42688_Status icm42688_read_reg_current(uint8_t reg_addr, uint8_t *data)
{
    I2C1_Status status = i2c1_mem_read(s_device_address, reg_addr, data, 1U, 20000U);
    return (status == I2C1_OK) ? ICM42688_OK : ((status == I2C1_TIMEOUT) ? ICM42688_TIMEOUT : ICM42688_ERROR);
}

static ICM42688_Status icm42688_read_regs(uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    uint16_t i;

    if ((data == 0) || (len == 0U))
    {
        return ICM42688_ERROR;
    }

    for (i = 0; i < len; i++)
    {
        I2C1_Status status = i2c1_mem_read(s_device_address, (uint8_t)(reg_addr + i), &data[i], 1U, 20000U);
        if (status != I2C1_OK)
        {
            return (status == I2C1_TIMEOUT) ? ICM42688_TIMEOUT : ICM42688_ERROR;
        }
    }

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
    I2C1_Status status = i2c1_mem_write(icm42688_normalize_address(DevAddress), RegAddress, &Data, 1U, 20000U);
    if (status == I2C1_TIMEOUT)
    {
        IIC_Timeout_Cnt++;
        IIC_Timeout_Cnt_noTimesClear++;
    }
}

uint8_t ICM42688_ReadReg(uint8_t DevAddress, uint8_t RegAddress)
{
    uint8_t data = 0;
    I2C1_Status status = i2c1_mem_read(icm42688_normalize_address(DevAddress), RegAddress, &data, 1U, 20000U);
    if (status == I2C1_TIMEOUT)
    {
        IIC_Timeout_Cnt++;
        IIC_Timeout_Cnt_noTimesClear++;
    }

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

ICM42688_Status ICM42688_Init(void)
{
    i2c1_init(400000U);

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
    (void)ICM42688_Init();
}

void ImuSensor_ReadReg_BuffAll(void)
{
    uint8_t imu_sensor_buff[14] = {0};
    IIC_Timeout_Cnt = 0;

    if (icm42688_read_regs(ICM42688_TEMP_DATA1, imu_sensor_buff, 14U) != ICM42688_OK)
    {
        IIC_Timeout_Cnt++;
        IIC_Timeout_Cnt_noTimesClear++;
        return;
    }

    MPU_Data.AccX = (int16_t)((imu_sensor_buff[2] << 8) | imu_sensor_buff[3]);
    MPU_Data.AccY = (int16_t)((imu_sensor_buff[4] << 8) | imu_sensor_buff[5]);
    MPU_Data.AccZ = (int16_t)((imu_sensor_buff[6] << 8) | imu_sensor_buff[7]);
    MPU_Data.GyroX = (int16_t)((imu_sensor_buff[8] << 8) | imu_sensor_buff[9]);
    MPU_Data.GyroY = (int16_t)((imu_sensor_buff[10] << 8) | imu_sensor_buff[11]);
    MPU_Data.GyroZ = (int16_t)((imu_sensor_buff[12] << 8) | imu_sensor_buff[13]);
    MPU_Data.Temp = (int16_t)((imu_sensor_buff[0] << 8) | imu_sensor_buff[1]);
}

void GET_MPU_DATA(void)
{
    ImuSensor_ReadReg_BuffAll();
}



#endif /* IMU_SENSOR == BMI088 */


#include "FFCY_NEW_ICM42688.h"
#include "IMU_FilterPortable.h"  //包含IMU滤波器的头文件
#include "KalmanFilter.h"//包含卡尔曼滤波器的头文件
#include "systick_delay.h"

#define ICM42688_USE_SPI 0
#define ICM42688_USE_i2c 1

#define ICM42688_USE_mod ICM42688_USE_SPI


#if (ICM42688_USE_mod==ICM42688_USE_SPI)
    #include "spi2.h"
#endif

#if (ICM42688_USE_mod==ICM42688_USE_i2c)
    #include "i2c1.h"
#endif




MPU6050_Data MPU_Data;
MPU6050_Data MPU_FilteredData;

u32 IIC_Timeout_Cnt = 0;
u32 IIC_Timeout_Cnt_noTimesClear = 0;

#if ICM42688_GYRO_BIAS_ENABLE//陀螺仪偏置校准相关的静态变量
    static float s_gyro_bias_lsb[3] = {0.0f, 0.0f, 0.0f};
    static uint8_t s_gyro_bias_ready = 0U;
#endif

#if ICM42688_ACC_BIAS_ENABLE//加速度计偏置校准相关的静态变量
    static float s_acc_bias_lsb[3] = {0.0f, 0.0f, 0.0f};
    static uint8_t s_acc_bias_ready = 0U;
#endif

#if ICM42688_SOFT_FILTER_ENABLE//软件低通滤波器相关的静态变量
    static IMU_FilterPortableBiquadCoeff s_acc_lpf_coeff;
    static IMU_FilterPortableBiquadCoeff s_gyro_lpf_coeff;
    static IMU_FilterPortableBiquadState s_acc_lpf_state[3];
    static IMU_FilterPortableBiquadState s_gyro_lpf_state[3];
#endif

#if ICM42688_KALMAN_ENABLE//卡尔曼滤波器相关的静态变量
    static KalmanFilterState s_gyro_kalman[3];
    static KalmanFilterState s_acc_kalman[3];
#endif

static uint8_t s_device_address = ICM42688_ADDRESS_DEFAULT;

static int16_t icm42688_abs_i16(int16_t v)
{
    return (v >= 0) ? v : (int16_t)(-v);
}

static int32_t icm42688_abs_i32(int32_t v)
{
    return (v >= 0) ? v : -v;
}

static int16_t icm42688_sub_bias_lsb(int16_t raw, float bias)
{
    int32_t bias_i = (bias >= 0.0f) ? (int32_t)(bias + 0.5f) : (int32_t)(bias - 0.5f);
    int32_t corrected = (int32_t)raw - bias_i;

    if (corrected > 32767)
    {
        corrected = 32767;
    }
    else if (corrected < -32768)
    {
        corrected = -32768;
    }

    return (int16_t)corrected;
}

//判断加速度计读数的模长是否在合理范围内，以验证是否接近1g，适用于静止状态的校准和数据有效性检查
static uint8_t icm42688_acc_norm_is_valid_1g(int16_t ax, int16_t ay, int16_t az)
{
    float norm_sq = (float)ax * (float)ax + (float)ay * (float)ay + (float)az * (float)az;
    float one_g = (float)ICM42688_ACC_1G_LSB;
    float min_sq = one_g * one_g * ICM42688_ACC_NORM_MIN_RATIO * ICM42688_ACC_NORM_MIN_RATIO;
    float max_sq = one_g * one_g * ICM42688_ACC_NORM_MAX_RATIO * ICM42688_ACC_NORM_MAX_RATIO;

    return (norm_sq >= min_sq && norm_sq <= max_sq) ? 1U : 0U;
}

//陀螺仪偏置校准函数，采集一定数量的样本并计算平均值作为偏置，校准期间需要保持IMU静止
#if ICM42688_GYRO_BIAS_ENABLE
    static void icm42688_calibrate_gyro_bias(void)
    {
        uint32_t valid = 0U;
        uint32_t attempt = 0U;
        int32_t sum_x = 0;
        int32_t sum_y = 0;
        int32_t sum_z = 0;
        const uint32_t max_attempt = ICM42688_GYRO_BIAS_CAL_SAMPLES * 3U;

        s_gyro_bias_lsb[0] = 0.0f;
        s_gyro_bias_lsb[1] = 0.0f;
        s_gyro_bias_lsb[2] = 0.0f;
        s_gyro_bias_ready = 0U;

        while ((valid < ICM42688_GYRO_BIAS_CAL_SAMPLES) && (attempt < max_attempt))
        {
            int16_t gx;
            int16_t gy;
            int16_t gz;
            int16_t ax;
            int16_t ay;
            int16_t az;

            attempt++;
            IIC_Timeout_Cnt = 0U;

            gx = GetData_Gyro(ICM42688_GYRO_XOUT);
            gy = GetData_Gyro(ICM42688_GYRO_YOUT);
            gz = GetData_Gyro(ICM42688_GYRO_ZOUT);
            ax = GetData_Acc(ICM42688_ACCEL_XOUT);
            ay = GetData_Acc(ICM42688_ACCEL_YOUT);
            az = GetData_Acc(ICM42688_ACCEL_ZOUT);

            if (IIC_Timeout_Cnt != 0U)
            {
                continue;
            }

            if ((icm42688_abs_i16(gx) > ICM42688_GYRO_BIAS_MAX_ABS_LSB) ||
                (icm42688_abs_i16(gy) > ICM42688_GYRO_BIAS_MAX_ABS_LSB) ||
                (icm42688_abs_i16(gz) > ICM42688_GYRO_BIAS_MAX_ABS_LSB))
            {
                continue;
            }

            if (icm42688_acc_norm_is_valid_1g(ax, ay, az) == 0U)
            {
                continue;
            }

            sum_x += gx;
            sum_y += gy;
            sum_z += gz;
            valid++;
        }

        if (valid > 0U)
        {
            s_gyro_bias_lsb[0] = (float)sum_x / (float)valid;
            s_gyro_bias_lsb[1] = (float)sum_y / (float)valid;
            s_gyro_bias_lsb[2] = (float)sum_z / (float)valid;
            s_gyro_bias_ready = 1U;
        }
    }
#endif

//加速度计偏置校准函数（静止、水平放置时效果最佳）
#if ICM42688_ACC_BIAS_ENABLE
    static uint8_t icm42688_acc_is_near_expected(int16_t ax, int16_t ay, int16_t az)
    {
        const int32_t ex = (int32_t)ICM42688_ACC_BIAS_EXPECT_X_LSB;
        const int32_t ey = (int32_t)ICM42688_ACC_BIAS_EXPECT_Y_LSB;
        const int32_t ez = (int32_t)ICM42688_ACC_BIAS_EXPECT_Z_LSB;

        if (icm42688_abs_i32((int32_t)ax - ex) > (int32_t)ICM42688_ACC_BIAS_MAX_ABS_XY_LSB)
        {
            return 0U;
        }
        if (icm42688_abs_i32((int32_t)ay - ey) > (int32_t)ICM42688_ACC_BIAS_MAX_ABS_XY_LSB)
        {
            return 0U;
        }
        if (icm42688_abs_i32((int32_t)az - ez) > (int32_t)ICM42688_ACC_BIAS_MAX_1G_DEV_LSB)
        {
            return 0U;
        }

        return 1U;
    }

    static void icm42688_calibrate_acc_bias(void)
    {
        uint32_t valid = 0U;
        uint32_t attempt = 0U;
        int32_t sum_x = 0;
        int32_t sum_y = 0;
        int32_t sum_z = 0;
        const uint32_t max_attempt = ICM42688_ACC_BIAS_CAL_SAMPLES * 3U;

        s_acc_bias_lsb[0] = 0.0f;
        s_acc_bias_lsb[1] = 0.0f;
        s_acc_bias_lsb[2] = 0.0f;
        s_acc_bias_ready = 0U;

        while ((valid < ICM42688_ACC_BIAS_CAL_SAMPLES) && (attempt < max_attempt))
        {
            int16_t gx;
            int16_t gy;
            int16_t gz;
            int16_t ax;
            int16_t ay;
            int16_t az;

            attempt++;
            IIC_Timeout_Cnt = 0U;

            gx = GetData_Gyro(ICM42688_GYRO_XOUT);
            gy = GetData_Gyro(ICM42688_GYRO_YOUT);
            gz = GetData_Gyro(ICM42688_GYRO_ZOUT);
            ax = GetData_Acc(ICM42688_ACCEL_XOUT);
            ay = GetData_Acc(ICM42688_ACCEL_YOUT);
            az = GetData_Acc(ICM42688_ACCEL_ZOUT);

            if (IIC_Timeout_Cnt != 0U)
            {
                continue;
            }

            /* Reject samples that indicate motion */
            if ((icm42688_abs_i16(gx) > ICM42688_GYRO_BIAS_MAX_ABS_LSB) ||
                (icm42688_abs_i16(gy) > ICM42688_GYRO_BIAS_MAX_ABS_LSB) ||
                (icm42688_abs_i16(gz) > ICM42688_GYRO_BIAS_MAX_ABS_LSB))
            {
                continue;
            }

            if (icm42688_acc_norm_is_valid_1g(ax, ay, az) == 0U)
            {
                continue;
            }

            if (icm42688_acc_is_near_expected(ax, ay, az) == 0U)
            {
                continue;
            }

            sum_x += ax;
            sum_y += ay;
            sum_z += az;
            valid++;
        }

        if (valid > 0U)
        {
            const float avg_x = (float)sum_x / (float)valid;
            const float avg_y = (float)sum_y / (float)valid;
            const float avg_z = (float)sum_z / (float)valid;

            /* bias = measured_avg - expected */
            s_acc_bias_lsb[0] = avg_x - (float)ICM42688_ACC_BIAS_EXPECT_X_LSB;
            s_acc_bias_lsb[1] = avg_y - (float)ICM42688_ACC_BIAS_EXPECT_Y_LSB;
            s_acc_bias_lsb[2] = avg_z - (float)ICM42688_ACC_BIAS_EXPECT_Z_LSB;
            s_acc_bias_ready = 1U;
        }
    }
#endif

//由于有些I2C库要求7位地址，有些要求8位地址，所以这里做一个统一处理，确保无论传入7位还是8位地址都能正确使用
static uint8_t icm42688_normalize_address(uint8_t dev_address)
{
    if (dev_address > 0x7FU)
    {
        return (uint8_t)(dev_address >> 1);
    }

    return dev_address;
}

//尝试访问指定地址并读取WHO_AM_I寄存器以验证设备是否存在，返回相应的状态码
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

//当前使用的寄存器读写函数，内部根据使用的通信接口（SPI或I2C）进行相应的操作，并处理超时情况，更新超时计数器
static ICM42688_Status icm42688_write_reg_current(uint8_t reg_addr, uint8_t data)
{
#if (ICM42688_USE_mod==ICM42688_USE_SPI)
    /* SPI write: send register address (write = MSB=0) followed by data */
    SPI2_Status status;
    uint8_t tx[2] = { (uint8_t)(reg_addr & 0x7FU), data };
    uint8_t rx[2] = {0};

    status = spi2_transfer(tx, rx, 2U, 20000U);

    if (status == SPI2_TIMEOUT)
    {
        IIC_Timeout_Cnt++;
        IIC_Timeout_Cnt_noTimesClear++;
    }

    return (status == SPI2_OK) ? ICM42688_OK : ((status == SPI2_TIMEOUT) ? ICM42688_TIMEOUT : ICM42688_ERROR);
#endif	
	
#if (ICM42688_USE_mod==ICM42688_USE_i2c)	
    I2C1_Status status = i2c1_mem_write(s_device_address, reg_addr, &data, 1U, 20000U);

    if (status == I2C1_TIMEOUT)
    {
        IIC_Timeout_Cnt++;
        IIC_Timeout_Cnt_noTimesClear++;
    }

    return (status == I2C1_OK) ? ICM42688_OK : ((status == I2C1_TIMEOUT) ? ICM42688_TIMEOUT : ICM42688_ERROR);
#endif
}

//当前使用的寄存器读写函数，内部根据使用的通信接口（SPI或I2C）进行相应的操作，并处理超时情况，更新超时计数器
static ICM42688_Status icm42688_read_reg_current(uint8_t reg_addr, uint8_t *data)
{
#if (ICM42688_USE_mod==ICM42688_USE_SPI)
    /* SPI read: send register address with MSB=1, then read data */
    SPI2_Status status;
    uint8_t tx[2] = { (uint8_t)(reg_addr | 0x80U), 0xFFU };
    uint8_t rx[2] = {0};

    status = spi2_transfer(tx, rx, 2U, 20000U);

    if (status == SPI2_TIMEOUT)
    {
        IIC_Timeout_Cnt++;
        IIC_Timeout_Cnt_noTimesClear++;
    }

    if (status == SPI2_OK)
    {
        *data = rx[1];
        return ICM42688_OK;
    }

    return (status == SPI2_TIMEOUT) ? ICM42688_TIMEOUT : ICM42688_ERROR;
	
#elif (ICM42688_USE_mod==ICM42688_USE_i2c)
    I2C1_Status status = i2c1_mem_read(s_device_address, reg_addr, data, 1U, 20000U);

    if (status == I2C1_TIMEOUT)
    {
        IIC_Timeout_Cnt++;
        IIC_Timeout_Cnt_noTimesClear++;
    }

    return (status == I2C1_OK) ? ICM42688_OK : ((status == I2C1_TIMEOUT) ? ICM42688_TIMEOUT : ICM42688_ERROR);
#endif
	
}

//读取WHO_AM_I寄存器的值以验证设备是否存在，并返回相应的状态码
ICM42688_Status ICM42688_ReadWhoAmI(uint8_t *who_am_i)
{
    if (who_am_i == 0)
    {
        return ICM42688_ERROR;
    }

    return icm42688_read_reg_current(ICM42688_WHO_AM_I, who_am_i);
}

//对外暴露的寄存器写函数，内部根据使用的通信接口（SPI或I2C）进行相应的操作，并处理超时情况，更新超时计数器
void ICM42688_WriteReg(uint8_t DevAddress, uint8_t RegAddress, uint8_t Data)
{
#if (ICM42688_USE_mod==ICM42688_USE_SPI)
    SPI2_Status status;
    uint8_t tx[2] = { (uint8_t)(RegAddress & 0x7FU), Data };
    uint8_t rx[2] = {0};

    status = spi2_transfer(tx, rx, 2U, 20000U);
    if (status == SPI2_TIMEOUT)
    {
        IIC_Timeout_Cnt++;
        IIC_Timeout_Cnt_noTimesClear++;
    }
#elif (ICM42688_USE_mod==ICM42688_USE_i2c)
    I2C1_Status status = i2c1_mem_write(icm42688_normalize_address(DevAddress), RegAddress, &Data, 1U, 20000U);
    if (status == I2C1_TIMEOUT)
    {
        IIC_Timeout_Cnt++;
        IIC_Timeout_Cnt_noTimesClear++;
    }
#endif
}

//对外暴露的寄存器读函数，内部根据使用的通信接口（SPI或I2C）进行相应的操作，并处理超时情况，更新超时计数器
uint8_t ICM42688_ReadReg(uint8_t DevAddress, uint8_t RegAddress)
{
    uint8_t data = 0;
#if (ICM42688_USE_mod==ICM42688_USE_SPI)
    SPI2_Status status;
    uint8_t tx[2] = { (uint8_t)(RegAddress | 0x80U), 0xFFU };
    uint8_t rx[2] = {0};

    status = spi2_transfer(tx, rx, 2U, 20000U);
    if (status == SPI2_TIMEOUT)
    {
        IIC_Timeout_Cnt++;
        IIC_Timeout_Cnt_noTimesClear++;
        return 0;
    }

    if (status == SPI2_OK)
    {
        return rx[1];
    }

    return 0;
	
#elif (ICM42688_USE_mod==ICM42688_USE_i2c)
    I2C1_Status status = i2c1_mem_read(icm42688_normalize_address(DevAddress), RegAddress, &data, 1U, 20000U);
    if (status == I2C1_TIMEOUT)
    {
        IIC_Timeout_Cnt++;
        IIC_Timeout_Cnt_noTimesClear++;
    }

    return data;
#endif
	
}

//读取陀螺仪或加速度计的高字节和低字节寄存器，并合成一个16位有符号整数返回
int16_t GetData_Gyro(uint8_t REG_Address)
{
    uint8_t hd = ICM42688_ReadReg(s_device_address, REG_Address);
    uint8_t ld = ICM42688_ReadReg(s_device_address, (uint8_t)(REG_Address + 1U));
    return (int16_t)(((uint16_t)hd << 8) | ld);
}

//读取陀螺仪或加速度计的高字节和低字节寄存器，并合成一个16位有符号整数返回
int16_t GetData_Acc(uint8_t REG_Address)
{
    uint8_t hd = ICM42688_ReadReg(s_device_address, REG_Address);
    uint8_t ld = ICM42688_ReadReg(s_device_address, (uint8_t)(REG_Address + 1U));
    return (int16_t)(((uint16_t)hd << 8) | ld);
}

//读取ICM42688的所有相关寄存器数据到全局变量MPU_Data中
void ImuSensor_ReadReg_BuffAll(void)
{
    int16_t imu_sensor_buff[6];

    IIC_Timeout_Cnt = 0;

    imu_sensor_buff[0] = GetData_Gyro(ICM42688_GYRO_XOUT);
    imu_sensor_buff[1] = GetData_Gyro(ICM42688_GYRO_YOUT);
    imu_sensor_buff[2] = GetData_Gyro(ICM42688_GYRO_ZOUT);
    imu_sensor_buff[3] = GetData_Acc(ICM42688_ACCEL_XOUT);
    imu_sensor_buff[4] = GetData_Acc(ICM42688_ACCEL_YOUT);
    imu_sensor_buff[5] = GetData_Acc(ICM42688_ACCEL_ZOUT);

    if (IIC_Timeout_Cnt == 0)
    {
        MPU_Data.GyroX = imu_sensor_buff[0];
        MPU_Data.GyroY = imu_sensor_buff[1];
        MPU_Data.GyroZ = imu_sensor_buff[2];
        MPU_Data.AccX = imu_sensor_buff[3];
        MPU_Data.AccY = imu_sensor_buff[4];
        MPU_Data.AccZ = imu_sensor_buff[5];
        MPU_Data.Temp = (int16_t)(((uint16_t)ICM42688_ReadReg(s_device_address, ICM42688_TEMP_DATA1) << 8) |
                                  ICM42688_ReadReg(s_device_address, (uint8_t)(ICM42688_TEMP_DATA1 + 1U)));
    }
}

//处理原始数据得到滤波后的加速度计和陀螺仪数据，并存储在MPU_FilteredData中
void ImuSensor_ProcessData(void){
    ICM42688_RawData raw_data;

    raw_data = MPU_Data;
    MPU_FilteredData = raw_data;

//===========================================================================
//陀螺仪偏置校准和跟踪
//===========================================================================
#if ICM42688_GYRO_BIAS_ENABLE 
    if (s_gyro_bias_ready != 0U)
    {
        MPU_FilteredData.GyroX = icm42688_sub_bias_lsb(MPU_FilteredData.GyroX, s_gyro_bias_lsb[0]);
        MPU_FilteredData.GyroY = icm42688_sub_bias_lsb(MPU_FilteredData.GyroY, s_gyro_bias_lsb[1]);
        MPU_FilteredData.GyroZ = icm42688_sub_bias_lsb(MPU_FilteredData.GyroZ, s_gyro_bias_lsb[2]);
    }
#endif
    
//===========================================================================
//加速度计偏置校准
//===========================================================================
#if ICM42688_ACC_BIAS_ENABLE 
    if (s_acc_bias_ready != 0U)
    {
        MPU_FilteredData.AccX = icm42688_sub_bias_lsb(MPU_FilteredData.AccX, s_acc_bias_lsb[0]);
        MPU_FilteredData.AccY = icm42688_sub_bias_lsb(MPU_FilteredData.AccY, s_acc_bias_lsb[1]);
        MPU_FilteredData.AccZ = icm42688_sub_bias_lsb(MPU_FilteredData.AccZ, s_acc_bias_lsb[2]);
    }
#endif

//===========================================================================
//陀螺仪偏置跟踪
//===========================================================================
#if ICM42688_GYRO_BIAS_TRACK_ENABLE
    if ((icm42688_abs_i16(MPU_FilteredData.GyroX) < ICM42688_GYRO_STILL_THRESH_LSB) &&
        (icm42688_abs_i16(MPU_FilteredData.GyroY) < ICM42688_GYRO_STILL_THRESH_LSB) &&
        (icm42688_abs_i16(MPU_FilteredData.GyroZ) < ICM42688_GYRO_STILL_THRESH_LSB) &&
        (icm42688_acc_norm_is_valid_1g(raw_data.AccX, raw_data.AccY, raw_data.AccZ) != 0U))
    {
        s_gyro_bias_lsb[0] += ICM42688_GYRO_BIAS_TRACK_ALPHA * ((float)raw_data.GyroX - s_gyro_bias_lsb[0]);
        s_gyro_bias_lsb[1] += ICM42688_GYRO_BIAS_TRACK_ALPHA * ((float)raw_data.GyroY - s_gyro_bias_lsb[1]);
        s_gyro_bias_lsb[2] += ICM42688_GYRO_BIAS_TRACK_ALPHA * ((float)raw_data.GyroZ - s_gyro_bias_lsb[2]);
    }
#endif

//===========================================================================
//软件低通滤波
//===========================================================================
#if ICM42688_SOFT_FILTER_ENABLE
    MPU_FilteredData.GyroX = (int16_t)IMU_FilterPortable_Process((float)MPU_FilteredData.GyroX, &s_gyro_lpf_state[0], &s_gyro_lpf_coeff);
    MPU_FilteredData.GyroY = (int16_t)IMU_FilterPortable_Process((float)MPU_FilteredData.GyroY, &s_gyro_lpf_state[1], &s_gyro_lpf_coeff);
    MPU_FilteredData.GyroZ = (int16_t)IMU_FilterPortable_Process((float)MPU_FilteredData.GyroZ, &s_gyro_lpf_state[2], &s_gyro_lpf_coeff);
    MPU_FilteredData.AccX = (int16_t)IMU_FilterPortable_Process((float)MPU_FilteredData.AccX, &s_acc_lpf_state[0], &s_acc_lpf_coeff);
    MPU_FilteredData.AccY = (int16_t)IMU_FilterPortable_Process((float)MPU_FilteredData.AccY, &s_acc_lpf_state[1], &s_acc_lpf_coeff);
    MPU_FilteredData.AccZ = (int16_t)IMU_FilterPortable_Process((float)MPU_FilteredData.AccZ, &s_acc_lpf_state[2], &s_acc_lpf_coeff);
#endif

//===========================================================================
//卡尔曼滤波
//===========================================================================
#if ICM42688_KALMAN_ENABLE
    
    MPU_FilteredData.GyroX = (int16_t)KalmanFilter_Update(&s_gyro_kalman[0], (float)MPU_FilteredData.GyroX);
    MPU_FilteredData.GyroY = (int16_t)KalmanFilter_Update(&s_gyro_kalman[1], (float)MPU_FilteredData.GyroY);
    MPU_FilteredData.GyroZ = (int16_t)KalmanFilter_Update(&s_gyro_kalman[2], (float)MPU_FilteredData.GyroZ);
    MPU_FilteredData.AccX = (int16_t)KalmanFilter_Update(&s_acc_kalman[0], (float)MPU_FilteredData.AccX);
    MPU_FilteredData.AccY = (int16_t)KalmanFilter_Update(&s_acc_kalman[1], (float)MPU_FilteredData.AccY);
    MPU_FilteredData.AccZ = (int16_t)KalmanFilter_Update(&s_acc_kalman[2], (float)MPU_FilteredData.AccZ);
#endif
}

//初始化ICM42688传感器，配置通信接口，并进行基本的寄存器设置，最后返回初始化状态
ICM42688_Status ICM42688_Init(void)
{
#if (ICM42688_USE_mod==ICM42688_USE_SPI)
    /* initialize SPI2 with a reasonable prescaler if using SPI */
    #ifndef ICM42688_SPI_PRESCALER
    #define ICM42688_SPI_PRESCALER SPI_BaudRatePrescaler_16
    #endif
    spi2_init(ICM42688_SPI_PRESCALER); 
	
#elif (ICM42688_USE_mod==ICM42688_USE_i2c)
    i2c1_init(400000U);
#endif

    /* Probe device: for I2C try default and alternate addresses; for SPI try default once */
#if (ICM42688_USE_mod==ICM42688_USE_i2c)
    if (icm42688_probe_address(ICM42688_ADDRESS_DEFAULT) != ICM42688_OK)
    {
        if (icm42688_probe_address(ICM42688_ADDRESS_ALT) != ICM42688_OK)
        {
            return ICM42688_TIMEOUT;
        }
    }
#elif (ICM42688_USE_mod==ICM42688_USE_SPI)
    if (icm42688_probe_address(ICM42688_ADDRESS_DEFAULT) != ICM42688_OK)
    {
        return ICM42688_TIMEOUT;
    }
#endif

    /* Common sensor configuration (works over I2C or SPI via icm42688_write_reg_current) */
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

    if (icm42688_write_reg_current(ICM42688_GYRO_ACCEL_CONFIG0, ICM42688_HW_FILTER_CFG) != ICM42688_OK) //设置陀螺仪和加速度计的数字低通滤波器
    {
        return ICM42688_TIMEOUT;
    }

    if (icm42688_write_reg_current(ICM42688_INT_CONFIG, 0x1B) != ICM42688_OK)
    {
        return ICM42688_TIMEOUT;
    }

    return ICM42688_OK;
}

//对外暴露的初始化函数，内部调用ICM42688_Init进行设备初始化，并进行陀螺仪和加速度计的偏置校准，以及软件滤波器和卡尔曼滤波器的初始化
void ImuSensor_Init(void)
{

    /* Try to initialize device several times instead of infinite loop to avoid hanging
       If initialization fails after attempts, give up and return (caller must handle) */
#ifndef ICM42688_INIT_MAX_ATTEMPTS
#define ICM42688_INIT_MAX_ATTEMPTS 5
#endif
    int attempts = 0;
    while ((ICM42688_Init() != ICM42688_OK) && (attempts++ < ICM42688_INIT_MAX_ATTEMPTS))
    {
        systick_delay_ms(200);
    }

    /* short delay after init (or after giving up) */
    systick_delay_ms(1000);

#if ICM42688_GYRO_BIAS_ENABLE
    icm42688_calibrate_gyro_bias();
#endif

#if ICM42688_ACC_BIAS_ENABLE
    icm42688_calibrate_acc_bias();
#endif
	
	systick_delay_ms(1000);
	
#if ICM42688_SOFT_FILTER_ENABLE   
    IMU_FilterPortable_DesignLP2(ICM42688_SOFT_FILTER_SAMPLE_HZ, ICM42688_acc_SOFT_FILTER_CUTOFF_HZ, &s_acc_lpf_coeff);
    IMU_FilterPortable_DesignLP2(ICM42688_SOFT_FILTER_SAMPLE_HZ, ICM42688_gyro_SOFT_FILTER_CUTOFF_HZ, &s_gyro_lpf_coeff);

    IMU_FilterPortable_Init(&s_acc_lpf_state[0], ICM42688_SOFT_FILTER_WARMUP_COUNT);
    IMU_FilterPortable_Init(&s_acc_lpf_state[1], ICM42688_SOFT_FILTER_WARMUP_COUNT);
    IMU_FilterPortable_Init(&s_acc_lpf_state[2], ICM42688_SOFT_FILTER_WARMUP_COUNT);
    IMU_FilterPortable_Init(&s_gyro_lpf_state[0], ICM42688_SOFT_FILTER_WARMUP_COUNT);
    IMU_FilterPortable_Init(&s_gyro_lpf_state[1], ICM42688_SOFT_FILTER_WARMUP_COUNT);
    IMU_FilterPortable_Init(&s_gyro_lpf_state[2], ICM42688_SOFT_FILTER_WARMUP_COUNT);
#endif

	systick_delay_ms(1000);
	
#if ICM42688_KALMAN_ENABLE
    /* 初始化卡尔曼滤波器，参数可根据需要调整 */
    for (int i = 0; i < 3; ++i)
    {
        KalmanFilter_Init(&s_gyro_kalman[i] , 0.001f    , 0.5f      , 1.0f      , 0.0f);
	    KalmanFilter_Init(&s_acc_kalman[i]  , 0.001f    , 0.5f      , 1.0f      , 0.0f);
    }
#endif
}

//对外暴露的获取数据函数，内部调用ImuSensor_ReadReg_BuffAll读取原始数据，并调用ImuSensor_ProcessData进行数据处理，得到滤波后的结果
void GET_MPU_DATA(void)
{
    ImuSensor_ReadReg_BuffAll(); 
    ImuSensor_ProcessData();
}

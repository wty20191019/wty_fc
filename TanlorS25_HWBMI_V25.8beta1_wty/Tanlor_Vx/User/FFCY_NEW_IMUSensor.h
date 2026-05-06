#ifndef __FFCY_NEW_IMU_SENSOR_H
#define __FFCY_NEW_IMU_SENSOR_H

#define MPU6500_G_PER_LSB_     (float)((2 * 6) / 65536.0)
#define SENSORS_G_PER_LSB_CFG     MPU6500_G_PER_LSB_

#define MPU6500_DEG_PER_LSB_2000 (float)((2 * 2000.0) / 65536.0)
#define SENSORS_DEG_PER_LSB_CFG   MPU6500_DEG_PER_LSB_2000

typedef struct
{
	int16_t AccX;	int16_t AccY;	int16_t AccZ;
	int16_t GyroX;	int16_t GyroY;	int16_t GyroZ;
} MPU6050_Data;

extern MPU6050_Data MPU_Data;



extern Vector3f Accel, Gyro;
extern float X_Origion, Y_Origion, Z_Origion;
extern float IMU_K[3];//标度误差
extern float IMU_B[3];//零位误差
extern int16_t Acce_Correct[3];  //加速度计校准容
extern Vector3f gyro_filter, accel_filter, gyro_filter_QU;
extern Vector3f gyro_nofilter;
void GET_MPU_DATA(void);
void ImuSensor_ReadReg_BuffAll(void);
void ImuSensor_Init(void);


int16_t GetData_Gyro(uint8_t REG_Address);
int16_t GetData_Acc(uint8_t REG_Address);





#endif
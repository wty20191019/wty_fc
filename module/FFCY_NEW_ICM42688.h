#ifndef __FFCY_NEW_ICM42688_H__
#define __FFCY_NEW_ICM42688_H__

#include "stm32f4xx.h"

#define ICM42688_ADDRESS_DEFAULT      0x68
#define ICM42688_ADDRESS_ALT          0x69
#define ICM42688_WHO_AM_I            0x75
#define ICM42688_WHOAMI_VALUE        0x47
#define ICM42688_REG_BANK_SEL        0x76
#define ICM42688_PWR_MGMT0           0x4E
#define ICM42688_ACCEL_CONFIG0       0x50
#define ICM42688_GYRO_CONFIG0        0x4F
#define ICM42688_GYRO_ACCEL_CONFIG0  0x52
#define ICM42688_INT_CONFIG          0x14
#define ICM42688_TEMP_DATA1          0x1D

#define ICM42688_GYRO_ODR            0x06
#define ICM42688_GYRO_RNG            0x00
#define ICM42688_ACC_ODR             0x06
#define ICM42688_ACC_RNG             0x02



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
extern u32 IIC_Timeout_Cnt;
extern u32 IIC_Timeout_Cnt_noTimesClear;

void ImuSensor_Init(void);
ICM42688_Status ICM42688_Init(void);
void ImuSensor_ReadReg_BuffAll(void);
ICM42688_Status ICM42688_ReadWhoAmI(uint8_t *who_am_i);
void ICM42688_WriteReg(uint8_t DevAddress, uint8_t RegAddress, uint8_t Data);
uint8_t ICM42688_ReadReg(uint8_t DevAddress, uint8_t RegAddress);
int16_t GetData_Gyro(uint8_t REG_Address);
int16_t GetData_Acc(uint8_t REG_Address);
void GET_MPU_DATA(void);

#endif

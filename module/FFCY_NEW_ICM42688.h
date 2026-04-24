#ifndef __FFCY_NEW_ICM42688_H__
#define __FFCY_NEW_ICM42688_H__

#include "stm32f4xx.h"

//ADDRESS
#define ICM42688_ADDR_LOW            0x68
#define ICM42688_ADDR_HIGH           0x69
#define ICM42688_ADDRESS_DEFAULT     ICM42688_ADDR_LOW
#define ICM42688_ADDRESS_ALT         ICM42688_ADDR_HIGH

//CHIP ID
#define ICM42688_WHO_AM_I            0x75
#define ICM42688_WHOAMI_VALUE        0x47

//REGISTER
#define ICM42688_REG_BANK_SEL        0x76
#define ICM42688_PWR_MGMT0           0x4E
#define ICM42688_ACCEL_CONFIG0       0x50
#define ICM42688_GYRO_CONFIG0        0x4F
#define ICM42688_GYRO_ACCEL_CONFIG0   0x52
#define ICM42688_INT_CONFIG          0x14
#define ICM42688_TEMP_DATA1          0x1D

//ACC_BANDWITH
#define ICM42688_ACC_ODR_200hz       0x06
#define ICM42688_ACC_ODR_400hz       0x07
#define ICM42688_ACC_ODR_800hz       0x08
#define ICM42688_ACC_ODR_1600hz      0x09
#define ICM42688_ACC_ODR             ICM42688_ACC_ODR_200hz

//ACC_RANGE
#define ICM42688_ACC_RNG_6G          0x02
#define ICM42688_ACC_RNG             ICM42688_ACC_RNG_6G

//GYRO_ODR
#define ICM42688_GYRO_ODR_200hz      0x06
#define ICM42688_GYRO_ODR_400hz      0x07
#define ICM42688_GYRO_ODR_800hz      0x08
#define ICM42688_GYRO_ODR_1600hz     0x09
#define ICM42688_GYRO_ODR            ICM42688_GYRO_ODR_200hz

//GYRO_RANGE
#define ICM42688_GYRO_RNG_2000       0x00
#define ICM42688_GYRO_RNG            ICM42688_GYRO_RNG_2000

//DATA OUT
#define ICM42688_ACCEL_XOUT          0x1F
#define ICM42688_ACCEL_YOUT          0x21
#define ICM42688_ACCEL_ZOUT          0x23
#define ICM42688_GYRO_XOUT           0x25
#define ICM42688_GYRO_YOUT           0x27
#define ICM42688_GYRO_ZOUT           0x29



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

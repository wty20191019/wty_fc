#ifndef __FFCY_NEW_ICM42688_H__
#define __FFCY_NEW_ICM42688_H__

#include "stm32f4xx.h"

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

extern ICM42688_RawData MPU_Data;

void ImuSensor_Init(void);
ICM42688_Status ICM42688_Init(void);
void ImuSensor_ReadReg_BuffAll(void);
ICM42688_Status ICM42688_ReadWhoAmI(uint8_t *who_am_i);
ICM42688_Status GET_MPU_DATA(void);

#endif

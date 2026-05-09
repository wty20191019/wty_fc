#ifndef __FFCY_NEW_ICM42688_H
#define __FFCY_NEW_ICM42688_H

#include "stdint.h"



/* Registers (bank 0) */
#define ICM42688_WHO_AM_I            0x75
#define ICM42688_WHOAMI_VALUE        0x47
#define ICM42688_REG_BANK_SEL        0x76
#define ICM42688_PWR_MGMT0           0x4E
#define ICM42688_GYRO_CONFIG0        0x4F
#define ICM42688_ACCEL_CONFIG0       0x50
#define ICM42688_GYRO_ACCEL_CONFIG0  0x52
#define ICM42688_INT_CONFIG          0x14
#define ICM42688_TEMP_DATA1          0x1D

/* Data registers */
#define ICM42688_ACCEL_XOUT          0x1F
#define ICM42688_ACCEL_YOUT          0x21
#define ICM42688_ACCEL_ZOUT          0x23
#define ICM42688_GYRO_XOUT           0x25
#define ICM42688_GYRO_YOUT           0x27
#define ICM42688_GYRO_ZOUT           0x29

/* Config preset (matches existing wty I2C implementation) */
#define ICM42688_GYRO_ODR            0x06
#define ICM42688_GYRO_RNG            0x00
#define ICM42688_ACC_ODR             0x06
#define ICM42688_ACC_RNG             0x02

/* Sensor-side digital low-pass filter setting.
 * Register: GYRO_ACCEL_CONFIG0(0x52)
 * [7:4] accel UI filter BW, [3:0] gyro UI filter BW.
 */
#define ICM42688_FILTER_BW_180HZ     0x00U
#define ICM42688_FILTER_BW_121HZ     0x01U
#define ICM42688_FILTER_BW_73HZ      0x02U
#define ICM42688_FILTER_BW_53HZ      0x03U
#define ICM42688_FILTER_BW_34HZ      0x04U
#define ICM42688_FILTER_BW_25HZ      0x05U
#define ICM42688_FILTER_BW_16HZ      0x06U
#define ICM42688_FILTER_BW_8HZ       0x07U

#define ICM42688_FILTER_PRESET       ICM42688_FILTER_BW_34HZ

#define ICM42688_ACCEL_UI_FILT_BW    ICM42688_FILTER_PRESET
#define ICM42688_GYRO_UI_FILT_BW     ICM42688_FILTER_PRESET
#define ICM42688_HW_FILTER_CFG       ((uint8_t)(((ICM42688_ACCEL_UI_FILT_BW & 0x0FU) << 4) | (ICM42688_GYRO_UI_FILT_BW & 0x0FU)))

#endif

#ifndef __FFCY_NEW_ICM42688_COMPAT_H__
#define __FFCY_NEW_ICM42688_COMPAT_H__

typedef struct
{
    float x;
    float y;
    float z;
} Vector3f;

typedef struct
{
    uint32_t reserved;
} Testime;

#define Sampling_Freq 200
#define GRAVITY_MSS 9.80665f
#define AcceMax_1G 8192.0f
#define GYRO_CALIBRATION_COFF (2000.0f / 32768.0f)
#define GYRO_LPF_CUTOFF_FREQ 25
#define ACCEL_LPF_CUTOFF_FREQ 30
#define One_G_TO_Accel (AcceMax_1G / GRAVITY_MSS)
#define SENSORS_G_PER_LSB_CFG  (1.0f / AcceMax_1G)
#define SENSORS_DEG_PER_LSB_CFG (GYRO_CALIBRATION_COFF)

typedef struct
{
    float a[3];
    float b[3];
} Butter_Parameter;

typedef struct
{
    float Input_Butter[3];
    float Output_Butter[3];
} Butter_BufferData;

typedef struct
{
    float state[2];
} lpf2p_data;

typedef struct
{
    Vector3f gyro;
    Vector3f acc;
} IMU_SensorFrame;

typedef struct
{
    float x;
    float y;
    float z;
} Axis3f;

IMU_SensorFrame sensors = {0};
IMU_SensorFrame sensors1 = {0};
Vector3f Body_Frame = {0};
Vector3f MBody_Frame = {0};
Axis3f Gyro_feedback = {0};

float MPitch_Gyro = 0.0f;
float MRoll_Gyro = 0.0f;
float MYaw_Gyro = 0.0f;

float X_w_off = 0.0f;
float Y_w_off = 0.0f;
float Z_w_off = 0.0f;

lpf2p_data gyroLpf[3] = {0};
lpf2p_data accLpf[3] = {0};

static inline void lpf2pInit(lpf2p_data *filter, float sampleFreq, float cutoffFreq)
{
    (void)sampleFreq;
    (void)cutoffFreq;

    if (filter != 0)
    {
        filter->state[0] = 0.0f;
        filter->state[1] = 0.0f;
    }
}

static inline void applyAxis3fLpf(lpf2p_data filter[3], Vector3f *axis)
{
    (void)filter;
    (void)axis;
}

static inline void Set_Cutoff_Frequency(float samplingFreq, float cutoffFreq, Butter_Parameter *parameter)
{
    (void)samplingFreq;
    (void)cutoffFreq;

    if (parameter != 0)
    {
        parameter->a[0] = 1.0f;
        parameter->a[1] = 0.0f;
        parameter->a[2] = 0.0f;
        parameter->b[0] = 1.0f;
        parameter->b[1] = 0.0f;
        parameter->b[2] = 0.0f;
    }
}

static inline float LPButterworth(float curr_input, Butter_BufferData *Buffer, Butter_Parameter *Parameter)
{
    (void)Buffer;
    (void)Parameter;
    return curr_input;
}

#endif

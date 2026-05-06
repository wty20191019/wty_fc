#ifndef _CALIBRATION_H
#define _CALIBRATION_H

void Consol_main();
extern u8 RC_Calibration_Flag ;
extern u8 ACC_Calibration_Flag;

typedef struct
{
 float x;
 float y;
 float z;
}Acce_Unit;

typedef struct
{
 float x;
 float y;
 float z;
}Mag_Unit;


typedef struct {
    int16_t x_max;
    int16_t y_max;
    int16_t z_max;
    int16_t x_min;
    int16_t y_min;
    int16_t z_min;
    float x_offset;
    float y_offset;
    float z_offset;
}Calibration;


void Accel_Calibration_Check(void);
uint8_t Accel_Calibartion(void);
u8 IMU_Acc_Cal_Init(void);
void Mag_Calibration_Check(void);
uint8_t Mag_Calibartion(Mag_Unit MagData,Vector3f_Body Circle_Angle_Calibartion);
void Reset_Mag_Calibartion(uint8_t Type);
void Reset_Accel_Calibartion(uint8_t Type);

void Mag_LS_Init(void);
uint8_t Mag_Calibartion_LS(Mag_Unit MagData,Vector3f_Body Circle_Angle_Calibartion);


void RC_Calibration_Trigger(void);
u8 RC_Calibration_Check(uint16 *rc_date);
void ESC_HardWave_Init(void);//只初始化校准电调的必要资源
void ESC_Calibration_Check(void);
uint8_t Check_Calibration_Flag(void);
void Reset_RC_Calibartion(uint8_t Type);
void Horizontal_Calibration_Check(void);
void Horizontal_Calibration_Init(void);
void Headless_Mode_Calibration_Check(void);
extern Acce_Unit acce_sample[6];
extern uint8_t  Mag_Calibration_Mode;
extern uint8_t flight_direction;
extern Acce_Unit Accel_Offset_Read,Accel_Scale_Read;
extern Mag_Unit DataMag;
extern Mag_Unit Mag_Offset_Read;
extern Calibration Mag;
extern uint8_t Mag_360_Flag[3][36];
extern uint16_t Mag_Is_Okay_Flag[3];
extern float Yaw_Correct;
extern Vector2f MagN;
extern int16_t Mag_Offset[3];
extern float Mag_Data[3];
extern float HMC5883L_Yaw;
extern Vector_RC  RC_Calibration[8];
extern float Hor_Accel_Offset[3];
extern float mag_a,mag_b,mag_c,mag_r;
extern float ESC_Calibration_Flag;
extern float Pitch_Offset,Roll_Offset;
extern float Headless_Mode_Yaw;

void GYRO_Calibration(void);
extern float  X_w_off , Y_w_off , Z_w_off ;

#endif


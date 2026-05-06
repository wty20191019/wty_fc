#ifndef _FFCY_SDK_H
#define _FFCY_SDK_H


void FFCY_SDK_Control(uint8_t force_brake_flag);
void MY_SDK_REST(void);
void SDK_Machine(void);
void SDK_SPEED_output(float Sx, float Sy);
void SDK_Posion_output(float Sx, float Sy);
void SDK_Get_Nowposion(float *Px, float *Py);
void SDK_Turnto_HANG1s(u16  NowSTATE);
float SDK_Calcuate_X(float NowXPOS);
float SDK_Calcuate_Y(float NowYPOS);
void FFCY_SDK_Control(uint8_t force_brake_flag);
void Target_Location(void);
u8 MY_SDK_Yaw_WaitFinish(void);
u8 MY_SDK_Yaw_WaitFinish(void);
void MY_SDK_Yaw_Calcuate(float MYSDK_YAW_TARG_Deta, float GYRO_SPEED);
void SDK_Wait_For_T265(u16  NowSTATE);
void Camera_Decoder(u8 Device_Choose);

extern float FL_X, FL_Y;
extern u8 Search_Circle_Step;
extern float Openmv_Correct;
extern float Lonth_Head, Lonth_Left1, Lonth_Back, Lonth_Left2, Lonth_Head2, Lonth_Left3, Lonth_Back2, Lonth_Left4, Lonth_Head3;
extern int  Search_Location_Buff[20][3];
extern float Fire_UWB_Y[10];
extern float Fire_UWB_X[10];
extern float UWB_I, UWB_Err;

#endif

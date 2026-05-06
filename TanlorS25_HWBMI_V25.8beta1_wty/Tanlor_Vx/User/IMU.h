#ifndef __IMU_H
#define __IMU_H



typedef struct
{
  float q[4];
  float angle[3];
}_Attitude_Tag;

extern float Yaw, Pitch, Roll;
extern float MPitch, MRoll, MYaw, MMYaw;
extern float Yaw_Gyro,Pitch_Gyro,Roll_Gyro,Yaw_Gyro_Earth_Frame;
extern float MYaw_Gyro , MPitch_Gyro , MRoll_Gyro ;

void Vector_From_BodyFrame2EarthFrame(Vector3f *bf,Vector3f *ef);

void Quaternion_Init(void);
void DirectionConsineMatrix(Vector3f gyro,Vector3f acc,Vector2f magn);

extern  float rMat[3][3];
extern float Gyro_Length, MGyro_Length,Gyro_Length_Filter;
extern float Gyro_Delta_Length;

extern float gyro[3];

#endif

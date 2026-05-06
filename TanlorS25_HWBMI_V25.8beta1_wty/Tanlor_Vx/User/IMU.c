#include "Headfile.h"
#include "IMU.h"


extern float T265_data_Yaw; extern u8 T265_Confidence;

float Yaw = 0, Pitch = 0, Roll = 0;							//四元数计算出的角度
float Yaw_Gyro = 0, Pitch_Gyro = 0, Roll_Gyro = 0;
float MYaw_Gyro = 0, MPitch_Gyro = 0, MRoll_Gyro = 0;
float Gyro_Length = 0;//陀螺仪模长
float Yaw_Gyro_Earth_Frame = 0;
float q0 = 1.0f, q1 = 0, q2 = 0, q3 = 0;
float rMat[3][3];


//#define dt 0.005f	
//void AHRSUpdate_GraDes_TimeSync(float gx, float gy, float gz, float ax, float ay, float az, u8 Senser_Choice)
//{
//	float recipNorm;	float s0, s1, s2, s3;	float qDot1, qDot2, qDot3, qDot4;
//	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2;
//	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
//	float beta_temp = 0;
//	
//	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz)*DEG2RAD;	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy)*DEG2RAD;
//	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx)*DEG2RAD;	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx)*DEG2RAD;
//
//	Gyro_Length = sqrt(Yaw_Gyro*Yaw_Gyro + Pitch_Gyro*Pitch_Gyro + Roll_Gyro*Roll_Gyro);//单位deg/s
//	Yaw_Gyro_Earth_Frame = -Sin_Roll*gx + Cos_Roll*Sin_Pitch *gy + Cos_Pitch * Cos_Roll *gz;    //{-sinθ  cosθsin Φ  cosθcosΦ  }
//
//	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
//	{
//		recipNorm = invSqrt(ax * ax + ay * ay + az * az);	ax *= recipNorm;	ay *= recipNorm;	az *= recipNorm;
//
//		_2q0 = 2.0f * q0;	_2q1 = 2.0f * q1;	_2q2 = 2.0f * q2;	_2q3 = 2.0f * q3;
//		_4q0 = 4.0f * q0;	_4q1 = 4.0f * q1;	_4q2 = 4.0f * q2;
//		_8q1 = 8.0f * q1;	_8q2 = 8.0f * q2;
//		q0q0 = q0 * q0;	q0q1 = q0 * q1;	q0q2 = q0 * q2;	q0q3 = q0 * q3;
//		q1q1 = q1 * q1;	q1q2 = q1 * q2;	q1q3 = q1 * q3;
//		q2q2 = q2 * q2;	q2q3 = q2 * q3;
//		q3q3 = q3 * q3;
//
//		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
//		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
//		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
//		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
//
//		recipNorm = invSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3); s0 *= recipNorm;	s1 *= recipNorm;	s2 *= recipNorm;	s3 *= recipNorm;
//		beta_temp = 0.0075 + 0.025f*dt*constrain_float(Gyro_Length, 0, 500);
//		beta_temp = constrain_float(beta_temp, 0.0075, 0.06f);
//		qDot1 -= beta_temp * s0;	qDot2 -= beta_temp * s1;	qDot3 -= beta_temp * s2;	qDot4 -= beta_temp * s3;
//	}
//
//	q0 += qDot1 * dt;	q1 += qDot2 * dt;	q2 += qDot3 * dt;	q3 += qDot4 * dt;
//	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);	q0 *= recipNorm;	q1 *= recipNorm;	q2 *= recipNorm;	q3 *= recipNorm;
//
//	Pitch = atan2(2.0f * q2 * q3 + 2.0f * q0 * q1, -2.0f *q1 *q1 - 2.0f * q2* q2 + 1.0f) * RAD2DEG;		// Pitch
//	Roll = asin(2.0f * q0* q2 - 2.0f * q1 * q3) * RAD2DEG;			
//	// Roll		
//	//顺时针 YAW 0- 180  逆时针 YAW 0- -180
//
//	if (Yaw_Gyro_Earth_Frame >= 0.07 || Yaw_Gyro_Earth_Frame <= -0.07)  		Yaw += Yaw_Gyro_Earth_Frame*dt;
//	if (Senser_Choice == 1) 		Yaw = T265_data_Yaw;
//	if (Yaw < -180) Yaw = Yaw + 360.0; else if (Yaw > 180) Yaw = Yaw - 360.0; else Yaw = Yaw;
//
//
//	//更新旋转矩阵
//	Sin_Pitch = sin(Pitch* DEG2RAD);  Cos_Pitch = cos(Pitch* DEG2RAD);  Sin_Roll = sin(Roll* DEG2RAD);
//	Cos_Roll = cos(Roll* DEG2RAD);  Sin_Yaw = sin(Yaw* DEG2RAD);  Cos_Yaw = cos(Yaw* DEG2RAD);
//
//	rMat[0][0] = Cos_Yaw* Cos_Roll;
//	rMat[0][1] = Sin_Pitch*Sin_Roll*Cos_Yaw - Cos_Pitch * Sin_Yaw;
//	rMat[0][2] = Sin_Pitch * Sin_Yaw + Cos_Pitch * Sin_Roll * Cos_Yaw;
//
//	rMat[1][0] = Sin_Yaw * Cos_Roll;
//	rMat[1][1] = Sin_Pitch * Sin_Roll * Sin_Yaw + Cos_Pitch * Cos_Yaw;
//	rMat[1][2] = Cos_Pitch * Sin_Roll * Sin_Yaw - Sin_Pitch * Cos_Yaw;
//
//	rMat[2][0] = -Sin_Roll;
//	rMat[2][1] = Sin_Pitch * Cos_Roll;
//	rMat[2][2] = Cos_Pitch * Cos_Roll;
//
//}


/****************** 根据初始化欧拉角初始化四元数 *****************************/


void Quaternion_Init(void)//初始四元数初始化
{
	float euler_roll_init, euler_pitch_init, euler_yaw_init;
	GET_MPU_DATA();
	euler_roll_init = (57.3	*	atan(Y_Origion*invSqrt(X_Origion*X_Origion + Z_Origion*Z_Origion)))*DEG2RAD; //初始化欧拉翻滚角
	euler_pitch_init = (-57.3	*	atan(X_Origion	*invSqrt(Y_Origion*Y_Origion + Z_Origion*Z_Origion)))	*DEG2RAD; //初始化欧拉俯仰角
	euler_yaw_init = 0;

	q0 = cos(euler_yaw_init / 2)*cos(euler_pitch_init / 2)*cos(euler_roll_init / 2) + sin(euler_yaw_init / 2)*sin(euler_pitch_init / 2)*sin(euler_roll_init / 2);
	q1 = cos(euler_yaw_init / 2)*cos(euler_pitch_init / 2)*sin(euler_roll_init / 2) - sin(euler_yaw_init / 2)*sin(euler_pitch_init / 2)*cos(euler_roll_init / 2);
	q2 = cos(euler_yaw_init / 2)*sin(euler_pitch_init / 2)*cos(euler_roll_init / 2) + sin(euler_yaw_init / 2)*cos(euler_pitch_init / 2)*sin(euler_roll_init / 2);
	q3 = sin(euler_yaw_init / 2)*cos(euler_pitch_init / 2)*cos(euler_roll_init / 2) - cos(euler_yaw_init / 2)*sin(euler_pitch_init / 2)*sin(euler_roll_init / 2);
	
	Mq0 = q0;
	Mq1 = q1;
	Mq2 = q2;
	Mq3 = q3;


}

void Vector_From_BodyFrame2EarthFrame(Vector3f *bf,Vector3f *ef)
{
  ef->x=rMat[0][0]*bf->x+rMat[0][1]*bf->y+rMat[0][2]*bf->z;
  ef->y=rMat[1][0]*bf->x+rMat[1][1]*bf->y+rMat[1][2]*bf->z;
  ef->z=rMat[2][0]*bf->x+rMat[2][1]*bf->y+rMat[2][2]*bf->z;
}


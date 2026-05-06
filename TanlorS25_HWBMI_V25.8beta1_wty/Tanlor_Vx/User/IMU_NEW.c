#include "Headfile.h"
#include "IMU_NEW.h"
#include "FFCY_NEW_IMUSensor.h"




lpf2pData accLpf[3];
lpf2pData gyroLpf[3];
lpf2pData gyroFeedBackLPF[3];
sensorData_t sensors, sensors1;


/**
* 设置二阶低通滤波截至频率
*/
void lpf2pSetCutoffFreq(lpf2pData* lpfData, float sample_freq, float cutoff_freq)
{
	float fr = sample_freq / cutoff_freq;
	float ohm = tanf(M_PI_F / fr);
	float c = 1.0f + 2.0f*cosf(M_PI_F / 4.0f)*ohm + ohm*ohm;
	lpfData->b0 = ohm*ohm / c;
	lpfData->b1 = 2.0f*lpfData->b0;
	lpfData->b2 = lpfData->b0;
	lpfData->a1 = 2.0f*(ohm*ohm - 1.0f) / c;
	lpfData->a2 = (1.0f - 2.0f*cosf(M_PI_F / 4.0f)*ohm + ohm*ohm) / c;
	lpfData->delay_element_1 = 0.0f;
	lpfData->delay_element_2 = 0.0f;
}

/**
* 二阶低通滤波
*/
void lpf2pInit(lpf2pData* lpfData, float sample_freq, float cutoff_freq)
{
	if (lpfData == NULL || cutoff_freq <= 0.0f)
	{
		return;
	}

	lpf2pSetCutoffFreq(lpfData, sample_freq, cutoff_freq);
}



float lpf2pApply(lpf2pData* lpfData, float sample)
{
	float delay_element_0 = sample - lpfData->delay_element_1 * lpfData->a1 - lpfData->delay_element_2 * lpfData->a2;
	if (!isfinite(delay_element_0))
	{
		// don't allow bad values to propigate via the filter
		delay_element_0 = sample;
	}

	float output = delay_element_0 * lpfData->b0 + lpfData->delay_element_1 * lpfData->b1 + lpfData->delay_element_2 * lpfData->b2;

	lpfData->delay_element_2 = lpfData->delay_element_1;
	lpfData->delay_element_1 = delay_element_0;
	return output;
}

/*二阶低通滤波*/
void applyAxis3fLpf(lpf2pData *data, Axis3f* in)
{
	for (u8 i = 0; i < 3; i++)
	{
		in->axis[i] = lpf2pApply(&data[i], in->axis[i]);
	}
}

float lpf2pReset(lpf2pData* lpfData, float sample)
{
	float dval = sample / (lpfData->b0 + lpfData->b1 + lpfData->b2);
	lpfData->delay_element_1 = dval;
	lpfData->delay_element_2 = dval;
	return lpf2pApply(lpfData, sample);
}





 float accScale = 1;


 Axis3i16	gyroRaw;
 Axis3i16	accRaw;
 Axis3f  gyroBias;


#define ACCZ_SAMPLE		350

float Kp = 0.1f;		/*比例增益*/
float Ki = 0.0001f;		/*积分增益*/
//float Kp = 0.4f;		/*比例增益*/
//float Ki = 0.001f;		/*积分增益*/
float exInt = 0.0f;
float eyInt = 0.0f;
float ezInt = 0.0f;		/*积分误差累计*/

 float Mq0 = 1.0f;	/*四元数*/
 float Mq1 = 0.0f;
 float Mq2 = 0.0f;
 float Mq3 = 0.0f;
static float MrMat[3][3];/*旋转矩阵*/

static float maxError = 0.f;		/*最大误差*/
u8 isGravityCalibrated = FALSE;	/*是否校校准完成*/
static float baseAcc[3] = { 0.f,0.f,1.0f };	/*静态加速度*/

						
float MinvSqrt(float x)	/*快速开平方求倒*/
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i >> 1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}


/*计算旋转矩阵*/
void imuComputeRotationMatrix(void)
{
	float q1q1 = Mq1 * Mq1;
	float q2q2 = Mq2 * Mq2;
	float q3q3 = Mq3 * Mq3;

	float q0q1 = Mq0 * Mq1;
	float q0q2 = Mq0 * Mq2;
	float q0q3 = Mq0 * Mq3;
	float q1q2 = Mq1 * Mq2;
	float q1q3 = Mq1 * Mq3;
	float q2q3 = Mq2 * Mq3;

	MrMat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
	MrMat[0][1] = 2.0f * (q1q2 + -q0q3);
	MrMat[0][2] = 2.0f * (q1q3 - -q0q2);
	
	MrMat[1][0] = 2.0f * (q1q2 - -q0q3);
	MrMat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
	MrMat[1][2] = 2.0f * (q2q3 + -q0q1);
	
	MrMat[2][0] = 2.0f * (q1q3 + -q0q2);
	MrMat[2][1] = 2.0f * (q2q3 - -q0q1);
	MrMat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}

float MPitch, MRoll, MYaw, MMYaw=0;
float MGyro_Length = 0;//陀螺仪模长
float MYaw_Gyro_Earth_Frame = 0;
extern float  T265_data_Yaw;
extern u8 T265_Confidence;

void imuUpdate(Axis3f acc, Axis3f gyro, float dt)	/*数据融合 互补滤波*/
{
	float normalise;
	float ex, ey, ez;
	float halfT = 0.5f * dt;
	float accBuf[3] = { 0.f };
	Axis3f tempacc = acc;

	gyro.x = gyro.x * DEG2RAD;	/* 度转弧度 */
	gyro.y = gyro.y * DEG2RAD;
	gyro.z = gyro.z * DEG2RAD;

	/* 加速度计输出有效时,利用加速度计补偿陀螺仪*/
	if ((acc.x != 0.0f) || (acc.y != 0.0f) || (acc.z != 0.0f))
	{
		/*单位化加速计测量值*/
		normalise = MinvSqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
		acc.x *= normalise;
		acc.y *= normalise;
		acc.z *= normalise;

		/*加速计读取的方向与重力加速计方向的差值，用向量叉乘计算*/
		ex = (acc.y * MrMat[2][2] - acc.z * MrMat[2][1]);
		ey = (acc.z * MrMat[2][0] - acc.x * MrMat[2][2]);
		ez = (acc.x * MrMat[2][1] - acc.y * MrMat[2][0]);

		/*误差累计，与积分常数相乘*/
		exInt += Ki * ex * dt;
		eyInt += Ki * ey * dt;
		ezInt += Ki * ez * dt;

		/*用叉积误差来做PI修正陀螺零偏，即抵消陀螺读数中的偏移量*/
		gyro.x += Kp * ex + exInt;
		gyro.y += Kp * ey + eyInt;
		gyro.z += Kp * ez + ezInt;
	}
	/* 一阶近似算法，四元数运动学方程的离散化形式和积分 */
	float q0Last = Mq0;
	float q1Last = Mq1;
	float q2Last = Mq2;
	float q3Last = Mq3;
	Mq0 += (-q1Last * gyro.x - q2Last * gyro.y - q3Last * gyro.z) * halfT;
	Mq1 += (q0Last * gyro.x + q2Last * gyro.z - q3Last * gyro.y) * halfT;
	Mq2 += (q0Last * gyro.y - q1Last * gyro.z + q3Last * gyro.x) * halfT;
	Mq3 += (q0Last * gyro.z + q1Last * gyro.y - q2Last * gyro.x) * halfT;

	/*单位化四元数*/
	normalise = MinvSqrt(Mq0 * Mq0 + Mq1 * Mq1 + Mq2 * Mq2 + Mq3 * Mq3);
	Mq0 *= normalise;
	Mq1 *= normalise;
	Mq2 *= normalise;
	Mq3 *= normalise;

	imuComputeRotationMatrix();	/*计算旋转矩阵*/

	/*计算roll pitch yaw 欧拉角*/
	MPitch = atan2f(MrMat[2][1], MrMat[2][2]) * RAD2DEG;
	MRoll = -asinf(MrMat[2][0]) * RAD2DEG;
	MYaw = atan2f(MrMat[1][0], MrMat[0][0]) * RAD2DEG;


	MSin_Pitch = sin(MPitch* DEG2RAD);  MCos_Pitch = cos(MPitch* DEG2RAD);  MSin_Roll = sin(MRoll* DEG2RAD);
	MCos_Roll = cos(MRoll* DEG2RAD);  MSin_Yaw = sin(MYaw* DEG2RAD);  MCos_Yaw = cos(MYaw* DEG2RAD);

	MGyro_Length = sqrt(MYaw_Gyro*MYaw_Gyro + MPitch_Gyro*MPitch_Gyro + MRoll_Gyro*MRoll_Gyro);//单位deg/s
	MYaw_Gyro_Earth_Frame = -MSin_Roll*gyro.x*RAD2DEG + MCos_Roll*MSin_Pitch *gyro.y*RAD2DEG + MCos_Pitch * MCos_Roll *gyro.z*RAD2DEG;    //{-sinθ  cosθsin Φ  cosθcosΦ  }

	if (MYaw_Gyro_Earth_Frame >= 0.3f || MYaw_Gyro_Earth_Frame <= -0.3f)  
		MMYaw += MYaw_Gyro_Earth_Frame*dt;

	if (MMYaw < -180) MMYaw = MMYaw + 360.0; else if (MMYaw > 180) MMYaw = MMYaw - 360.0; else MMYaw = MMYaw;


	Pitch = MPitch;
	Roll = MRoll;

	if (T265_Confidence == 3)
	{
		Yaw = T265_data_Yaw;
	}
	else
	{
		Yaw = MMYaw;
	}


	

	Pitch_Gyro = MPitch_Gyro;
	Roll_Gyro = MRoll_Gyro;
	Yaw_Gyro = MYaw_Gyro;
	Gyro_Length = MGyro_Length;

	Yaw_Gyro_Earth_Frame = Yaw_Gyro_Earth_Frame;

}

void NEW_Vector_From_BodyFrame2EarthFrame(Vector3f *bf, Vector3f *ef)
{
	ef->x = MrMat[0][0] * bf->x + MrMat[0][1] * bf->y + MrMat[0][2] * bf->z;
	ef->y = MrMat[1][0] * bf->x + MrMat[1][1] * bf->y + MrMat[1][2] * bf->z;
	ef->z = MrMat[2][0] * bf->x + MrMat[2][1] * bf->y + MrMat[2][2] * bf->z;
}

/*机体到地球*/
void imuTransformVectorBodyToEarth(Axis3f * v)
{
	/* From body frame to earth frame */
	const float x = MrMat[0][0] * v->x + MrMat[0][1] * v->y + MrMat[0][2] * v->z;
	const float y = MrMat[1][0] * v->x + MrMat[1][1] * v->y + MrMat[1][2] * v->z;
	const float z = MrMat[2][0] * v->x + MrMat[2][1] * v->y + MrMat[2][2] * v->z;

	float yawRad = atan2f(MrMat[1][0], MrMat[0][0]);
	float cosy = cosf(yawRad);
	float siny = sinf(yawRad);
	float vx = x * cosy + y * siny;
	float vy = y * cosy - x * siny;

	v->x = vx;
	v->y = -vy;
	v->z = z - baseAcc[2] * 980.f;	/*去除重力加速度*/
}


/*地球到机体*/
void imuTransformVectorEarthToBody(Axis3f * v)
{
	v->y = -v->y;

	/* From earth frame to body frame */
	const float x = MrMat[0][0] * v->x + MrMat[1][0] * v->y + MrMat[2][0] * v->z;
	const float y = MrMat[0][1] * v->x + MrMat[1][1] * v->y + MrMat[2][1] * v->z;
	const float z = MrMat[0][2] * v->x + MrMat[1][2] * v->y + MrMat[2][2] * v->z;

	v->x = x;
	v->y = y;
	v->z = z;
}


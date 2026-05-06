#ifndef __IMU_NEW_H
#define __IMU_NEW_H


extern float Mq0 ;	/*四元数*/
extern float Mq1 ;
extern float Mq2 ;
extern float Mq3 ;

typedef union
{
	struct
	{
		float x;
		float y;
		float z;
	};
	float axis[3];
} Axis3f;

typedef struct
{
	float a1;
	float a2;
	float b0;
	float b1;
	float b2;
	float delay_element_1;
	float delay_element_2;
} lpf2pData;



typedef struct
{
	Axis3f acc;
	Axis3f gyro;

} sensorData_t;

typedef union
{
	struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
	};
	int16_t axis[3];
} Axis3i16;


extern lpf2pData accLpf[3];
extern lpf2pData gyroLpf[3];
extern lpf2pData gyroFeedBackLPF[3];
extern Axis3i16	gyroRaw;
extern Axis3i16	accRaw;
extern Axis3f  gyroBias;

void lpf2pInit(lpf2pData* lpfData, float sample_freq, float cutoff_freq);

extern sensorData_t sensors, sensors1;
extern Axis3f Gyro_feedback;
void applyAxis3fLpf(lpf2pData *data, Axis3f* in);
void imuUpdate(Axis3f acc, Axis3f gyro, float dt);	/*数据融合 互补滤波*/
void NEW_Vector_From_BodyFrame2EarthFrame(Vector3f *bf, Vector3f *ef);
#endif

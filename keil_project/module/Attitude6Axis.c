#include "Attitude6Axis.h"

#include <math.h>

#define ATT6_DEG2RAD (0.01745329251994329577f)  // PI / 180.0f
#define ATT6_RAD2DEG (57.295779513082320876f)   // 180.0f / PI

static float att6InvSqrt(float x)  
{
	if (x <= 0.0f)
	{
		return 0.0f;
	}
	return 1.0f / sqrtf(x);
}

static float att6Clamp(float value, float minValue, float maxValue) 
{
	if (value < minValue)
	{
		return minValue;
	}
	if (value > maxValue)
	{
		return maxValue;
	}
	return value;
}

void Attitude6Axis_Reset(Attitude6AxisState *state)
{
	if (state == 0)
	{
		return;
	}

	state->q0 = 1.0f;
	state->q1 = 0.0f;
	state->q2 = 0.0f;
	state->q3 = 0.0f;
	state->exInt = 0.0f;
	state->eyInt = 0.0f;
	state->ezInt = 0.0f;
	state->pitchDeg = 0.0f;
	state->rollDeg = 0.0f;
	state->yawDeg = 0.0f;
}

void Attitude6Axis_Init(Attitude6AxisState *state, float kp, float ki)
{
	if (state == 0)
	{
		return;
	}

	Attitude6Axis_Reset(state);
	state->kp = kp;
	state->ki = ki;
}

void Attitude6Axis_UpdateRaw(
	Attitude6AxisState *state,
	int16_t ax,
	int16_t ay,
	int16_t az,
	int16_t gx,
	int16_t gy,
	int16_t gz,
	float accGPerLsb,
	float gyroDpsPerLsb,
	float dtSec)
{
	float q0;
	float q1;
	float q2;
	float q3;
	float norm;
	float ex;
	float ey;
	float ez;
	float vx;
	float vy;
	float vz;
	float gxRad;
	float gyRad;
	float gzRad;
	float q0Last;
	float q1Last;
	float q2Last;
	float q3Last;
	float halfT;
	float sinp;

	if (state == 0)
	{
		return;
	}

	if (dtSec <= 0.0f)
	{
		return;
	}

	q0 = state->q0;
	q1 = state->q1;
	q2 = state->q2;
	q3 = state->q3;

	gxRad = (float)gx * gyroDpsPerLsb * ATT6_DEG2RAD;
	gyRad = (float)gy * gyroDpsPerLsb * ATT6_DEG2RAD;
	gzRad = (float)gz * gyroDpsPerLsb * ATT6_DEG2RAD;

	norm = att6InvSqrt(((float)ax * accGPerLsb) * ((float)ax * accGPerLsb)
		+ ((float)ay * accGPerLsb) * ((float)ay * accGPerLsb)
		+ ((float)az * accGPerLsb) * ((float)az * accGPerLsb));

	if (norm > 0.0f)
	{
		float axN = (float)ax * accGPerLsb * norm;
		float ayN = (float)ay * accGPerLsb * norm;
		float azN = (float)az * accGPerLsb * norm;

		vx = 2.0f * (q1 * q3 - q0 * q2);
		vy = 2.0f * (q0 * q1 + q2 * q3);
		vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

		ex = ayN * vz - azN * vy;
		ey = azN * vx - axN * vz;
		ez = axN * vy - ayN * vx;

		state->exInt += state->ki * ex * dtSec;
		state->eyInt += state->ki * ey * dtSec;
		state->ezInt += state->ki * ez * dtSec;

		gxRad += state->kp * ex + state->exInt;
		gyRad += state->kp * ey + state->eyInt;
		gzRad += state->kp * ez + state->ezInt;
	}

	halfT = 0.5f * dtSec;
	q0Last = q0;
	q1Last = q1;
	q2Last = q2;
	q3Last = q3;

	q0 += (-q1Last * gxRad - q2Last * gyRad - q3Last * gzRad) * halfT;
	q1 += (q0Last * gxRad + q2Last * gzRad - q3Last * gyRad) * halfT;
	q2 += (q0Last * gyRad - q1Last * gzRad + q3Last * gxRad) * halfT;
	q3 += (q0Last * gzRad + q1Last * gyRad - q2Last * gxRad) * halfT;

	norm = att6InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	if (norm > 0.0f)
	{
		q0 *= norm;
		q1 *= norm;
		q2 *= norm;
		q3 *= norm;
	}

	state->q0 = q0;
	state->q1 = q1;
	state->q2 = q2;
	state->q3 = q3;

	state->rollDeg = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * ATT6_RAD2DEG;
	sinp = 2.0f * (q0 * q2 - q3 * q1);
	sinp = att6Clamp(sinp, -1.0f, 1.0f);
	state->pitchDeg = asinf(sinp) * ATT6_RAD2DEG;
	state->yawDeg = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * ATT6_RAD2DEG;
}

void Attitude6Axis_GetEulerDeg(
	const Attitude6AxisState *state,
	float *pitchDeg,
	float *rollDeg,
	float *yawDeg)
{
	if (state == 0)
	{
		return;
	}

	if (pitchDeg != 0)
	{
		*pitchDeg = state->pitchDeg;
	}

	if (rollDeg != 0)
	{
		*rollDeg = state->rollDeg;
	}

	if (yawDeg != 0)
	{
		*yawDeg = state->yawDeg;
	}
}



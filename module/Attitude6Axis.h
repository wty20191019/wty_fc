#ifndef __ATTITUDE_6AXIS_H
#define __ATTITUDE_6AXIS_H

#include <stdint.h>

typedef struct
{
	float q0;
	float q1;
	float q2;
	float q3;
	float exInt;
	float eyInt;
	float ezInt;
	float kp;
	float ki;
	float pitchDeg;
	float rollDeg;
	float yawDeg;
} Attitude6AxisState;

void Attitude6Axis_Init(Attitude6AxisState *state, float kp, float ki);
void Attitude6Axis_Reset(Attitude6AxisState *state);

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
	float dtSec);

void Attitude6Axis_GetEulerDeg(
	const Attitude6AxisState *state,
	float *pitchDeg,
	float *rollDeg,
	float *yawDeg);

#endif




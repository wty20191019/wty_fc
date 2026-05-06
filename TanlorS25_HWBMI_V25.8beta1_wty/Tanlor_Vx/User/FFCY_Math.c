#include "Headfile.h"
#include "FFCY_Math.h"



//**************************************
//µÝÍÆ¾ùÖµÂË²¨
//**************************************

float GildeAverageValueFilter(float NewValue, float *Data)
{
	float max, min;
	float sum;
	unsigned char i;
	Data[0] = NewValue;
	max = Data[0];
	min = Data[0];
	sum = Data[0];
	for (i = GildeAverageValueFilter_N - 1; i != 0; i--)
	{
		if (Data[i]>max) max = Data[i];
		else if (Data[i]<min) min = Data[i];
		sum += Data[i];
		Data[i] = Data[i - 1];
	}
	i = GildeAverageValueFilter_N - 2;
	sum = sum - max - min;
	sum = sum / i;
	return(sum);
}


float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;

	i = 0x5f3759df - (i >> 1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));

	return y;
}

float constrain_float(float amt, float low, float high) {
	return ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)));
}

int16_t constrain_int16_t(int16_t amt, int16_t low, int16_t high) {
	return ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)));
}

// square
float sq(float v) { return v*v; }
// 2D vector length
float pythagorous2(float a, float b) { return sqrtf(sq(a) + sq(b)); }

#ifndef _FFCY_OPTICALFLOW_
#define _FFCY_OPTICALFLOW_


typedef struct
{
	float out;
	float a;
	float b;
	float e_nr;

}_filter_1_st;



extern SINS OpticalFlow_SINS;
extern Vector2f OpticalFlow_Speed, OpticalFlow_Position;
void OpticalFlow_Init(void);
void Optflow_Statemachine(void);
void PMW3901_Data_Process(void);

#endif




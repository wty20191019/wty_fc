/*----------------------------------------------------------------------------------------------------------------------/
*               本程序只供购买者学习使用，版权著作权属于飞凡创翼团队，飞凡创翼团队将飞控程序源码提供给购买者，
*               购买者要为飞凡创翼团队提供保护，未经作者许可，不得将源代码提供给他人，不得将源代码放到网上供他人免费下载，
*               更不能以此销售牟利，如发现上述行为，飞凡创翼团队将诉之以法律解决！！！
-----------------------------------------------------------------------------------------------------------------------/
*               修改日期:2023/5/6
*               版本：TanlorS V1.0
*               版权所有，盗版必究。
*               Copyright(C) 中国民航大学飞凡创翼团队 2017-2025
*               All rights reserved
----------------------------------------------------------------------------------------------------------------------*/
#include "Headfile.h"
#include "RC.H"

#define CH1 0
#define CH2 1
#define CH3 2
#define CH4 3

uint16_t RC_Buttom=1000,RC_Top=2000,RC_Middle=1500,RC_Deadband=100;
uint16_t RC_Deadzone_Buttom=0,RC_Deadzone_Top=0;
u8 Data_buf[32]={0,0};
uint16 Throttle_Control = 0;
uint16 Throttle_Control_buff = 0;
int16 Pitch_Control = 0, Roll_Control = 0, Yaw_Control = 0;
int16 Target_Angle[2] = { 0 };
uint8 RC_Control[32];
uint16 Last_Throttle_Control, Pre_Last_Throttle_Control;
uint16 TempThrottle_Control;
uint16 QuadData[2];
uint8 QuadRemoteFlag[4] = { 0 };
int16 RC_NewData[4] = { 0 };
int16 RC3_Origal_Value = 0; //第三通道原始数据
uint16 RC_Init_Cnt = 0;
uint16_t Controler_State = 0;
int16 Y_T = 0, T_temp = 0;
int16 Temp_RC = 0;
uint16_t Unlock_Makesure_Cnt = 0, Lock_Makesure_Cnt = 0;
int16_t Throttle_Rate = 0;
Butter_BufferData RC_LPF_Buffer[4];
uint16_t PPM_LPF_Databuf[4];


int16_t Get_Thr_Rate(float Thr)//油门变化量
{
  static float Thr_Rec[20];
  uint16_t i=0;
  for(i=19;i>0;i--)
  {
    Thr_Rec[i]=Thr_Rec[i-1];
  }
  Thr_Rec[0]=Thr;
  return (int16_t)((Thr_Rec[0]-Thr_Rec[9])/1.0f);
}


/************************************************************************/
/* FFCY 孙艺东                                                          */
/* 函数功能:  RC数据比例变换                                            */
/************************************************************************/
#define  Buttom_Safe_Deadband  350 //中值下350
int16_t Throttle_Scale_Transformation(u8 CH, Vector_RC *RC_Calibration_Date)
{
	RC_Middle = AT10_MID;// (uint16_t)(RC_Calibration_Date->middle);
	RC3_Origal_Value = constrain_int16_t((PPM_Databuf[CH3] - (RC_Middle- Buttom_Safe_Deadband-50)) , 0, 1000);
	return constrain_int16_t((PPM_Databuf[CH3] - (RC_Middle - Buttom_Safe_Deadband - 60)), 0, 1000);
}

/************************************************************************/
/* FFCY 孙艺东                                                          */
/* 函数功能:  RC数据比例变换                                            */
/************************************************************************/

// Range+Deadband 400
#define Deadband 40
#define Range 360 

int16_t RC_Scale_Transformation(u8 CH,u16 Max, Vector_RC *RC_Calibration_Date)
{
	int16 Control_Value = 0;

	RC_Middle = AT10_MID; //(uint16_t)(RC_Calibration_Date->middle);

	if (PPM_Databuf[CH] <= RC_Middle - Deadband)
		Control_Value = (RC_Middle - Deadband - PPM_Databuf[CH])*Max / Range;
	else if (PPM_Databuf[CH] >= RC_Middle + Deadband)
		Control_Value = (RC_Middle + Deadband - PPM_Databuf[CH])*Max / Range;
	else    Control_Value = 0;

	return constrain_int16_t(Control_Value, -Max, Max);
}



void RC_Statemachine(void)
{
  /****************RC比例变换*****************/
	Roll_Control=RC_Scale_Transformation(CH1, Pit_Rol_Max, &RC_Calibration[CH1]);
	Pitch_Control = RC_Scale_Transformation(CH2, Pit_Rol_Max, &RC_Calibration[CH2]);
	Yaw_Control = RC_Scale_Transformation(CH4, Yaw_Max, &RC_Calibration[CH4]);
	Throttle_Control_buff= Throttle_Scale_Transformation(CH3,&RC_Calibration[CH3]);


	
	Throttle_Rate=Get_Thr_Rate(Throttle_Control_buff);
	Throttle_Control=1000+ Throttle_Control_buff;
	RC_NewData[0] = Throttle_Control_buff;//遥感油门原始行程量

	Target_Angle[0] = -Pitch_Control;//自稳时，期望俯仰角
	Target_Angle[1] = -Roll_Control;//自稳时，期望横滚角

	//开始的50ms不处理数据
  if (RC_Init_Cnt <= 10)
  {
	  Throttle_Control = 1000;
	  Pitch_Control = 0;
	  Roll_Control = 0;
	  Yaw_Control = 0;
	  RC_Init_Cnt++;
  }


}

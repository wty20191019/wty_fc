#include "Headfile.h"
#include "PID.h"



AllControler Total_Controller;//系统总控制器


float PID_Control(PID_Controler *Controler,float dt)
{
	float controller_dt = 0.005f;
	//Test_Period(&Controler->PID_Controller_Dt);
	//controller_dt = Controler->PID_Controller_Dt.Time_Delta / 1000.0;
	//if (controller_dt < 0.001) return 0;
   Controler->PID_Controller_Dt.Time_Delta= controller_dt * 1000.0f;
  /*******偏差计算*********************/
  Controler->Last_Err=Controler->Err;//保存上次偏差
  Controler->Err=Controler->Expect-Controler->FeedBack;//期望减去反馈得到偏差
  if(Controler->Err_Limit_Flag==1)//偏差限幅度标志位
  {
    if(Controler->Err>=Controler->Err_Max)   Controler->Err= Controler->Err_Max;
    if(Controler->Err<=-Controler->Err_Max)  Controler->Err=-Controler->Err_Max;
  }
  /*******积分计算*********************/
  if(Controler->Integrate_Separation_Flag==1)//积分分离标志位
  {
    if(ABS(Controler->Err)<=Controler->Integrate_Separation_Err)
      Controler->Integrate+=Controler->Scale_Ki*Controler->Ki*Controler->Err*controller_dt;
  }
  else
  {
    Controler->Integrate+=Controler->Scale_Ki*Controler->Ki*Controler->Err*controller_dt;
  }
  /*******积分限幅*********************/
  if(Controler->Integrate_Limit_Flag==1)//积分限制幅度标志
  {
    if(Controler->Integrate>=Controler->Integrate_Max)
      Controler->Integrate=Controler->Integrate_Max;
    if(Controler->Integrate<=-Controler->Integrate_Max)
      Controler->Integrate=-Controler->Integrate_Max ;
  }
  /*******总输出计算*********************/
  Controler->Last_Control_OutPut=Controler->Control_OutPut;//输出值递推
  Controler->Control_OutPut=Controler->Scale_Kp*Controler->Kp*Controler->Err//比例
    +Controler->Integrate//积分
      +Controler->Kd*(Controler->Err-Controler->Last_Err);//微分
  /*******总输出限幅*********************/
  if(Controler->Control_OutPut>=Controler->Control_OutPut_Limit)
    Controler->Control_OutPut=Controler->Control_OutPut_Limit;
  if(Controler->Control_OutPut<=-Controler->Control_OutPut_Limit)
    Controler->Control_OutPut=-Controler->Control_OutPut_Limit;
  /*******返回总输出*********************/
  return Controler->Control_OutPut;
}

float PID_Control_Yaw(PID_Controler *Controler)
{
  float controller_dt=0;
  Test_Period(&Controler->PID_Controller_Dt);
  controller_dt=Controler->PID_Controller_Dt.Time_Delta/1000.0;
  if(controller_dt<0.001) return 0;
  /*******偏差计算*********************/
  Controler->Last_Err=Controler->Err;//保存上次偏差
  Controler->Err=Controler->Expect-Controler->FeedBack;//期望减去反馈得到偏差
  /***********************偏航角偏差超过+-180处理*****************************/
  if(Controler->Err<-180)  Controler->Err=Controler->Err+360;
  if(Controler->Err>180)  Controler->Err=Controler->Err-360;
  
  if(Controler->Err_Limit_Flag==1)//偏差限幅度标志位
  {
    if(Controler->Err>=Controler->Err_Max)   Controler->Err= Controler->Err_Max;
    if(Controler->Err<=-Controler->Err_Max)  Controler->Err=-Controler->Err_Max;
  }
  /*******积分计算*********************/
  if(Controler->Integrate_Separation_Flag==1)//积分分离标志位
  {
    if(ABS(Controler->Err)<=Controler->Integrate_Separation_Err)
      Controler->Integrate+=Controler->Scale_Ki*Controler->Ki*Controler->Err*controller_dt;
  }
  else
  {
    Controler->Integrate+=Controler->Scale_Ki*Controler->Ki*Controler->Err*controller_dt;
  }
  /*******积分限幅*********************/
  if(Controler->Integrate_Limit_Flag==1)//积分限制幅度标志
  {
    if(Controler->Integrate>=Controler->Integrate_Max)
      Controler->Integrate=Controler->Integrate_Max;
    if(Controler->Integrate<=-Controler->Integrate_Max)
      Controler->Integrate=-Controler->Integrate_Max ;
  }
  /*******总输出计算*********************/
  Controler->Last_Control_OutPut=Controler->Control_OutPut;//输出值递推
  Controler->Control_OutPut=Controler->Scale_Kp*Controler->Kp*Controler->Err//比例
    +Controler->Integrate//积分
      +Controler->Kd*(Controler->Err-Controler->Last_Err);//微分
  /*******总输出限幅*********************/
  if(Controler->Control_OutPut>=Controler->Control_OutPut_Limit)
    Controler->Control_OutPut=Controler->Control_OutPut_Limit;
  if(Controler->Control_OutPut<=-Controler->Control_OutPut_Limit)
    Controler->Control_OutPut=-Controler->Control_OutPut_Limit;
  /*******返回总输出*********************/
  return Controler->Control_OutPut;
}



void  PID_Integrate_Reset(PID_Controler *Controler)  {Controler->Integrate=0.0f;}

void Take_Off_Reset(void)
{
  PID_Integrate_Reset(&Total_Controller.Roll_Gyro_Control);//起飞前屏蔽积分
  PID_Integrate_Reset(&Total_Controller.Pitch_Gyro_Control);
  PID_Integrate_Reset(&Total_Controller.Yaw_Gyro_Control);
  PID_Integrate_Reset(&Total_Controller.Pitch_Angle_Control);
  PID_Integrate_Reset(&Total_Controller.Roll_Angle_Control);
  PID_Integrate_Reset(&Total_Controller.Yaw_Angle_Control);
  
  PID_Integrate_Reset(&Total_Controller.Longitude_Speed_Control);//位置控制速度环积分项
  PID_Integrate_Reset(&Total_Controller.Latitude_Speed_Control);
  PID_Integrate_Reset(&Total_Controller.High_Speed_Control);
  OpticalFlow_Ctrl_Reset();
}

void Throttle_Control_Reset(void)
{
  PID_Integrate_Reset(&Total_Controller.High_Acce_Control);
  PID_Integrate_Reset(&Total_Controller.High_Speed_Control);
  PID_Integrate_Reset(&Total_Controller.High_Position_Control);
}


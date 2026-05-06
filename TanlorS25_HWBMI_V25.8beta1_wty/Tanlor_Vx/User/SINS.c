#include "Headfile.h"
#include "SINS.h"
#include "Earth_Declination.h"
SINS NamelessQuad;
SINS Origion_NamelessQuad;
SINS PosSenser_SINS;

//float Sin_Pitch=0,Sin_Roll=0,Sin_Yaw=0;
//float Cos_Pitch=0,Cos_Roll=0,Cos_Yaw=0;
float MSin_Pitch = 0, MSin_Roll = 0, MSin_Yaw = 0;
float MCos_Pitch = 0, MCos_Roll = 0, MCos_Yaw = 0;

float Acceleration_Length=0;
Vector2f SINS_Accel_Body={0,0};
Vector3f Body_Frame,MBody_Frame, Earth_Frame,MEarth_Frame;

void  SINS_Prepare(void)
{
 
  Vector2f SINS_Accel_Earth={0,0};

  //旋转到地理坐标系
  //Vector_From_BodyFrame2EarthFrame(&Body_Frame,&Earth_Frame);
  NEW_Vector_From_BodyFrame2EarthFrame(&Body_Frame, &MEarth_Frame);
  //单位变换   加速度cm/s^2
  Origion_NamelessQuad.Accel.z=((MEarth_Frame.z*(GRAVITY_MSS / AcceMax_1G))- GRAVITY_MSS)*100;
  Origion_NamelessQuad.Accel.y=MEarth_Frame.x*(GRAVITY_MSS / AcceMax_1G)*100;
  Origion_NamelessQuad.Accel.x=MEarth_Frame.y*(GRAVITY_MSS / AcceMax_1G)*100;
  
  Acceleration_Length=sqrt(Origion_NamelessQuad.Accel.z*Origion_NamelessQuad.Accel.z
                           +Origion_NamelessQuad.Accel.y*Origion_NamelessQuad.Accel.y
                             +Origion_NamelessQuad.Accel.x*Origion_NamelessQuad.Accel.x);
  
  /******************************************************************************/
  //将无人机在导航坐标系下的沿着正东、正北方向的运动加速度旋转到当前航向的运动加速度:机头(俯仰)+横滚
  SINS_Accel_Earth.x=Origion_NamelessQuad.Accel.y;//沿地理坐标系，正东方向运动加速度,单位为CM
  SINS_Accel_Earth.y=Origion_NamelessQuad.Accel.x;//沿地理坐标系，正北方向运动加速度,单位为CM
  
  SINS_Accel_Body.x=SINS_Accel_Earth.x*MCos_Yaw+SINS_Accel_Earth.y*MSin_Yaw;  //横滚正向运动加速度  X轴正向
  SINS_Accel_Body.y=-SINS_Accel_Earth.x*MSin_Yaw+SINS_Accel_Earth.y*MCos_Yaw; //机头正向运动加速度  Y轴正向

}

uint16 High_Delay_Cnt = 0;
void Observation_Tradeoff(uint8_t HC_SR04_Enable)
{
	Altitude_Estimate = VL53Lx_Data.Distance*MCos_Roll*MCos_Pitch / 10.0f;
	High_Delay_Cnt = 2;
}



float Altitude_Estimate=0;
float Kalman_R[2]={5.0e-4f,6.0e-4f};
float Kalman_Q=30;//50

float Acce_Bias_Gain[3]={
  0.0005,//0.001
  0.0005,//0.001
  0.0005,//0.001
};
float Pre_conv[4]=
{
  0.18,0.1,//0.001,0,
  0.1,0.18//0,0.001
};//上一次协方差


float New_Yaw_speed = 0.0;
float New_Yaw_Posion = 0.0;
float My_desiner_Posion_err = 0;

//位置观测量//观测传感器延时量//惯导结构体//系统原始驱动量，惯导加速度
void  KalmanFilter(float Observation, uint16 Pos_Delay_Cnt, SINS *Ins_Kf, float System_drive,  float *R, float Q,float dt)
{
  uint16 Cnt=0;
  static uint16 Speed_Sync_Cnt=0;
  float Temp_conv[4]={0};//先验协方差
  float Conv_Z=0,Z_Cor=0;
  float k[2]={0};//增益矩阵
  float Ctemp=0;

	//先验状态
	Ins_Kf->Accel.z = System_drive;
	Ins_Kf->Accel.z = Ins_Kf->Accel_Bias.z + Ins_Kf->Accel.z;
	Ins_Kf->Location.z += Ins_Kf->Velocity.z * dt + (Ins_Kf->Accel.z*dt*dt) / 2.0;
	Ins_Kf->Velocity.z += Ins_Kf->Accel.z*dt;

    //先验协方差
	Ctemp = Pre_conv[1] + Pre_conv[3] * dt;
	Temp_conv[0] = Pre_conv[0] + Pre_conv[2] * dt + Ctemp*dt + R[0];
	Temp_conv[1] = Ctemp;
	Temp_conv[2] = Pre_conv[2] + Pre_conv[3] * dt;
	Temp_conv[3] = Pre_conv[3] + R[1];

    //计算卡尔曼增益
	Conv_Z = Temp_conv[0] + Q;
	k[0] = Temp_conv[0] / Conv_Z;
	k[1] = Temp_conv[2] / Conv_Z;

    //融合数据输出
	Z_Cor = Observation - Ins_Kf->Positon_History.z[Pos_Delay_Cnt];
    //Z_Cor=Positional-*Position;
	Ins_Kf->Location.z += k[0] * Z_Cor;
	Ins_Kf->Velocity.z += k[1] * Z_Cor;
	Ins_Kf->Accel_Bias.z += Acce_Bias_Gain[0] * Z_Cor;

    //更新状态协方差矩阵
	Pre_conv[0] = (1 - k[0])*Temp_conv[0];
	Pre_conv[1] = (1 - k[0])*Temp_conv[1];
	Pre_conv[2] = Temp_conv[2] - k[1] * Temp_conv[0];
	Pre_conv[3] = Temp_conv[3] - k[1] * Temp_conv[1];;
    
	for (Cnt = Num - 1; Cnt > 0; Cnt--)//5ms滑动一次
   {
	   Ins_Kf->Positon_History.z[Cnt] = Ins_Kf->Positon_History.z[Cnt - 1];
   }
   //Ins_Kf->Pos_History[N][0]=Ins_Kf->Position[N];

	New_Yaw_Posion = Ins_Kf->Location.z + 5.0f * My_desiner_Posion_err;
	My_desiner_Posion_err += (Observation - New_Yaw_Posion) * dt;

	New_Yaw_speed = (Ins_Kf->Location.z - Ins_Kf->Positon_History.z[0]) / dt;
	Ins_Kf->Positon_History.z[0] = Ins_Kf->Location.z;

	Speed_Sync_Cnt++;
  if (Speed_Sync_Cnt >= 20)//100ms滑动一次
  {
	  for (Cnt = Num - 1; Cnt > 0; Cnt--)
	  {
		  Ins_Kf->Velocity_History.z[Cnt] = Ins_Kf->Velocity_History.z[Cnt - 1];
	  }
	  Speed_Sync_Cnt = 0;
  }
  Ins_Kf->Velocity_History.z[0] = Ins_Kf->Velocity.z;
}

void Strapdown_INS_High_Kalman(void)
{

  Observation_Tradeoff(1);
  KalmanFilter(Altitude_Estimate,//位置观测量
               High_Delay_Cnt,//观测传感器延时量
               &NamelessQuad,//惯导结构体
               Origion_NamelessQuad.Accel.z,//系统原始驱动量，惯导加速度
				Kalman_R,
				Kalman_Q,
               0.005f);
}

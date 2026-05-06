#include "Headfile.h"
#include "Total_Control.h"
#include "control_config.h"


u8 Simu_T265;
u8 T265_Confidence_Last = 3;
u8 Simu_T265_Wrong_Flag = 1;
u8 T265_Wrong_Stage = 0;
float Position_Offset_X = 0.0;
float Position_Offset_Y = 0.0;
float T265_Record_X = 0.0;
float T265_Record_Y = 0.0;
extern float Altitude_Speed;
extern u8 Time_Sys_Record;
extern u8 T265_Confidence;
extern u8 T265_Start_Flag;
u8 T265_Start = 0;
u16 T265_Start_Cnt = 0;
extern float New_Yaw_speed;
extern Vector2f OpticalFlow_Speed;
extern Vector2f OpticalFlow_Position_fix;
extern float New_Yaw_Posion;
extern float Altitude_Position;

void Simu_T265_Wrong(u8 flag)
{
	if (flag)
	{
		if (Simu_T265 > 0)
		{
			T265_Confidence = 0;
			Time_Sys_Record = Time_Sys[Second];
		}
	}
}

float UWB_X = 0;
float UWB_Y = 0;
float UWB_Z = 0;
float UWB_X_Filter = 0;
float UWB_Y_Filter = 0;
float UWB_Z_Filter = 0;
u16 UWB_X_Filter_Pre = 0;
u16 UWB_Y_Filter_Pre = 0;
float UWB_X_Filter_Pre_All = 0;
float UWB_Y_Filter_Pre_All = 0;
u16 UWB_X_Filter_Pre_Ava = 0;
u16 UWB_Y_Filter_Pre_Ava = 0;
u32 UWB_Ava_Cnt;
Butter_BufferData  UWB_filter_buf[3];
extern Butter_Parameter Gyro_Parameter;
u8 UWB_Position[10];
u8 Targ_UWB_Flag = 0;
u16 ldca1 = 1234;
u16 ldca2 = 12345;
u16 ldca3 = 123;
u16 Position_CND = 0;
float UWB_posyresult_0_Last = 0;
float UWB_posxresult_0_Last = 0;
float UWB_posxresult_0 = 0;
float UWB_posyresult_0 = 0;
u8 Gezi_x = 0;
u8 Gezi_x_Lst = 0;
u8 Gezi_y = 0;
u8 Gezi_y_Lst = 0;
u8 Gezi_x_Rec = 0;
u8 Gezi_y_Rec = 0;
u8 U6_Send_Buff[10] = { 0 };
u8 Of_Begin_Flag = 0;
u8 Gezi_CF = 0;
extern u8 tiaoshi;
extern Vector3f SDK_Zero_POSITION;
extern float radar_speed_x, radar_speed_y;
extern float fx_gyro_fix_rd, fy_gyro_fix_rd;

void Location_Get(void)
{

	//if (VL53Lx_Data.Is_OK)
	//{
	//	Altitude_Speed = New_Yaw_speed;
	//	Altitude_Position = New_Yaw_Posion;
	//}
	//else
	//{
	//	Altitude_Speed = ps_data.velocity.z;
	//	Altitude_Position = ps_data.translation.z;
	//}

	Altitude_Speed = New_Yaw_speed;
	Altitude_Position = New_Yaw_Posion;
	PosSenser_SINS.Location.y = ps_data.translation.x;
	PosSenser_SINS.Location.x = ps_data.translation.y;

	if (T265_Confidence == 3)
	{
		PosSenser_SINS.Velocity.y = ps_data.velocity.x;
		PosSenser_SINS.Velocity.x = ps_data.velocity.y;
	}
	else
	{
		PosSenser_SINS.Velocity.y = radar_speed_y;
		PosSenser_SINS.Velocity.x = radar_speed_x;

	}

	//if (ABS(ld_data.translation.x + 200) > 2000 || ABS(ld_data.translation.x - 150) > 2000)
	//{
	//	PosSenser_SINS.Location.y = ps_data.translation.x;
	//	PosSenser_SINS.Location.x = ps_data.translation.y;
	//	tiaoshi = 1;
	//}
	//else
	//{
	//	tiaoshi = 0;
	//	PosSenser_SINS.Location.y = ld_data.translation.y;
	//	PosSenser_SINS.Location.x = ld_data.translation.x;
	//}

	//if ((ps_data.translation.x != 0)
	//	&& (ps_data.translation.y != 0)
	//	&& (ps_data.velocity.x != 0)
	//	&& (ps_data.velocity.y != 0))		//串口有数
	//{
	//	PosSenser_SINS.Velocity.y = ps_data.velocity.x;
	//	PosSenser_SINS.Velocity.x = ps_data.velocity.y;
	//	PosSenser_SINS.Location.y = ps_data.translation.x;
	//	PosSenser_SINS.Location.x = ps_data.translation.y;
	//}
	//else {
	//	PosSenser_SINS.Velocity.y = OpticalFlow_SINS.Velocity.y;
	//	PosSenser_SINS.Velocity.x = OpticalFlow_SINS.Velocity.x;
	//	PosSenser_SINS.Location.y = OpticalFlow_SINS.Location.y;
	//	PosSenser_SINS.Location.x = OpticalFlow_SINS.Location.x;
	//}

	//格子位置
	if (PosSenser_SINS.Location.y <= 25 + SDK_Zero_POSITION.y)			Gezi_y = 1;
	else if (PosSenser_SINS.Location.y <= 75 + SDK_Zero_POSITION.y)		Gezi_y = 2;
	else if (PosSenser_SINS.Location.y <= 125 + SDK_Zero_POSITION.y)	Gezi_y = 3;
	else if (PosSenser_SINS.Location.y <= 175 + SDK_Zero_POSITION.y)	Gezi_y = 4;
	else if (PosSenser_SINS.Location.y <= 225 + SDK_Zero_POSITION.y)	Gezi_y = 5;
	else if (PosSenser_SINS.Location.y <= 275 + SDK_Zero_POSITION.y)	Gezi_y = 6;
	else																Gezi_y = 7;

	if (PosSenser_SINS.Location.x >= -25 + SDK_Zero_POSITION.x)			Gezi_x = 1;
	else if (PosSenser_SINS.Location.x >= -75 + SDK_Zero_POSITION.x)	Gezi_x = 2;
	else if (PosSenser_SINS.Location.x >= -125 + SDK_Zero_POSITION.x)	Gezi_x = 3;
	else if (PosSenser_SINS.Location.x >= -175 + SDK_Zero_POSITION.x)	Gezi_x = 4;
	else if (PosSenser_SINS.Location.x >= -225 + SDK_Zero_POSITION.x)	Gezi_x = 5;
	else if (PosSenser_SINS.Location.x >= -275 + SDK_Zero_POSITION.x)	Gezi_x = 6;
	else if (PosSenser_SINS.Location.x >= -325 + SDK_Zero_POSITION.x)	Gezi_x = 7;
	else if (PosSenser_SINS.Location.x >= -375 + SDK_Zero_POSITION.x)	Gezi_x = 8;
	else 																Gezi_x = 9;

	if ((Gezi_x_Lst != Gezi_x) || (Gezi_y_Lst != Gezi_y))
	{
		Of_Begin_Flag = 1;
		Gezi_CF = 1;
		Gezi_x_Lst = Gezi_x;
		Gezi_y_Lst = Gezi_y;
	}

}

u16 U6_Will_Send = 0;
extern u8 U6_Send_Buff[10];
u8 Anm_Kind = 0;
u8 U6_Send_Stage = 0;
u8 Send_Anml_Num = 0;
u8 EW_Send = 0;
u16 EW_Send_Cnd = 0;
u8 EW_Anm_Kind = 0;
u8 EW_Send_Anml_Num = 0;
u8 U6_Send_Buff1[10] = { 0 };
u8 U6_Send_Buff2[10] = { 0 };

void Ground_Station(void) 
{

	if (U6_Send_Stage == 1)
	{
		EW_Send_Cnd++;
		if (EW_Send_Cnd > 100 && EW_Send)
		{
			EW_Send = 0;
			U6_Will_Send = 0;
			U6_Send_Buff[0] = 0xaa;
			U6_Send_Buff[1] = 0xff;
			U6_Send_Buff[2] = EW_Anm_Kind;//种类
			U6_Send_Buff[3] = Gezi_x_Rec;//X
			U6_Send_Buff[4] = Gezi_y_Rec;//Y
			U6_Send_Buff[5] = EW_Send_Anml_Num;//数量
			U6_Send_Buff[6] = U6_Send_Stage;
			U6_Send_Buff[7] = 0xaf;
			DMA_USART6_Send_Buff(U6_Send_Buff, 8);
		}
		if (EW_Send_Cnd > 200)
		{
			EW_Send_Cnd = 0;
			U6_Will_Send = 0;
			U6_Send_Buff1[0] = 0xaa;
			U6_Send_Buff1[1] = 0xff;
			U6_Send_Buff1[2] = Anm_Kind;//种类
			U6_Send_Buff1[3] = Gezi_x_Rec;//X
			U6_Send_Buff1[4] = Gezi_y_Rec;//Y
			U6_Send_Buff1[5] = Send_Anml_Num;//数量
			U6_Send_Buff1[6] = U6_Send_Stage;
			U6_Send_Buff1[7] = 0xaf;
			DMA_USART6_Send_Buff(U6_Send_Buff1, 8);
			U6_Send_Stage = 0;
		}
	}

	U6_Will_Send++;
	if (U6_Will_Send >= 200)
	{
		U6_Will_Send = 0;
		U6_Send_Buff2[0] = 0xaa;
		U6_Send_Buff2[1] = 0xff;
		U6_Send_Buff2[2] = 0;//种类
		U6_Send_Buff2[3] = Gezi_x;//X
		U6_Send_Buff2[4] = Gezi_y;//Y
		U6_Send_Buff2[5] = 0;//数量
		U6_Send_Buff2[6] = 0;
		U6_Send_Buff2[7] = 0xaf;
		DMA_USART6_Send_Buff(U6_Send_Buff2, 8);
	}

}


/*****************遥控器行程设置**********************/
uint16 Motor_PWM_1,Motor_PWM_2,Motor_PWM_3,Motor_PWM_4,Motor_PWM_5,Motor_PWM_6;//六个电机输出PWM
uint16 Last_Motor_PWM_1,Last_Motor_PWM_2,Last_Motor_PWM_3,Last_Motor_PWM_4,Last_Motor_PWM_5,Last_Motor_PWM_6;//上次六个电机输出PWM
uint8 MotorTest=0;//电机序号测试
uint8_t Controler_High_Mode=0,Last_Controler_High_Mode=0;
uint8_t Controler_Horizontal_Mode=0,Last_Controler_Horizontal_Mode=0;
uint8_t Controler_Land_Mode=1,Last_Controler_Land_Mode=1;
uint8_t Reserve_Mode=0,Last_Reserve_Mode=0;
uint8_t Reserve_Mode_Fast_Exchange_Flag=0,Reserve_Mode_Fast_Exchange_Cnt=0;
uint16_t Reserve_Mode_Cnt=0;
uint8_t Control_Mode_Change=0;
uint16_t High_Hold_Throttle= Thr_Hover_Default;
uint8_t  Pos_Hold_SetFlag=0;
uint8_t SDK_Ctrl_Mode=0;
uint8_t SDK_Ctrl_Mode_LY = 0, Last_SDK_Ctrl_Mode_LY = 0;
uint8_t SDK_WAIT_TO_FLY = 0;
uint16 Throttle = 0, Last_Throttle = 0;
uint16 PPM4_LAST = 0;
uint16 PPM5_LAST = 0;
extern int Delivery_Stage;

void Remote_Control(void)
{
	/************************************************************************/
	/* 加锁操作                                                             */
	/************************************************************************/
	if (Throttle_Control <= 1020 && Yaw_Control >= Yaw_Max*Scale_Pecent_Max)	Lock_Makesure_Cnt++;
	else	Lock_Makesure_Cnt = 0;
	if (Throttle_Control <= 1020 && Yaw_Control >= Yaw_Max*Scale_Pecent_Max&&Lock_Makesure_Cnt > 200)
	{
		Controler_State = Lock_Controler;
		Lock_Makesure_Cnt = 0;
		Unlock_Makesure_Cnt = 0;
		Reset_RC_Calibartion(1);
	}

	/************************************************************************/
	/* 解锁操作                                                             */
	/************************************************************************/
	if (Throttle_Control <= 1020 && Yaw_Control <= -Yaw_Max*Scale_Pecent_Max)	Unlock_Makesure_Cnt++;
	else	Unlock_Makesure_Cnt = 0;
	if (Throttle_Control <= 1020 && Yaw_Control <= -Yaw_Max*Scale_Pecent_Max&&Unlock_Makesure_Cnt > 200 && Check_Calibration_Flag() == 0x00)
	{
		Controler_State = Unlock_Controler;
		Lock_Makesure_Cnt = 0;
		Unlock_Makesure_Cnt = 0;
		Reset_RC_Calibartion(1);
		MY_SDK_REST();
	}

	/*************控制器模式选择******************/
	Last_Controler_High_Mode = Controler_High_Mode;					//上次高度控制模式
	Last_Controler_Horizontal_Mode = Controler_Horizontal_Mode;		//上次位置控制模式
	Last_SDK_Ctrl_Mode_LY = SDK_Ctrl_Mode_LY;

	/***************** 5通道*************************/
	if (PPM_Databuf[4] - PPM4_LAST >= 100 || PPM_Databuf[4] - PPM4_LAST <= -100 || PPM_Databuf[5] - PPM5_LAST >= 100 || PPM_Databuf[5] - PPM5_LAST <= -100)
		key_flag = 0;
	if (PPM_Databuf[4] <= (RC_Calibration[4].min + RC_Calibration[4].deadband))      //自稳模式
	{
		Controler_High_Mode = 0;
		Controler_Horizontal_Mode = 0;
	}
	else if ((PPM_Databuf[4] <= (RC_Calibration[4].middle + RC_Calibration[4].deadband))
		&& (PPM_Databuf[4] >= (RC_Calibration[4].middle - RC_Calibration[4].deadband)))   //定高模式
	{
		Controler_High_Mode = 1;
		Controler_Horizontal_Mode = 0;
	}
	else if (PPM_Databuf[4] >= (RC_Calibration[4].max - RC_Calibration[4].deadband) || key_flag == 1)		//定点模式
	{
		Controler_High_Mode = 1;
		Controler_Horizontal_Mode = 1;
	}

	/***************** 6通道*************************/
	if (PPM_Databuf[5] >= (RC_Calibration[5].max - RC_Calibration[5].deadband) || key_flag == 1)
	{
		Controler_High_Mode = 1;
		Controler_Horizontal_Mode = 1;
		SDK_Ctrl_Mode_LY = 1;
	}
	else
	{
		SDK_Ctrl_Mode_LY = 0;
	}

	/***************** 8通道*************************/
	if (PPM_Databuf[7] <= (RC_Calibration[7].min + RC_Calibration[7].deadband)) 
	{
		Delivery_Stage = 0;
	}
	else if ((PPM_Databuf[7] <= (RC_Calibration[7].middle + RC_Calibration[7].deadband))
		&& (PPM_Databuf[7] >= (RC_Calibration[7].middle - RC_Calibration[7].deadband)))   
	{
		Delivery_Stage = 1;
	}
	else
	{
		Delivery_Stage = 2;
	}

}


void Position_Control(void)
{
	/**************定高通道切换处理******************/
	if (Controler_High_Mode != Last_Controler_High_Mode)
	{
		if (Controler_High_Mode == 1)
		{
			//将当前的油门值设置成“悬停基础油门值”  避免切到定高模式突然往下掉或往上升
			//将开关拨动瞬间的惯导高度设置为期望高度
			Total_Controller.High_Position_Control.Expect = Altitude_Position;
		}
		else if (Controler_High_Mode == 0)    //定高切自稳
		{
			Throttle_Control_Reset();
		}
	}

	/**************位置通道切换处理******************/
	if (Controler_Horizontal_Mode != Last_Controler_Horizontal_Mode)
	{
		if (Controler_Horizontal_Mode == 1)
		{
			OpticalFlow_Pos_Ctrl_Expect.x = PosSenser_SINS.Location.x;
			OpticalFlow_Pos_Ctrl_Expect.y = PosSenser_SINS.Location.y;
		}
		else if (Controler_High_Mode == 0)    //定高切自稳
		{
			Throttle_Control_Reset();
		}
	}

	/************** SDK通道切换处理******************/
	if (SDK_Ctrl_Mode_LY != Last_SDK_Ctrl_Mode_LY)
	{
		if (SDK_Ctrl_Mode_LY == 1)
		{
			//将当前的油门值设置成“悬停基础油门值”  避免切到定高模式突然往下掉或往上升
			High_Hold_Throttle = Thr_Hover_Default;
			//将开关拨动瞬间的惯导高度设置为期望高度
			MY_SDK_REST();
			//Total_Controller.High_Position_Control.Expect =80;
			SDK_WAIT_TO_FLY = 1;
		}
		else if (SDK_Ctrl_Mode_LY == 0)
		{
			Throttle_Control_Reset();
		}
	}

	/************************位置环控制器*********************************/
	if (Controler_High_Mode == 1)
	{
		Throttle = Althold_Control();
	}
	else if (Controler_High_Mode == 0)
	{
		Throttle = Throttle_Control;
	}

	if (Controler_Horizontal_Mode == 1)
	{
	/************************进入SDK模式*********************************/
		if (SDK_Ctrl_Mode_LY == 1)		FFCY_SDK_Control(0);
		else		Loiter_Control(0);
	}
	else if (Controler_Horizontal_Mode == 0)
	{
		Total_Controller.Pitch_Angle_Control.Expect = Target_Angle[0];
		Total_Controller.Roll_Angle_Control.Expect = Target_Angle[1];
	}

	PPM4_LAST = PPM_Databuf[4];
	PPM5_LAST = PPM_Databuf[5];
}


uint16 Value_Limit(uint16 Min,uint16 Max,uint16 Data)
{
  if(Data>=Max) Data=Max;
  else if(Data<=Min) Data=Min;
  return Data;
}


int Yaw_Control_flag = 0;
float Pitch_Offset = 0;
float Roll_Offset = 0;

void Angle_Control()//角度环节
{
  static uint16 Yaw_Cnt=0;

  //设置角度反馈值
  Total_Controller.Pitch_Angle_Control.FeedBack = (Pitch - Pitch_Offset);
  Total_Controller.Roll_Angle_Control.FeedBack = (Roll - Roll_Offset);
  Total_Controller.Yaw_Angle_Control.FeedBack = Yaw;											//偏航角反馈
  //PID计算输出
  PID_Control(&Total_Controller.Pitch_Angle_Control,0.005f);
  PID_Control(&Total_Controller.Roll_Angle_Control, 0.005f);
  
/**********************************************偏航角控制*********************************************/


  if (Throttle_Control <= 1080 && SDK_Ctrl_Mode_LY !=1 )
  {
	  Yaw_Control_flag = 1;
  }

  if((Yaw_Control<-10|| Yaw_Control>10) && Throttle_Control >= 1080)																							//波动偏航方向杆后，只进行内环角速度控制
  {
	Yaw_Control_flag = 1;
    Total_Controller.Yaw_Angle_Control.Expect=0;				//偏航角期望给0,不进行角度控制
    Total_Controller.Yaw_Gyro_Control.Expect=Yaw_Control;		//偏航角速度环期望，直接来源于遥控器打杆量
  }
  else //(Yaw_Control == 0)//偏航杆置于中位
  {
	  if (Yaw_Cnt <= 500)//无头模式、飞机上电后一段时间锁定偏航角，磁力计、陀螺仪融合需要一段时间，这里取500
	  {
		  Yaw_Cnt++;
	  }
	  /************************复位偏航角期望*****************************************/
	  ////首次回中时 复位偏航角期望
	  if (Yaw_Control_flag == 1)
	  {
		  Total_Controller.Yaw_Angle_Control.Expect = Yaw;
		  Yaw_Control_flag = 0;
	  }
	  /************************复位偏航角期望---END************************************/
	  PID_Control_Yaw(&Total_Controller.Yaw_Angle_Control);										//偏航角度控制
	  Total_Controller.Yaw_Gyro_Control.Expect = Total_Controller.Yaw_Angle_Control.Control_OutPut;//偏航角速度环期望，来源于偏航角度控制器输出
  }

  
}


uint16_t Yaw_Control_Fault_Cnt=0;
float Last_Yaw_Gyro_Control_Expect=0,Yaw_Gyro_Control_Expect_Delta=0;
float Last_Pitch_Gyro_Control_Expect=0,Pitch_Gyro_Control_Expect_Delta=0;
float Last_Roll_Gyro_Control_Expect=0,Roll_Gyro_Control_Expect_Delta=0;



void Gyro_Control()//角速度环
{

	/***************内环角速度期望****************/
	Total_Controller.Pitch_Gyro_Control.Expect =  Total_Controller.Pitch_Angle_Control.Control_OutPut;
	Total_Controller.Roll_Gyro_Control.Expect =  Total_Controller.Roll_Angle_Control.Control_OutPut;

	/***************内环角速度反馈****************/
	Total_Controller.Pitch_Gyro_Control.FeedBack = MPitch_Gyro;
	Total_Controller.Roll_Gyro_Control.FeedBack = MRoll_Gyro;
	Total_Controller.Yaw_Gyro_Control.FeedBack = MYaw_Gyro;

	/***************内环角速度控制：微分参数动态调整****************/
	PID_Control(&Total_Controller.Pitch_Gyro_Control, 0.02f);
	PID_Control(&Total_Controller.Roll_Gyro_Control, 0.02f);
	PID_Control(&Total_Controller.Yaw_Gyro_Control, 0.02f);

}


uint16_t Throttle_Output=0;
void Throttle_Angle_Compensate()//油门倾角补偿
{
  float CosPitch_CosRoll=ABS(Cos_Pitch*Cos_Roll);
  float Throttle_Makeup=0;
  float Temp=0;
  if(CosPitch_CosRoll>=0.999999)  CosPitch_CosRoll=0.999999;
  if(CosPitch_CosRoll<=0.000001)  CosPitch_CosRoll=0.000001;
  if(CosPitch_CosRoll<=0.50)  CosPitch_CosRoll=0.50;//Pitch,Roll约等于30度
  if(Throttle>=Thr_Start)//大于起转油门量
  {
    Temp=(uint16_t)(FFCY_MAX(ABS(100*Pitch),ABS(100*Roll)));
    Temp=constrain_float(9000-Temp,0,3000)/(3000*CosPitch_CosRoll);
    Throttle_Makeup=(Throttle-Thr_Start)*Temp;//油门倾角补偿
    Throttle_Output=(uint16_t)(Thr_Start+Throttle_Makeup);
    Throttle_Output=(uint16_t)(constrain_float(Throttle_Output,Thr_Start,2000));
  }
  else Throttle_Output=Throttle;
}


/**************************************************************
***************************************************************
X型安装方式，电机序号与姿态角关系
                  -
                Pitch
          3#             1#
            *          *
-   Roll          *         Roll   +
            *          *
          2#             4#
                Pitch
                  +
加速度传感器轴向与载体X、Y、Z同轴，沿轴向原点看，逆时针旋转角度为+
Y Aixs
*
*
*
* * * ** * *   X Axis
(0)
*******************************************************************
******************************************************************/
uint16_t Idel_Cnt=0;
#define Idel_Transition_Gap 2//怠速递增间隔时间 10*5=50ms
#define Idel_Transition_Period 100//怠速启动最大计数器  50ms*100=5s
uint16_t Thr_Idle_Transition_Cnt=0;
u8 DC_Moto_output_switch = 0;

void Control_Output()
{
	Throttle_Output = Throttle;

	if (Controler_State == Unlock_Controler  )//解锁
	{
		Motor_PWM_1 = Int_Sort(Moter1_Thr_Scale*Throttle_Output + Moter1_Roll_Scale*Total_Controller.Roll_Gyro_Control.Control_OutPut + Moter1_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut + Moter1_Yaw_Scale*Total_Controller.Yaw_Gyro_Control.Control_OutPut);
		Motor_PWM_2 = Int_Sort(Moter2_Thr_Scale*Throttle_Output + Moter2_Roll_Scale*Total_Controller.Roll_Gyro_Control.Control_OutPut + Moter2_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut + Moter2_Yaw_Scale*Total_Controller.Yaw_Gyro_Control.Control_OutPut);
		Motor_PWM_3 = Int_Sort(Moter3_Thr_Scale*Throttle_Output + Moter3_Roll_Scale*Total_Controller.Roll_Gyro_Control.Control_OutPut + Moter3_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut + Moter3_Yaw_Scale*Total_Controller.Yaw_Gyro_Control.Control_OutPut);
		Motor_PWM_4 = Int_Sort(Moter4_Thr_Scale*Throttle_Output + Moter4_Roll_Scale*Total_Controller.Roll_Gyro_Control.Control_OutPut + Moter4_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut + Moter4_Yaw_Scale*Total_Controller.Yaw_Gyro_Control.Control_OutPut);

		//小于起飞油门 遥控器控制量直接传递
		if (Throttle_Output < Thr_Fly_Start)
		{
			Motor_PWM_1 = Int_Sort(Throttle_Output);
			Motor_PWM_2 = Int_Sort(Throttle_Output);
			Motor_PWM_3 = Int_Sort(Throttle_Output);
			Motor_PWM_4 = Int_Sort(Throttle_Output);
			Take_Off_Reset();//清积分
		}
		Motor_PWM_1 = Value_Limit(Thr_Idle, 2000, Motor_PWM_1);//总输出限幅
		Motor_PWM_2 = Value_Limit(Thr_Idle, 2000, Motor_PWM_2);
		Motor_PWM_3 = Value_Limit(Thr_Idle, 2000, Motor_PWM_3);
		Motor_PWM_4 = Value_Limit(Thr_Idle, 2000, Motor_PWM_4);

	}
	else//未解锁，停转
	{
		Motor_PWM_1 = Thr_Min;
		Motor_PWM_2 = Thr_Min;
		Motor_PWM_3 = Thr_Min;
		Motor_PWM_4 = Thr_Min;
		Take_Off_Reset();//清积分
		Throttle_Control_Reset();
	}

	if (Controler_State == Unlock_Controler && SDK_Ctrl_Mode_LY == 1 && SDK_WAIT_TO_FLY==1)
	{
		Motor_PWM_1 = Thr_Idle;
		Motor_PWM_2 = Thr_Idle;
		Motor_PWM_3 = Thr_Idle;
		Motor_PWM_4 = Thr_Idle;
	}
	Motor_PWM_1 = Value_Limit(Thr_Min, 2000, Motor_PWM_1);//总输出限幅
	Motor_PWM_2 = Value_Limit(Thr_Min, 2000, Motor_PWM_2);
	Motor_PWM_3 = Value_Limit(Thr_Min, 2000, Motor_PWM_3);
	Motor_PWM_4 = Value_Limit(Thr_Min, 2000, Motor_PWM_4);

	if (MotorTest == 1)
	{
		PWM_Set(PPM_Databuf[2], PPM_Databuf[2], PPM_Databuf[2], PPM_Databuf[2]);
	}
	else
	{
		PWM_Set(Motor_PWM_4, Motor_PWM_3, Motor_PWM_2, Motor_PWM_1);
	}
}

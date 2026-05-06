#include "Headfile.h"
#include "FFCY_Position_Ctrl.h"

Vector2f OpticalFlow_Pos_Ctrl_Expect = { 0 };
Vector2f OpticalFlow_Pos_Ctrl_Err = { 0 };
Vector2f OpticalFlow_Pos_Ctrl_Integrate = { 0 };
Vector2f OpticalFlow_Pos_Ctrl_Output = { 0 };

Vector2f OpticalFlow_Ctrl_Expect = { 0 };
Vector2f OpticalFlow_Ctrl_Err = { 0 };
Vector2f OpticalFlow_Ctrl_Integrate = { 0 };
Vector2f OpticalFlow_Ctrl_Output = { 0 };

Vector2f accel_target, angle_target;

void OpticalFlow_SINS_Reset(void)
{
	OpticalFlow_SINS.Location.y = 0;
	OpticalFlow_SINS.Location.x = 0;
	OpticalFlow_SINS.Velocity.y = 0;
	OpticalFlow_SINS.Velocity.x = 0;
	OpticalFlow_Position.x = 0;
	OpticalFlow_Position.y = 0;
}


void OpticalFlow_Ctrl_Reset(void)
{
	OpticalFlow_Ctrl_Integrate.x = 0.0f;
	OpticalFlow_Ctrl_Integrate.y = 0.0f;
	OpticalFlow_Pos_Ctrl_Integrate.x = 0.0f;
	OpticalFlow_Pos_Ctrl_Integrate.y = 0.0f;
	OpticalFlow_Pos_Ctrl_Expect.x = 0;
	OpticalFlow_Pos_Ctrl_Expect.y = 0;
}

float OpticalFlow_Expect_Speed_Mapping(float input, uint16_t input_max, float output_max)
{
	float output_speed = 0;
	float temp_scale = (float)(input / input_max);
	temp_scale = constrain_float(temp_scale, -1.0f, 1.0f);
	if (temp_scale >= 0) output_speed = (float)(output_max*temp_scale*temp_scale);
	else output_speed = (float)(-output_max*temp_scale*temp_scale);
	return output_speed;
}

void OpticalFlow_Set_Target_Point(Vector2f target)
{
	OpticalFlow_Pos_Ctrl_Expect.x = target.x;
	OpticalFlow_Pos_Ctrl_Expect.y = target.y;
}

void OpticalFlow_Set_Target_Vel(Vector2f target)
{
	OpticalFlow_Ctrl_Expect.x = target.x;
	OpticalFlow_Ctrl_Expect.y = target.y;
}

u8 XY_Move_flag = 0;
void Loiter_Control(uint8_t force_brake_flag)
{
	//无水平遥控量给定
	if (Roll_Control == 0 && Pitch_Control == 0)
	{

		if (XY_Move_flag == 1)
		{
			//打杆回中后，根据当前速度、倾角判断是否进行悬停 避免出现当前倾角过大 位置估计错误的情况
			if (force_brake_flag || (rMat[2][2] >= 0.95 &&pythagorous2(PosSenser_SINS.Velocity.y, PosSenser_SINS.Velocity.x) <= 40))
			{
				OpticalFlow_Pos_Ctrl_Expect.y = PosSenser_SINS.Location.y;
				OpticalFlow_Pos_Ctrl_Expect.x = PosSenser_SINS.Location.x;
				XY_Move_flag = 0;
			}
			else  //打杆回中未满足悬停条件时，只进行速度控制 等待满足条件
			{
				OpticalFlow_Pos_Ctrl_Output.x = 0;
				OpticalFlow_Pos_Ctrl_Output.y = 0;
				XY_Move_flag = 1;
			}
		}

		/**************************光流位置控制器************************************/
		if (XY_Move_flag == 0)
				OpticalFlow_Pos_Control();
		/**************************基于模型的加速度-姿态角映射，相比直接给姿态，参数差异大概在20倍左右************************************/
		/***********当只需要速度控制时，开启以下注释*************/
		OpticalFlow_Pos_Ctrl_Output.x = 0;
		OpticalFlow_Pos_Ctrl_Output.y = 0;
		OpticalFlow_Vel_Control(OpticalFlow_Pos_Ctrl_Output);//速度期望
	}
	else
	{
		XY_Move_flag = 1;
		OpticalFlow_Pos_Ctrl_Output.x = OpticalFlow_Expect_Speed_Mapping(Target_Angle[1], Pit_Rol_Max, OpticalFlow_Speed_Control_Max);
		OpticalFlow_Pos_Ctrl_Output.y = OpticalFlow_Expect_Speed_Mapping(-Target_Angle[0], Pit_Rol_Max, OpticalFlow_Speed_Control_Max);
		OpticalFlow_Vel_Control(OpticalFlow_Pos_Ctrl_Output);//速度期望

	}
}


Vector2f SDK_OUT_Posion_I = { 10,10 };
Vector2f OpticalFlow_Pos_Ctrl_Original_Err = { 0 };
void OpticalFlow_Pos_Control(void)
{
	static uint16_t OpticalFlow_Pos_Ctrl_Cnt = 0;
	OpticalFlow_Pos_Ctrl_Cnt++;
	if (OpticalFlow_Pos_Ctrl_Cnt >= 10)//50ms控制一次速度，避免输入频率过大，系统响应不过来
	{
		//计算位置偏差  
		OpticalFlow_Pos_Ctrl_Original_Err.y = OpticalFlow_Pos_Ctrl_Expect.y - PosSenser_SINS.Location.y;
		OpticalFlow_Pos_Ctrl_Original_Err.x = OpticalFlow_Pos_Ctrl_Expect.x - PosSenser_SINS.Location.x;

		//位置偏差的旋转矩阵
		OpticalFlow_Pos_Ctrl_Err.y = cos(Yaw* DEG2RAD)*OpticalFlow_Pos_Ctrl_Original_Err.y - sin(Yaw* DEG2RAD)*OpticalFlow_Pos_Ctrl_Original_Err.x;
		OpticalFlow_Pos_Ctrl_Err.x = cos(Yaw* DEG2RAD)*OpticalFlow_Pos_Ctrl_Original_Err.x + sin(Yaw* DEG2RAD)*OpticalFlow_Pos_Ctrl_Original_Err.y;

		//积分分离
		if (ABS(OpticalFlow_Pos_Ctrl_Err.x) <= 20.0)
			OpticalFlow_Pos_Ctrl_Integrate.x += Total_Controller.Optical_Position_Control.Ki * OpticalFlow_Pos_Ctrl_Err.x;
		if (ABS(OpticalFlow_Pos_Ctrl_Err.y) <= 20.0)
			OpticalFlow_Pos_Ctrl_Integrate.y += Total_Controller.Optical_Position_Control.Ki * OpticalFlow_Pos_Ctrl_Err.y;
		//积分限幅
		if (OpticalFlow_Pos_Ctrl_Integrate.x >= SDK_OUT_Posion_I.x)  OpticalFlow_Pos_Ctrl_Integrate.x = SDK_OUT_Posion_I.x;
		if (OpticalFlow_Pos_Ctrl_Integrate.x <= -SDK_OUT_Posion_I.x)  OpticalFlow_Pos_Ctrl_Integrate.x = -SDK_OUT_Posion_I.x;
		if (OpticalFlow_Pos_Ctrl_Integrate.y >= SDK_OUT_Posion_I.y)  OpticalFlow_Pos_Ctrl_Integrate.y = SDK_OUT_Posion_I.y;
		if (OpticalFlow_Pos_Ctrl_Integrate.y <= -SDK_OUT_Posion_I.y)  OpticalFlow_Pos_Ctrl_Integrate.y = -SDK_OUT_Posion_I.y;


		//计算位置控制输出
		OpticalFlow_Pos_Ctrl_Output.x = Total_Controller.Optical_Position_Control.Kp*OpticalFlow_Pos_Ctrl_Err.x + OpticalFlow_Pos_Ctrl_Integrate.x;
		OpticalFlow_Pos_Ctrl_Output.y = Total_Controller.Optical_Position_Control.Kp*OpticalFlow_Pos_Ctrl_Err.y + OpticalFlow_Pos_Ctrl_Integrate.y;
		OpticalFlow_Pos_Ctrl_Cnt = 0;
	}
}


void OpticalFlow_Vel_Control(Vector2f target)
{
	static uint16_t OpticalFlow_Vel_Ctrl_Cnt = 0;
	OpticalFlow_Ctrl_Expect.x = target.x; //roll
	OpticalFlow_Ctrl_Expect.y = target.y; //pitch
	OpticalFlow_Vel_Ctrl_Cnt++;
	if (OpticalFlow_Vel_Ctrl_Cnt >= 4)//20ms控制一次速度，避免输入频率过大，系统响应不过来
	{
		OpticalFlow_Ctrl_Err.y = constrain_float(OpticalFlow_Ctrl_Expect.y - PosSenser_SINS.Velocity.y, -Total_Controller.Optical_Speed_Control.Err_Max, Total_Controller.Optical_Speed_Control.Err_Max);//30
		OpticalFlow_Ctrl_Err.x = -constrain_float(OpticalFlow_Ctrl_Expect.x - PosSenser_SINS.Velocity.x, -Total_Controller.Optical_Speed_Control.Err_Max, Total_Controller.Optical_Speed_Control.Err_Max);

		if (ABS(OpticalFlow_Ctrl_Err.x) <= Total_Controller.Optical_Speed_Control.Integrate_Separation_Err)
			OpticalFlow_Ctrl_Integrate.x += Total_Controller.Optical_Speed_Control.Ki*OpticalFlow_Ctrl_Err.x;//0.1  15
		if (ABS(OpticalFlow_Ctrl_Err.y) <= Total_Controller.Optical_Speed_Control.Integrate_Separation_Err)
			OpticalFlow_Ctrl_Integrate.y += Total_Controller.Optical_Speed_Control.Ki*OpticalFlow_Ctrl_Err.y;

		OpticalFlow_Ctrl_Integrate.x = constrain_float(OpticalFlow_Ctrl_Integrate.x, -Total_Controller.Optical_Speed_Control.Integrate_Max, Total_Controller.Optical_Speed_Control.Integrate_Max);
		OpticalFlow_Ctrl_Integrate.y = constrain_float(OpticalFlow_Ctrl_Integrate.y, -Total_Controller.Optical_Speed_Control.Integrate_Max, Total_Controller.Optical_Speed_Control.Integrate_Max);

		OpticalFlow_Ctrl_Output.x = OpticalFlow_Ctrl_Integrate.x + Total_Controller.Optical_Speed_Control.Kp*OpticalFlow_Ctrl_Err.x;//4.5
		OpticalFlow_Ctrl_Output.y = OpticalFlow_Ctrl_Integrate.y + Total_Controller.Optical_Speed_Control.Kp*OpticalFlow_Ctrl_Err.y*0.7;

		angle_target.y = -constrain_float(OpticalFlow_Ctrl_Output.y, -Total_Controller.Optical_Speed_Control.Control_OutPut_Limit, Total_Controller.Optical_Speed_Control.Control_OutPut_Limit);//450
		angle_target.x = -constrain_float(OpticalFlow_Ctrl_Output.x, -Total_Controller.Optical_Speed_Control.Control_OutPut_Limit, Total_Controller.Optical_Speed_Control.Control_OutPut_Limit);//期望运动加速度

																																																//	accel_to_lean_angles(accel_target,&angle_target);//期望运动加速度转期望姿态倾角
		Total_Controller.Pitch_Angle_Control.Expect = angle_target.y;
		Total_Controller.Roll_Angle_Control.Expect = angle_target.x;
		OpticalFlow_Vel_Ctrl_Cnt = 0;
	}
}


 /*************以下为默认值，校准遥控器后会自动赋值**********************/
#define Deadzone 100
u8 Z_Move_flag = 0;
extern uint8_t SDK_Ctrl_Mode_LY;
float Altitude_Speed;
float Altitude_Position;
float Targ_speed = 0.0f;
uint16 Althold_Control(void)
{
	static uint16 High_Pos_Control_Cnt = 0;//高度位置控制计数器

	/****************定高：高度位置环+速度环+加速度环，控制周期分别为8ms、4ms、4ms*******************/
	/*******************************定高高度控制器***********************************************************************************/

	if (RC3_Origal_Value > (400 + Deadzone))//摇杆上推
	{
		Z_Move_flag = 1;
		Targ_speed = (float)(RC3_Origal_Value - (400 + Deadzone)) / (400 - Deadzone);//范围0~1
		Total_Controller.High_Speed_Control.Expect = Climb_Up_Speed_Max* constrain_float(Targ_speed*Targ_speed, 0, 1);
	}
	else if (RC3_Origal_Value < (400 - Deadzone))//摇杆下推
	{
		Z_Move_flag = 1;
		Targ_speed = (float)(RC3_Origal_Value - (400 - Deadzone)) / (400 - Deadzone);//范围0~-1
		Total_Controller.High_Speed_Control.Expect = -Climb_Down_Speed_Max*constrain_float(Targ_speed*Targ_speed, 0, 1);
	}
	else if (RC3_Origal_Value >= (400 - Deadzone) && RC3_Origal_Value <= (400 + Deadzone))  //摇杆在中值位置
	{
		Targ_speed = 0.0f;
		//遥杆回中
		if (Z_Move_flag == 1 && SDK_Ctrl_Mode_LY == 0)
		{
			Total_Controller.High_Position_Control.Expect = Altitude_Position;
			Z_Move_flag = 0;
		}

		High_Pos_Control_Cnt++;
		if (High_Pos_Control_Cnt >= 2)//竖直高度控制周期2*5=10ms
		{
			High_Pos_Control_Cnt = 0;

			/************* 海拔高度位置控制器	***/
			Total_Controller.High_Position_Control.FeedBack = Altitude_Position;//反馈
			PID_Control(&Total_Controller.High_Position_Control,0.01f);
			Total_Controller.High_Speed_Control.Expect = Total_Controller.High_Position_Control.Control_OutPut;
		}
	}

	/*******************************定高速度控制器开始********************************************************************************/
	Total_Controller.High_Speed_Control.FeedBack = Altitude_Speed;//惯导速度估计给速度反馈
	PID_Control(&Total_Controller.High_Speed_Control,0.005f);//海拔高度速度控制
	
	/*****************************************高度控制器结束，给定油门控制量***********************************************************/
	return Int_Sort(High_Hold_Throttle + Total_Controller.High_Speed_Control.Control_OutPut);//油门来源于高度速度控制器输出
}




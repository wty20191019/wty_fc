#include "Headfile.h"
#include "FFCY_OpticalFlow.h"
#include "OpticalFlow_LC306.h"



SINS OpticalFlow_SINS;
Vector2f OpticalFlow_Speed, OpticalFlow_Position ;

#define LPF_1_(hz,t,in,out) ((out) += ( 1 / ( 1 + 1 / ( (hz) *3.14f *(t) ) ) ) *( (in) - (out) )) //一阶低通滤波
#define LIMIT( x,min,max ) ( ((x) < (min)) ? (min) : ( ((x) > (max))? (max) : (x)))
#define safe_div(numerator,denominator,safe_value) ( (denominator == 0)? (safe_value) : ((numerator) / (denominator))) //安全整除
//**************************飞**********凡*********创**********翼******************************************
//*	函 数 名 : filter_1(double base_hz, double gain_hz, double dT, double in, _filter_1_st *f1)
//*	功能说明 : 光流速度数据融合滤波 
//*	形    参 ：无
//*	返 回 值 : 无
//* 作者    ：FFCY
//* 最后更改：LZH 李智恒 2019:7:15 14:08:33 
//**********************************(C)COPYRIGHT 2019 FFCY Team*********************************************/
//动态调整滤波截止频率的一阶滤波
void filter_1_ffcy(double base_hz, double gain_hz, double dT, double in, _filter_1_st *f1)
{
	LPF_1_(gain_hz, dT, (in - f1->out), f1->a); //低通后的变化量
	f1->b = (in - f1->out) * (in - f1->out); //求一个数平方函数
	f1->e_nr = LIMIT(safe_div((f1->a) * (f1->a), ((f1->b) + (f1->a) * (f1->a)), 0), 0, 1);
	//变化量的有效率，LIMIT 将该数限制在 0-1 之间，safe_div 为安全除法
	LPF_1_(base_hz *f1->e_nr, dT, in, f1->out); //低通跟踪
}


float RESOLUTION=0;
float Real_DetX = 0, Real_DetY = 0;      //真实的每次平动距离   单位CM
float Last_Sum_flow_x, Last_Sum_flow_y;
float I_DetX = 0, I_DetY = 0;   //状态机 从预积分获取的平动位移  大小取决于光流的处理次数
double gyro_lpf_Detx, gyro_lpf_Dety;
double fx_gyro_fix, fy_gyro_fix;
u8 Optflow_Statemachine_cnt = 0;
Testime OPT_test_time;
Testime OPT_test_time_RD;
double Flow_Dt1 = 0;
_filter_1_st Acc_fx, Acc_fy;
Vector2f Optical_speed_error;
double OPT_fx, OPT_fy;  //光流线速度
double gyro_fx, gyro_fy;  //陀螺仪线速度
extern u8 Ux_Flag;
extern float DetX1, DetY1;

void Optflow_Statemachine(void)
{
		
/*****************************************光流位置预积分************************************************************/
		RESOLUTION = (1.0 / (10 * (1.0 / (Altitude_Estimate / 100.0f)))) * 2.54;//1英寸=2.54cm   1.0 / RESOLUTION = 一个像素点位移多少cm

		if (Ux_Flag)
		{
			//调整坐标系
			Real_DetX = -((float)DetX1 * RESOLUTION);		Real_DetY = ((float)DetY1 * RESOLUTION);
		}
		else {
			//调整坐标系
			Real_DetX = -((float)DetX * RESOLUTION);		Real_DetY = ((float)DetY * RESOLUTION);
		}

		OpticalFlow_Position.x += Real_DetX;  //*向右 Real_DetX 增大//单位CM
		OpticalFlow_Position.y += Real_DetY; //*向前 Real_DetY 增大

/*******************************************************************************************************************/
		Optflow_Statemachine_cnt++;
		if (Optflow_Statemachine_cnt >= 2)   // 1 2 3 4  //光流速度更新速度20ms   放慢一点更新速度 使速度更平滑
		{
			Optflow_Statemachine_cnt = 0;
			Test_Period(&OPT_test_time);
			Flow_Dt1 = OPT_test_time.Time_Delta / 1000.0;//两次处理光流数据的时间差   单位 秒
			
			////求取DT时间段内的位移 单位CM
			I_DetX = OpticalFlow_Position.x - Last_Sum_flow_x;   
			I_DetY = OpticalFlow_Position.y - Last_Sum_flow_y;
			Last_Sum_flow_x = OpticalFlow_Position.x;			Last_Sum_flow_y = OpticalFlow_Position.y;

			//求取光流旋转角速度  光流线速度  + 低通滤波  (cm/s)
			LPF_1_(40.0f, Flow_Dt1, ((double)I_DetX) / Flow_Dt1, OPT_fx);
			LPF_1_(40.0f, Flow_Dt1, ((double)I_DetY) / Flow_Dt1, OPT_fy);

			//求陀螺仪旋转线速度  陀螺仪速度  + 低通滤波  (cm/s)
			LPF_1_(20.0f, Flow_Dt1, -MRoll_Gyro * DEG2RAD * Altitude_Estimate, gyro_fx);
			LPF_1_(20.0f, Flow_Dt1, MPitch_Gyro * DEG2RAD * Altitude_Estimate, gyro_fy);

			//旋转补偿 速度cm/s  
			fx_gyro_fix = ((OPT_fx - LIMIT(gyro_fx, -150, 150)));
			fy_gyro_fix = ((OPT_fy - LIMIT(gyro_fy, -150, 150)));

	
			//加速度积分获取速度 低频信号         
			Acc_fx.out += (SINS_Accel_Body.x) * Flow_Dt1;     //速度cm/s
			Acc_fy.out += (SINS_Accel_Body.y) * Flow_Dt1;

			//光流的线速度与加速度积分获取的速度进行速度融合
			filter_1_ffcy(2.0f, 2.5f, Flow_Dt1, fx_gyro_fix, &Acc_fx);
			filter_1_ffcy(2.0f, 2.5f, Flow_Dt1, fy_gyro_fix, &Acc_fy);


			// 融合速度二次修正，最终输出结果 积分补偿消除稳态误差
			//P的修正
			OpticalFlow_Speed.x = Acc_fx.out + 0.1f * Optical_speed_error.x;   //输出速度量
			OpticalFlow_Speed.y = Acc_fy.out + 0.1f * Optical_speed_error.y;

			//I的修正  (fx_gyro_fix - f_out_x)以光流的线速度修正加速度得到的速度
			Optical_speed_error.x += (fx_gyro_fix - OpticalFlow_Speed.x) * Flow_Dt1;
			Optical_speed_error.y += (fy_gyro_fix - OpticalFlow_Speed.y) * Flow_Dt1;

			OpticalFlow_SINS.Velocity.y = OpticalFlow_Speed.y;
			OpticalFlow_SINS.Velocity.x = OpticalFlow_Speed.x;
			/****************************************位置部分*****************************************/

			//预积分位移旋转补偿

			OpticalFlow_SINS.Location.y = OpticalFlow_Position.y - (PI* Altitude_Estimate * MPitch * DEG2RAD / 3.0f);
			OpticalFlow_SINS.Location.x = OpticalFlow_Position.x - (-PI* Altitude_Estimate * MRoll * DEG2RAD / 3.0f);;

		}
}


extern OpticalFlowData optical_flow;
u8 Ux_Flag = 1;
float DetX1=0, DetY1=0;
void PMW3901_Data_Process(void)
{
	DMA_USART4_Data_Prase();

	if (Ux_Flag)
	{
		VL53Lx_Data.Distance = optical_flow.distance * 1000;
		DetX1 = -optical_flow.flow_x * 2;
		DetY1 = -optical_flow.flow_y * 2;
		//Optflow_Statemachine();						//光流状态机，初始化时存在光流外设
		Strapdown_INS_High_Kalman();				//卡尔曼滤波惯导融合

	}
	else if (VL53Lx_Data.Is_OK)
	{
		VL53LX_Statemachine();						//激光测距状态机
		PMW3901_Read_deltaXY(&DetX, &DetY);
		//Optflow_Statemachine();						//光流状态机，初始化时存在光流外设
		Strapdown_INS_High_Kalman();				//卡尔曼滤波惯导融合
	}

}

float radar_speed_x = 0;
float radar_speed_y = 0;
extern u8 Radar_New;

void Radar_StateMachine(void)
{
	static uint8_t radar_sm_cnt = 0;
	static _filter_1_st Acc_fx = { 0 }, Acc_fy = { 0 };
	static float radar_speed_error_x = 0.0f, radar_speed_error_y = 0.0f;
	static float radar_dt = 0.01f;
	static float radar_fx = 0.0f, radar_fy = 0.0f;
	static float fx_gyro_fix_rd = 0.0f, fy_gyro_fix_rd = 0.0f;
	static float gyro_fx_rd = 0.0f, gyro_fy_rd = 0.0f;  //陀螺仪线速度

	if (1)   // 每2次调用执行一次(10ms周期)
	{

		Test_Period(&OPT_test_time_RD);
		radar_dt = OPT_test_time_RD.Time_Delta / 1000.0;//两次处理光流数据的时间差   单位 秒

		//获取雷达原始速度并滤波
		LPF_1_(50.0f, radar_dt, ps_data.velocity.y, radar_fx);
		LPF_1_(50.0f, radar_dt, ps_data.velocity.x, radar_fy);

		//求陀螺仪旋转线速度  陀螺仪速度  + 低通滤波  (cm/s)
		LPF_1_(10.0f, radar_dt, MRoll_Gyro * DEG2RAD * 15, gyro_fx_rd);
		LPF_1_(10.0f, radar_dt, -MPitch_Gyro * DEG2RAD * 15, gyro_fy_rd);

		//旋转补偿 速度cm/s  
		fx_gyro_fix_rd = (radar_fx - LIMIT(gyro_fx_rd, -150, 150));
		fy_gyro_fix_rd = (radar_fy - LIMIT(gyro_fy_rd, -150, 150));

		//加速度积分
		Acc_fx.out += (SINS_Accel_Body.x) * radar_dt;
		Acc_fy.out += (SINS_Accel_Body.y) * radar_dt;

		//速度融合 - 使用正确的结构体指针
		filter_1_ffcy(1.0f, 5.0f, radar_dt, fx_gyro_fix_rd, &Acc_fx);
		filter_1_ffcy(1.0f, 5.0f, radar_dt, fy_gyro_fix_rd, &Acc_fy);

		//融合速度修正
		radar_speed_x = Acc_fx.out + 0.1 * radar_speed_error_x;
		radar_speed_y = Acc_fy.out + 0.1 * radar_speed_error_y;

		//误差累积
		radar_speed_error_x += (fx_gyro_fix_rd - radar_speed_x) * radar_dt;
		radar_speed_error_y += (fy_gyro_fix_rd - radar_speed_y) * radar_dt;

	}
}

#include "Headfile.h"
#include "FFCY_SDK.h"


#define SDK_WATE_TAKE_OFF	0
#define SDK_IN_Take_OFF		1
#define SDK_GET_UP			2
#define SDK_PosHode1		3
#define SDK_GET_UP_2		4

//移动
#define SDK_GO_target1		10
#define SDK_GO_target2		11
#define SDK_Throw1			12
#define SDK_Throw2			13
#define SDK_Recover1		14
#define SDK_Recover2		15
#define SDK_STAY1			16
#define SDK_STAY2			17
#define SDK_Sweep			18
#define SDK_Confirm			19

//追踪
#define SDK_Keep_Upon1		20
#define SDK_Keep_Upon2		21
#define SDK_Keep_Upon3		22
#define SDK_GO_target3		23
#define SDK_GO_target4		24
#define SDK_Pre_Upon		25

//转圈
#define SDK_YAW_TURN_LEFT		30
#define SDK_YAW_TURN_Right		31
#define SDK_Search_Pylon 		32
#define SDK_Circlre1		33
#define SDK_Circlre2		34
#define SDK_Circlre3		35
#define SDK_Circlre4		36
#define SDK_Circlre5		37
#define SDK_Circlre6		38
#define SDK_Circlre7		39

//钻圈
#define SDK_GO_Cross1		40
#define SDK_GO_Cross2		41
#define SDK_GO_Cross3		42
#define SDK_Search_Circle	43
#define SDK_Redress_Circle	34
#define	SDK_GO_Centre_Target	45
#define SDK_GO_Centre_Blind		46
#define SDK_Pre_Cross_Circle	47
#define SDK_Cross_Circle	48
#define SDK_Avoid_Circle	49

//找动物
#define SDK_Anm_Left1		50
#define SDK_Anm_Head1		51
#define SDK_Anm_Right1		52
#define SDK_Anm_Head2		53
#define SDK_Anm_Left2		54
#define SDK_Anm_Head3		55
#define SDK_Anm_Right2		56
#define SDK_Anm_Head4		57
#define SDK_Anm_Left3		58
#define SDK_Anm_Head5		59
#define SDK_Anm_Right3		60
#define SDK_Anm_Head6		61
#define SDK_Anm_Left4		62

//找动物new
#define SDK_Anm_Lesgo		70

//避障
#define SDK_Anm_Av1			80
#define SDK_Anm_Av2			81
#define SDK_Anm_Av3			82


//降落
#define SDK_GO_HOME			90
#define SDK_GET_DOWN		91
#define SDK_GET_DOWN_2		92
#define SDK_GET_DOWN_3		93
//等待
#define SDK_HANG			94
#define SDK_HANG1S			95 
#define SDK_STAY			96
#define SDK_T265_Gee		97

#define SDK_NONE			100

int  Search_buff[50][4] =
{
	{ -80,	0,	0,		2 },
	{ -80,	0,	-10,	2 },
	{ 0,	0,	-10,	2 },
	{ 0,	0,	-20,	2 },
	{ -80,	0,	-20,	2 },
	{ -80,	0,	-30,	2 },
	{ 0,	0,	-30,	2 },
	{ 0,	10,	-30,	2 },

	{ -80,	10,	0,		2 },
	{ -80,	10,	-10,	2 },
	{ 0,	10,	-10,	2 },
	{ 0,	10,	-20,	2 },
	{ -80,	10,	-20,	2 },
	{ -80,	10,	-30,	2 },
	{ 0,	10,	-30,	2 },
	{ 0,	20,	-30,	2 },

	{ -80,	20,	0,		2 },
	{ -80,	20,	-10,	2 },
	{ 0,	20,	-10,	2 },
	{ 0,	20,	-20,	2 },
	{ -80,	20,	-20,	2 },
	{ -80,	20,	-30,	2 },
	{ 0,	20,	-30,	2 },
	{ 0,	30,	-30,	2 },

	{ -80,	30,	0,		2 },
	{ -80,	30,	-10,	2 },
	{ 0,	30,	-10,	2 },
	{ 0,	30,	-20,	2 },
	{ -80,	30,	-20,	2 },
	{ -80,	30,	-30,	2 },
	{ 0,	30,	-30,	2 },
	{ 0,	40,	-30,	2 },

	{ -80,	40,	0,		2 },
	{ -80,	40,	-10,	2 },
	{ 0,	40,	-10,	2 },
	{ 0,	40,	-20,	2 },
	{ -80,	40,	-20,	2 },
	{ -80,	40,	-30,	2 },
	{ 0,	40,	-30,	2 },
	{ 0,	50,	-30,	2 },

	{ -80,	50,	0,		2 },
	{ -80,	50,	-10,	2 },
	{ 0,	50,	-10,	2 },
	{ 0,	50,	-20,	2 },
	{ -80,	50,	-20,	2 },
	{ -80,	50,	-30,	2 },
	{ 0,	50,	-30,	2 },
	{ 0,	0,	 0,	    2 },
	{ 0,	0,	 0,	    2 },
	{ 0,	0,	 0,	    2 },

};
int  Search_Circle_Buff[10][3] =
{
//		X		 Y		   Sec
	{ -100,		  0,		5  },
	{ -100,		100,		5  },
	{  100,		100,		10 },
	{  100,		200,		5  },
	{ -100,		200,		10 },
	{ -100,		300,		5  },
	{  100,		300,		10 },
	{  100,		400,		5  },
	{ -100,		400,		10 },
};
int  Search_Location_Buff[20][3] =
{
//		X		 Y		   Sec
	{   0,		380,		20 },
	{ -120,		380,		10 },
	{ -120,		 20,		20 },
	{ -260,		 20,		15 },
	{ -260,		380,		20 },
	{ 0,		  0,		30 },


	//{  350,		  0,		20 },
	//{  350,		100,		10 },
	//{    0,		100,		20 },
	//{    0,		300,		10 },
	//{  350,		300,		20 },
	//{    0,		  0,		20 },
};
int  Search_Pylon_Buff[10][4] =
{
//		X		 Y		   Sec
	{ -200,		  0,		5 },
	{ -200,		 50,		5 },
	{    0,		 50,		5 },
	{    0,		100,		5 },
	{ -200,		100,		5 },
	{ -200,		150,		5 },
	{    0,		150,		5 },
	{    0,		200,		5 },
	{ -200,		200,		5 },
};

float the_longth[20] = { -215.0f,65.0f,-50.0f,325.0f,55.0f,-280.0f,60.0f,280.0f,60.0f,-170.0f,55.0f,170.0f,55.0f,-380.0f };
float Y_Deliverer[5] = { -20, 55,  130, 205, 280 };
float X_Deliverer[5] = { 50,  125, 200, 275, 350 };
float Toward_Circle = 200.0;
float Corss_Circle = 50.0;

//#define fly_hight 150
#define long_line 50 
#define long_line_one 150  
#define long_line_two 100
#define long_line_three 100
#define long_line_four 100
#define Possion_error_limit 10.0f
#define Possion_error_limit_Normal 7.0f
#define Possion_error_limit_Task 2.0f
float Throttle_Slow_Up = Thr_Idle; //缓和启动油门

u8 SDK_SPECIAL_IN = 0;
u8 SDK_SPECIAL_IN_2 = 0;
u8 SDK_SPECIAL_IN_3 = 0;
u8 SDK_SPECIAL_IN_4 = 0;
u8  SDK_State_Flag = 0;
u8 SDK_Get_NowposionFlag = 0;
u8 high_expect_record = 0;
u8 SDK_pos_y_now;
u8 SDK_Machine_FIN_Flag = 0;
u8 MYSDK_Yaw_Control_flag = 0;
u8 bling_flag = 0;
u8 wait_to_bling = 0;
u8 Search_Targ_Roll_step = 0;
u8 circle_cnt = 0;
u8 circle_flag = 0;
u8 circle_stage = 0;
u8 circle_move_flag = 0;
u8 circle_turn_flag = 0;
u8 Search_Circle_Step = 0;
u8 Search_Circle_Overdue = 0;
u8 Upon_Step = 0;
u8 Color_Number = 0;
u8 Circle_Tar_Flag = 0;
u8 Circle_Gooooo = 0;
u8 Been_Flag = 0;
extern u8 T265_Confidence;
extern u8 XY_Move_flag;

u16 bling_times_2 = 0;
u16 bling_count = 0;
u16 SDK_Y_err;
u16 MYSDK_State = 0;
u16 MYSDK_State_last = 0;
u16 MYSDK_State_Record = 0;
u16 MYSDK_State_danger_Record = 0;
u16  SDK_HIGHT_CNT_L = 2;
u16 get_up_time = 0;
u16 fly_circle_time = 0;
u16 start_circle_time = 0;
u16 get_dowm_time = 0;
u16 SDK_GO_BACK_1_times = 0;
extern u16 Camera_data1, Camera_data2, Camera_data3, Camera_data4;
u32 SDK_TIME_CND = 0;
u32 Shangsuo_Cnd = 0;
u32 Fire_SDK_Record = 0;

int Delivery_Stage = 0;
int fly_hight_hight = 110;
int fly_hight = 60;
int each_place_laser[50];
int the_place_laser_count = 0;
int the_danger_place = 0;
int the_plane_place = 0;
int the_number = 0;
int the_new_number_last = 0;
int bling_times = 0;
int First_Loc_Number = 0, Second_Loc_Number = 0, Third_Loc_Number = 0;
extern int Openmv_Number;

float MYSDK_Target_Yaw_Gyro = 0;
float  MYSDK_YAW_TARG = 0;
float  MUSDK_YAW_Record = 0;
float Default_speed_out = 50.0f;
float Slow_speed_out = 10.0f;
float Slave_speed_out = 5.0f;
float Start_x_Record = 0;
float Start_y_Record = 0;
float First_Loc_Y, First_Loc_X, Second_Loc_Y, Second_Loc_X, Third_Loc_Y, Third_Loc_X;
float Yaw_Except = 0.0;
float Move_X = 0.0, Move_Z = 0.0;
float Rectify_X = 0.0, Rectify_Y = 0.0;
float High_Position_Record = 0.0;
float FL_X = 0.0;
float FL_Y = 0.0;
float Openmv_Correct = 20.0f;

float Lonth_Head = 380.0f;
float Lonth_Left1 = -80.0f;
float Lonth_Back = 0.0f;
float Lonth_Left2 = -160.0f;
float Lonth_Head2 = 380.0f;
float Lonth_Left3 = -240.0f;
float Lonth_Back2 = 0.0f;
float Lonth_Left4 = -320.0f;
float Lonth_Head3 = 380.0f;
float UWB_I = 1.0f;
float UWB_Err = 0.0f;
float UWB_Err_X = 0;
float UWB_Err_Y = 0;
float Fire_UWB_Y[10] = { 0.0f,80.0f,160.0f,240.0f,320.0f,400.0f };
float Fire_UWB_X[10] = { 0.0f,400.0f,240.0f,320.0f,400.0f,480.0f };
extern float New_Yaw_Posion;

Vector2f SDK_OUT_SPEED = { 0,0 };
Vector2f SDK_OUT_Position = { 0,0 };
Vector2f SDK_Control_Position = { 0,0 };
Vector2f RECORD_SDK_Control_Position = { 0,0 };
//Vector2f SDK_Control_Speed = { 0,0 };
Vector2f SDK_Control_Speed = { 0,0 };
Vector3f SDK_Record_POSITION = { 0,0 };
Vector2f SDK_GO_HAADE_LONGTH = { 0,0 };
Vector3f SDK_Home_POSITION = { 0,0 };
Vector3f SDK_Zero_POSITION = { 0,0 };

u8 Pos_Ge[10][10] = { 0 };
u8 Anm_Ge[10][10] = { 0 };
u8 Anm_Flag = 0;
u8 Src_Anm_Flag = 0;
u8 Anm_Finsh = 0;
u8 Targ_Stage = 0;
u8 Just_Dui = 0;
u8 Targ_Stage_last = 0;
u8 Targ_State_Flag = 0;
u16 saomiaocnd = 0;
extern u8 Gezi_x;
extern u8 Gezi_y;
extern u8 U6_Send_Buff[10];
extern u8 Anml_Num;
extern u8 Fst_Anml;
extern float Fst_Anml_X;
extern float Fst_Anml_Y;
extern u8 Sec_Anml;
extern float Sec_Anml_X;
extern float Sec_Anml_Y;
extern u8 Thr_Anml;
extern float Thr_Anml_X;
extern float Thr_Anml_Y;
u8 Again_Flag = 0;
u8 Which_Anm = 1;
extern u16 Lu_Cnd;
extern float Targ_Pos[3][30];
u8 Rec_Anml_Num = 0;
u8 Rec_Fst_Anml = 0;
float Rec_Fst_Anml_X = 0;
float Rec_Fst_Anml_Y = 0;
u8 Rec_Sec_Anml = 0;
float Rec_Sec_Anml_X = 0;
float Rec_Sec_Anml_Y = 0;
u8 Rec_Thr_Anml = 0;
float Rec_Thr_Anml_X = 0;
float Rec_Thr_Anml_Y = 0;
extern u8 Of_Begin_Flag;
extern u8 Gezi_x_Lst;
extern u8 Gezi_y_Lst;
extern u8 Gezi_y_Lst;
extern u8 Gezi_CF;
extern u8 Send_Anml_Num;
extern u8 U6_Send_Stage;
extern u8 EW_Send;
extern u8 EW_Anm_Kind;
extern u8 EW_Send_Anml_Num;
extern u8 Anm_Kind;
extern u8 Gezi_x_Rec;
extern u8 Gezi_y_Rec;
extern u8 Next_Tar_Flag;
extern u8 BackRoad_Flag;
extern u8 Get_Camera_Flag;
extern u8 Rec_Anm_Gx[10];
extern u8 Rec_Anm_Gy[10];
u8 Now_Gez[3] = { 0 };
#define Gez_X  1
#define Gez_Y  2
float Cnter_Gz[3] = { 0 };
extern void Anti_Where_RU_X(u8 gez_x, u8 gez_y);
extern float X_bf[75];
extern float Y_bf[75];
extern void Where_RU(float pos_x, float pos_y, u8 i);
float Back_XZ = 5;
float Anm_GoK = 1.1;
float BSpeed_Cor = 5;
u8 Been_Traverse[10][10] = { 0 };

void MY_SDK_REST()
{
	SDK_TIME_CND = 0;
	MYSDK_State = SDK_WATE_TAKE_OFF;
}

void SDK_SPEED_output(float Sx, float Sy)
{
	SDK_OUT_SPEED.x = Sx;
	SDK_OUT_SPEED.y = Sy;
}

void SDK_Posion_output(float Sx, float Sy)
{
	SDK_OUT_Position.x = Sx;
	SDK_OUT_Position.y = Sy;
}

void SDK_Get_Nowposion(float *Px, float *Py)
{

	*Px = PosSenser_SINS.Location.x;
	*Py = PosSenser_SINS.Location.y;
}

void SDK_Turnto_HANG1s(u16  NowSTATE)
{
	MYSDK_State_Record = NowSTATE;
	MYSDK_State = SDK_HANG1S;
	SDK_TIME_CND = 0;
}

void Let_s_Try(void)
{

	Gezi_x_Rec = Now_Gez[Gez_X];
	Gezi_y_Rec = Now_Gez[Gez_Y];

	//Gezi_x_Rec = Gezi_x;
	//Gezi_y_Rec = Gezi_y;

	if ((Rec_Anm_Gx[1] == Gezi_x_Rec) && (Rec_Anm_Gy[1] == Gezi_y_Rec))
	{
		Rec_Fst_Anml_X = Anm_GoK * Fst_Anml_X;
		Rec_Fst_Anml_Y = Anm_GoK * Fst_Anml_Y;
		Rec_Fst_Anml = Fst_Anml;
	}
	else
	{
		Rec_Fst_Anml = 0;
		Rec_Fst_Anml_X = 0;
		Rec_Fst_Anml_Y = 0;
	}

	if ((Rec_Anm_Gx[2] == Gezi_x_Rec) && (Rec_Anm_Gy[2] == Gezi_y_Rec))
	{
		Rec_Sec_Anml_X = Anm_GoK * Sec_Anml_X;
		Rec_Sec_Anml_Y = Anm_GoK * Sec_Anml_Y;
		Rec_Sec_Anml = Sec_Anml;
	}
	else
	{
		Rec_Sec_Anml = 0;
		Rec_Sec_Anml_X = 0;
		Rec_Sec_Anml_Y = 0;
	}

	Rec_Anml_Num = 0;

	if (Rec_Fst_Anml)		Rec_Anml_Num++;
	if (Rec_Sec_Anml)		Rec_Anml_Num++;

	Which_Anm = Rec_Anml_Num;

	if ((Rec_Fst_Anml == Rec_Sec_Anml) && Rec_Fst_Anml)		Send_Anml_Num = 2;
	else		Send_Anml_Num = 1;

	Anm_Kind = Rec_Fst_Anml;

	if (Send_Anml_Num < Rec_Anml_Num)
	{
		EW_Send = 1;
		EW_Send_Anml_Num = 1;
		EW_Anm_Kind = Rec_Sec_Anml;
	}
	else
	{
		EW_Send = 0;
		EW_Send_Anml_Num = 0;
		EW_Anm_Kind = 0;
	}
}

void SDK_Go_Pos(float Pos_X, float Pos_Y)
{
	if (Targ_State_Flag == 1) 
	{
		RECORD_SDK_Control_Position.y = PosSenser_SINS.Location.y;
		RECORD_SDK_Control_Position.x = PosSenser_SINS.Location.x;
		//前向
		SDK_Control_Position.y = SDK_Zero_POSITION.y + Pos_Y;    //期望位置Y
		SDK_Control_Speed.y = Default_speed_out;      //速度Y限幅
		//右向
		SDK_Control_Position.x = SDK_Zero_POSITION.x + Pos_X; //期望位置X
		SDK_Control_Speed.x = Default_speed_out;    //速度X限幅
	}

	//*******************走了50cm********************************/// 
	//if ((Targ_Pos[2][Targ_Stage] == 0) && Gezi_CF)
	if (Gezi_CF)
	{
		SDK_TIME_CND = 0;
		Src_Anm_Flag = 1;
		//Gezi_CF = 0;
		//if (ABS(PosSenser_SINS.Location.x - RECORD_SDK_Control_Position.x) > 25)
		//{
		//	RECORD_SDK_Control_Position.x = PosSenser_SINS.Location.x;
		//	SDK_TIME_CND = 0;
		//	Src_Anm_Flag = 1;
		//}
		//else if (ABS(PosSenser_SINS.Location.y - RECORD_SDK_Control_Position.y) > 25)
		//{
		//	RECORD_SDK_Control_Position.y = PosSenser_SINS.Location.y;
		//	SDK_TIME_CND = 0;
		//	Src_Anm_Flag = 1;
		//}
	}

	//*******************位置到达判定********************************/
	if (SDK_Calcuate_Y(SDK_Control_Position.y) > -Possion_error_limit && SDK_Calcuate_Y(SDK_Control_Position.y) < Possion_error_limit
		&&SDK_Calcuate_X(SDK_Control_Position.x) > -Possion_error_limit && SDK_Calcuate_X(SDK_Control_Position.x) < Possion_error_limit)  //位置到达
	{
		Targ_Stage++;
		SDK_TIME_CND = 0;
		Src_Anm_Flag = 1;
		//if (Targ_Pos[2][Targ_Stage] == 0)
	}
}

float SDK_Calcuate_X(float NowXPOS)
{
	return PosSenser_SINS.Location.x - NowXPOS;
}

float SDK_Calcuate_Y(float NowYPOS)
{
	return PosSenser_SINS.Location.y - NowYPOS;
}

void SDK_Wait_For_T265(u16  NowSTATE)
{
	if (MYSDK_State != SDK_T265_Gee) 
		MYSDK_State_Record = NowSTATE;
	MYSDK_State = SDK_T265_Gee;
	SDK_TIME_CND = 0;
}

u8 MY_SDK_Yaw_WaitFinish(void)
{
	if (MYSDK_Target_Yaw_Gyro > 0)  //逆时针旋转  增大    180---->-180
	{
		if (MYSDK_YAW_TARG > 0 && Yaw > 0)   if (Yaw > MYSDK_YAW_TARG)  MYSDK_Yaw_Control_flag = 0;
		if (MYSDK_YAW_TARG < 0 && Yaw < 0)   if (Yaw > MYSDK_YAW_TARG)  MYSDK_Yaw_Control_flag = 0;
	}
	else if (MYSDK_Target_Yaw_Gyro < 0)  //顺时针旋转    -180-----> 180
	{
		if (MYSDK_YAW_TARG > 0 && Yaw > 0)   if (Yaw < MYSDK_YAW_TARG)  MYSDK_Yaw_Control_flag = 0;
		if (MYSDK_YAW_TARG < 0 && Yaw < 0)   if (Yaw < MYSDK_YAW_TARG)  MYSDK_Yaw_Control_flag = 0;
	}
	return MYSDK_Yaw_Control_flag;
}

void MY_SDK_Yaw_Calcuate(float MYSDK_YAW_TARG_Deta, float GYRO_SPEED)
{
	MUSDK_YAW_Record = Yaw;
	MYSDK_YAW_TARG = Yaw + MYSDK_YAW_TARG_Deta;
	if (MYSDK_YAW_TARG > 180)	MYSDK_YAW_TARG = MYSDK_YAW_TARG - 360;
	if (MYSDK_YAW_TARG < -180)	MYSDK_YAW_TARG = MYSDK_YAW_TARG + 360;
	MYSDK_Target_Yaw_Gyro = GYRO_SPEED;
	MYSDK_Yaw_Control_flag = 1;
}


void FFCY_SDK_Control(uint8_t force_brake_flag)
{
	//无水平遥控量给定
	if (Roll_Control == 0 && Pitch_Control == 0)
	{
		//if (MYSDK_State == 0)  Target_Location();}
		SDK_Machine();

		//期望位置赋值
		OpticalFlow_Pos_Ctrl_Expect.x = SDK_OUT_Position.x;
		OpticalFlow_Pos_Ctrl_Expect.y = SDK_OUT_Position.y;
		//位置控制
		OpticalFlow_Pos_Control();
		//输出限幅
		if (OpticalFlow_Pos_Ctrl_Output.x >= SDK_OUT_SPEED.x)  OpticalFlow_Pos_Ctrl_Output.x = SDK_OUT_SPEED.x;
		if (OpticalFlow_Pos_Ctrl_Output.x <= -SDK_OUT_SPEED.x)  OpticalFlow_Pos_Ctrl_Output.x = -SDK_OUT_SPEED.x;
		if (OpticalFlow_Pos_Ctrl_Output.y >= SDK_OUT_SPEED.y)  OpticalFlow_Pos_Ctrl_Output.y = SDK_OUT_SPEED.y;
		if (OpticalFlow_Pos_Ctrl_Output.y <= -SDK_OUT_SPEED.y)  OpticalFlow_Pos_Ctrl_Output.y = -SDK_OUT_SPEED.y;

		OpticalFlow_Vel_Control(OpticalFlow_Pos_Ctrl_Output);//速度期望

		Total_Controller.Yaw_Angle_Control.Expect = Yaw_Except;
	}
	else
	{
		XY_Move_flag = 1;

		OpticalFlow_Pos_Ctrl_Output.x = OpticalFlow_Expect_Speed_Mapping(Target_Angle[1], Pit_Rol_Max, OpticalFlow_Speed_Control_Max);
		OpticalFlow_Pos_Ctrl_Output.y = OpticalFlow_Expect_Speed_Mapping(-Target_Angle[0], Pit_Rol_Max, OpticalFlow_Speed_Control_Max);

		OpticalFlow_Vel_Control(OpticalFlow_Pos_Ctrl_Output);//速度期望
		/***********当打杆不需要进行控速时 可以开启以下注释*************/
		//Total_Controller.Pitch_Angle_Control.Expect = Target_Angle[0];
		//Total_Controller.Roll_Angle_Control.Expect = Target_Angle[1];

		Total_Controller.Yaw_Angle_Control.Expect = Yaw_Except;
	}
}


//**************************飞**********凡*********创**********翼******************************************
//*	函 数 名 :  SDK_Machine()
//*	功能说明 :    SDK状态机
//*	形    参 ：无
//*	返 回 值 : 无
//* 作者    ：FFCY
//* 最后更改：CP_SYD 孙艺东 2019:8:8 16:38:14 
//**********************************(C)COPYRIGHT 2019 FFCY Team*********************************************/
void SDK_Machine(void)
{

	SDK_TIME_CND++;
	if (SDK_TIME_CND > 0xFFFFFFFA)		SDK_TIME_CND = 0;

	if (MYSDK_State != MYSDK_State_last)
		SDK_State_Flag = 1;
	else
		SDK_State_Flag = 0;

	MYSDK_State_last = MYSDK_State;

	//if (T265_Confidence < 3)		SDK_Wait_For_T265(MYSDK_State);



	switch (MYSDK_State)
	{
	case SDK_WATE_TAKE_OFF:		//怠速
	{
		if (SDK_TIME_CND >= 3 * 200)
		{
			MYSDK_State = SDK_IN_Take_OFF;
			SDK_TIME_CND = 0;
		}
		Throttle_Slow_Up = Thr_Idle;  //设置缓和启动油门
		SDK_WAIT_TO_FLY = 1;
	}	break;

	case SDK_IN_Take_OFF:		//一键起飞
	{
		if (SDK_State_Flag == 1)  //第一次进入该模式时
		{
			SDK_Get_Nowposion(&SDK_Zero_POSITION.x, &SDK_Zero_POSITION.y);
			//前向
			SDK_Control_Position.y = PosSenser_SINS.Location.y;    //期望位置Y
			SDK_Control_Speed.y = Default_speed_out;   //速度Y限幅
													   //右向
			SDK_Control_Position.x = PosSenser_SINS.Location.x; //期望位置X
			SDK_Control_Speed.x = Default_speed_out;  //速度X限幅

			High_Hold_Throttle = Throttle_Slow_Up;
			Take_Off_Reset();//清积分
		}

		/*****************起飞状态结束后开始悬停**************************/
		Take_Off_Reset();//清积分
		Throttle_Control_Reset();
		SDK_TIME_CND = 0;

		//一键起飞悬停高度设置
		Total_Controller.High_Position_Control.Expect = Altitude_Position + 2;

		SDK_WAIT_TO_FLY = 0;
		if (Throttle_Slow_Up < Thr_Hover_Default)
		{
			Throttle_Slow_Up = Throttle_Slow_Up + 0.5;
			High_Hold_Throttle = (int)Throttle_Slow_Up;

			Throttle_Control_Reset();
			OpticalFlow_Ctrl_Reset();
		}
		else
		{
			High_Hold_Throttle = Thr_Hover_Default;
			MYSDK_State = SDK_GET_UP;
			SDK_Turnto_HANG1s(MYSDK_State);
		}

		SDK_SPEED_output(SDK_Control_Speed.x, SDK_Control_Speed.y);
		SDK_Posion_output(SDK_Control_Position.x, SDK_Control_Position.y);
	}	break;

	case SDK_GET_UP:			//起飞爬升
	{
		if (SDK_State_Flag == 1)  //第一次进入该模式时
		{
			//前向
			SDK_Control_Position.y = SDK_Control_Position.y + 0.0f;    //期望位置Y
			SDK_Control_Speed.y = Default_speed_out;   //速度Y限幅
													   //右向
			SDK_Control_Position.x = SDK_Control_Position.x + 0.0f; //期望位置X
			SDK_Control_Speed.x = Default_speed_out;  //速度X限幅

			high_expect_record = Total_Controller.High_Position_Control.Expect;
		}

		if (Total_Controller.High_Position_Control.Expect <= high_expect_record + fly_hight_hight - 2)
		{
			get_up_time++;
			if (get_up_time >= 20)
			{
				get_up_time = 0;
				Total_Controller.High_Position_Control.Expect += 2;
			}

		}
		else 		MYSDK_State = SDK_PosHode1;

		SDK_SPEED_output(SDK_Control_Speed.x, SDK_Control_Speed.y);
		SDK_Posion_output(SDK_Control_Position.x, SDK_Control_Position.y);
	}	break;

	case SDK_PosHode1:			//爬升后悬停
	{
		if (SDK_State_Flag == 1)  //第一次进入该模式时
		{
			//前向
			SDK_Control_Position.y = SDK_Control_Position.y + 0.0f;    //期望位置Y
			SDK_Control_Speed.y = Default_speed_out;   //速度Y限幅
													   //右向
			SDK_Control_Position.x = SDK_Control_Position.x + 0.0f; //期望位置X
			SDK_Control_Speed.x = Default_speed_out;  //速度X限幅
			Total_Controller.High_Position_Control.Expect = fly_hight_hight;
		}
		if (SDK_TIME_CND >= 8 * 200)
		{
			SDK_TIME_CND = 0;
			//if(Next_Tar_Flag)	Targ_Stage = 1;
			//else	Targ_Stage = 0;
			//Targ_Stage_last = Targ_Stage;

			if (Delivery_Stage == 2)
			{
				MYSDK_State = SDK_GO_target3;
			}
			else if (Delivery_Stage == 1)
			{
				MYSDK_State = SDK_STAY1;
			}
			else
			{
				Targ_Stage = 0;
				MYSDK_State = SDK_Anm_Lesgo;
			}

		}

		SDK_SPEED_output(SDK_Control_Speed.x, SDK_Control_Speed.y);
		SDK_Posion_output(SDK_Control_Position.x, SDK_Control_Position.y);
	}	break;

	case SDK_Anm_Lesgo:
	{
		//该去下一个点了
		if (Targ_Stage != Targ_Stage_last || Just_Dui)
		{
			Just_Dui = 0;
			Targ_State_Flag = 1;
		}
		else
			Targ_State_Flag = 0;

		Targ_Stage_last = Targ_Stage;

		//if (Src_Anm_Flag)	LASER1_flag = 1;

		//开始扫描
		if (Anml_Num && Of_Begin_Flag)
		{
			Src_Anm_Flag = 0;
			Gezi_CF = 0;
			memset(Rec_Anm_Gx, 0, sizeof(Rec_Anm_Gx));
			memset(Rec_Anm_Gy, 0, sizeof(Rec_Anm_Gy));
			Where_RU(X_bf[1], Y_bf[1], 1);
			Where_RU(X_bf[2], Y_bf[2], 2);
			Where_RU(X_bf[3], Y_bf[3], 3);

			if ((((Rec_Anm_Gx[1] == Gezi_x) && (Rec_Anm_Gy[1] == Gezi_y))
				|| ((Rec_Anm_Gx[2] == Gezi_x) && (Rec_Anm_Gy[2] == Gezi_y))
				|| ((Rec_Anm_Gx[3] == Gezi_x) && (Rec_Anm_Gy[3] == Gezi_y)))
				&& Been_Traverse[Gezi_x][Gezi_y] == 0)
			{
				Been_Traverse[Gezi_x][Gezi_y] = 1;
				Now_Gez[Gez_X] = Gezi_x;
				Now_Gez[Gez_Y] = Gezi_y;
				Anti_Where_RU_X(Now_Gez[Gez_X], Now_Gez[Gez_Y]);
				MYSDK_State = SDK_Pre_Upon;
				Just_Dui = 1;
			}
			else
			{
				Just_Dui = 0;
			}

		}	

		//打点
		SDK_Go_Pos(Targ_Pos[0][Targ_Stage], Targ_Pos[1][Targ_Stage]);

		//到达终点
		if (Targ_Stage >= Lu_Cnd)
		{
			Shangsuo_Cnd = 0;
			SDK_TIME_CND = 0;
			if(BackRoad_Flag)	MYSDK_State = SDK_GO_target1;
			else				MYSDK_State = SDK_GO_target3;
		}

		SDK_SPEED_output(SDK_Control_Speed.x, SDK_Control_Speed.y);
		SDK_Posion_output(SDK_Control_Position.x, SDK_Control_Position.y);
	}	break;

	case SDK_Pre_Upon:
	{
		if (SDK_State_Flag == 1)  //第一次进入该模式时
		{
			//前向
			SDK_Control_Position.y = SDK_Zero_POSITION.y + Cnter_Gz[Gez_Y];    //期望位置Y
			SDK_Control_Speed.y = Default_speed_out;      //速度Y限幅
														  //右向
			SDK_Control_Position.x = SDK_Zero_POSITION.x + Cnter_Gz[Gez_X]; //期望位置X
			SDK_Control_Speed.x = Default_speed_out;    //速度X限幅
			Total_Controller.High_Position_Control.Expect = fly_hight_hight;
		}

		//*******************位置到达判定********************************/
		if (SDK_Calcuate_Y(SDK_Control_Position.y) > -Possion_error_limit && SDK_Calcuate_Y(SDK_Control_Position.y) < Possion_error_limit
			&&SDK_Calcuate_X(SDK_Control_Position.x) > -Possion_error_limit && SDK_Calcuate_X(SDK_Control_Position.x) < Possion_error_limit)  //位置到达
		{
			Get_Camera_Flag = 1;
			SDK_Get_Nowposion(&SDK_Record_POSITION.x, &SDK_Record_POSITION.y);
			MYSDK_State = SDK_Keep_Upon1;
		}

		SDK_SPEED_output(SDK_Control_Speed.x, SDK_Control_Speed.y);
		SDK_Posion_output(SDK_Control_Position.x, SDK_Control_Position.y);
	}	break;

	case SDK_Keep_Upon1:
	{
		if (SDK_State_Flag == 1 || Again_Flag)  //第一次进入该模式时
		{
			Again_Flag = 0;
			SDK_TIME_CND = 0;

			if (Which_Anm == 2)
			{
				//前向
				SDK_Control_Position.y = SDK_Record_POSITION.y + Rec_Sec_Anml_Y;    //期望位置Y
				SDK_Control_Speed.y = Default_speed_out;      //速度Y限幅
															  //右向
				SDK_Control_Position.x = SDK_Record_POSITION.x + Rec_Sec_Anml_X; //期望位置X
				SDK_Control_Speed.x = Default_speed_out;    //速度X限幅
			}
			else if (Which_Anm == 3)
			{
				//前向
				SDK_Control_Position.y = SDK_Record_POSITION.y + Rec_Thr_Anml_Y;    //期望位置Y
				SDK_Control_Speed.y = Default_speed_out;      //速度Y限幅
															  //右向
				SDK_Control_Position.x = SDK_Record_POSITION.x + Rec_Thr_Anml_X; //期望位置X
				SDK_Control_Speed.x = Default_speed_out;    //速度X限幅
			}
			else if (Which_Anm == 1)
			{
				//前向
				SDK_Control_Position.y = SDK_Record_POSITION.y + Rec_Fst_Anml_Y;    //期望位置Y
				SDK_Control_Speed.y = Default_speed_out;      //速度Y限幅
															  //右向
				SDK_Control_Position.x = SDK_Record_POSITION.x + Rec_Fst_Anml_X; //期望位置X
				SDK_Control_Speed.x = Default_speed_out;    //速度X限幅
			}
			else
			{
				Which_Anm = 1;
				MYSDK_State = SDK_Anm_Lesgo;
			}

			Total_Controller.High_Position_Control.Expect = fly_hight_hight;
		}

		if (SDK_TIME_CND > 400)
		{
			BEEP_flag = 1;
			SDK_TIME_CND = 0;
			if (Which_Anm > 1)
			{
				Again_Flag = 1;
				Which_Anm--;
			}
			else
			{
				Which_Anm = 1;
				Again_Flag = 0;
				MYSDK_State = SDK_Anm_Lesgo;
				U6_Send_Stage = 1;
			}
		}

		SDK_SPEED_output(SDK_Control_Speed.x, SDK_Control_Speed.y);
		SDK_Posion_output(SDK_Control_Position.x, SDK_Control_Position.y);
	}	break;

	case SDK_Recover1:
	{
		if (SDK_State_Flag == 1)  //第一次进入该模式时
		{
			//前向
			SDK_Control_Position.y = SDK_Zero_POSITION.y + Cnter_Gz[Gez_Y];    //期望位置Y
			SDK_Control_Speed.y = Default_speed_out;      //速度Y限幅
														  //右向
			SDK_Control_Position.x = SDK_Zero_POSITION.x + Cnter_Gz[Gez_X]; //期望位置X
			SDK_Control_Speed.x = Default_speed_out;    //速度X限幅
			high_expect_record = Total_Controller.High_Position_Control.Expect;
		}

		//*******************位置到达判定********************************/
		if (SDK_Calcuate_Y(SDK_Control_Position.y) > -Possion_error_limit && SDK_Calcuate_Y(SDK_Control_Position.y) < Possion_error_limit
			&&SDK_Calcuate_X(SDK_Control_Position.x) > -Possion_error_limit && SDK_Calcuate_X(SDK_Control_Position.x) < Possion_error_limit)  //位置到达
		{
			MYSDK_State = SDK_Anm_Lesgo;
		}

		SDK_SPEED_output(SDK_Control_Speed.x, SDK_Control_Speed.y);
		SDK_Posion_output(SDK_Control_Position.x, SDK_Control_Position.y);
	}	break;

	case SDK_GO_target1:
	{
		if (SDK_State_Flag == 1)  //第一次进入该模式时
		{
			//前向
			SDK_Control_Position.y = SDK_Zero_POSITION.y + 0.0f;    //期望位置Y
			SDK_Control_Speed.y = Default_speed_out + 50.0f;      //速度Y限幅
														  //右向
			SDK_Control_Position.x = SDK_Zero_POSITION.x - 400.0f; //期望位置X
			SDK_Control_Speed.x = Default_speed_out + 50.0f;    //速度X限幅
			Total_Controller.High_Position_Control.Expect = fly_hight_hight;
		}

		//*******************位置到达判定********************************/
		if (SDK_Calcuate_Y(SDK_Control_Position.y) > -Possion_error_limit && SDK_Calcuate_Y(SDK_Control_Position.y) < Possion_error_limit
			&&SDK_Calcuate_X(SDK_Control_Position.x) > -Possion_error_limit && SDK_Calcuate_X(SDK_Control_Position.x) < Possion_error_limit)  //位置到达
		{
			MYSDK_State = SDK_GO_target2;
		}

		SDK_SPEED_output(SDK_Control_Speed.x, SDK_Control_Speed.y);
		SDK_Posion_output(SDK_Control_Position.x, SDK_Control_Position.y);
	}	break;

	case SDK_GO_target2:
	{
		if (SDK_State_Flag == 1)  //第一次进入该模式时
		{
			//前向
			SDK_Control_Position.y = SDK_Zero_POSITION.y + 0.0f;    //期望位置Y
			SDK_Control_Speed.y = Default_speed_out + 50.0f;      //速度Y限幅
														  //右向
			SDK_Control_Position.x = SDK_Zero_POSITION.x - 110.0f; //期望位置X
			SDK_Control_Speed.x = Default_speed_out + 50.0f;    //速度X限幅
			Total_Controller.High_Position_Control.Expect = fly_hight_hight;
		}

		//*******************位置到达判定********************************/
		if (SDK_Calcuate_Y(SDK_Control_Position.y) > -Possion_error_limit && SDK_Calcuate_Y(SDK_Control_Position.y) < Possion_error_limit
			&&SDK_Calcuate_X(SDK_Control_Position.x) > -Possion_error_limit && SDK_Calcuate_X(SDK_Control_Position.x) < Possion_error_limit)  //位置到达
		{
			MYSDK_State = SDK_GET_DOWN_3;
			SDK_Turnto_HANG1s(MYSDK_State);
		}

		SDK_SPEED_output(SDK_Control_Speed.x, SDK_Control_Speed.y);
		SDK_Posion_output(SDK_Control_Position.x, SDK_Control_Position.y);

	}	break;

	case SDK_GO_target3:
	{
		if (SDK_State_Flag == 1)  //第一次进入该模式时
		{
			//前向
			SDK_Control_Position.y = SDK_Zero_POSITION.y + 300.0f;    //期望位置Y
			SDK_Control_Speed.y = Default_speed_out + 50.0f;      //速度Y限幅
														  //右向
			SDK_Control_Position.x = SDK_Zero_POSITION.x + 0.0f; //期望位置X
			SDK_Control_Speed.x = Default_speed_out + 50.0f;    //速度X限幅
			Total_Controller.High_Position_Control.Expect = fly_hight_hight;
		}

		//*******************位置到达判定********************************/
		if (SDK_Calcuate_Y(SDK_Control_Position.y) > -Possion_error_limit && SDK_Calcuate_Y(SDK_Control_Position.y) < Possion_error_limit
			&&SDK_Calcuate_X(SDK_Control_Position.x) > -Possion_error_limit && SDK_Calcuate_X(SDK_Control_Position.x) < Possion_error_limit)  //位置到达
		{
			MYSDK_State = SDK_STAY;
		}

		SDK_SPEED_output(SDK_Control_Speed.x, SDK_Control_Speed.y);
		SDK_Posion_output(SDK_Control_Position.x, SDK_Control_Position.y);
	}	break;

	case SDK_GO_target4:
	{
		if (SDK_State_Flag == 1)  //第一次进入该模式时
		{
			//前向
			SDK_Control_Position.y = SDK_Zero_POSITION.y + 110.0f;    //期望位置Y
			SDK_Control_Speed.y = Default_speed_out + 50.0f;      //速度Y限幅
														  //右向
			SDK_Control_Position.x = SDK_Zero_POSITION.x + 0.0f; //期望位置X
			SDK_Control_Speed.x = Default_speed_out + 50.0f;    //速度X限幅
			Total_Controller.High_Position_Control.Expect = fly_hight_hight;
		}

		//*******************位置到达判定********************************/
		if (SDK_Calcuate_Y(SDK_Control_Position.y) > -Possion_error_limit && SDK_Calcuate_Y(SDK_Control_Position.y) < Possion_error_limit
			&&SDK_Calcuate_X(SDK_Control_Position.x) > -Possion_error_limit && SDK_Calcuate_X(SDK_Control_Position.x) < Possion_error_limit)  //位置到达
		{
			MYSDK_State = SDK_GET_DOWN_3;
			SDK_Turnto_HANG1s(MYSDK_State);
		}

		SDK_SPEED_output(SDK_Control_Speed.x, SDK_Control_Speed.y);
		SDK_Posion_output(SDK_Control_Position.x, SDK_Control_Position.y);

	}	break;

	case SDK_GET_DOWN_3:
	{
		Shangsuo_Cnd++;
		if (SDK_State_Flag == 1)  //第一次进入该模式时
		{
			Anm_Led_flag = 1;
			BEEP_flag = 1;
			if (BackRoad_Flag)
			{
				SDK_Control_Position.y = SDK_Zero_POSITION.y + 0.0f;    //期望位置Y
				SDK_Control_Position.x = SDK_Zero_POSITION.x + Back_XZ; //期望位置X
			}
			else
			{
				SDK_Control_Position.y = SDK_Zero_POSITION.y - Back_XZ;    //期望位置Y
				SDK_Control_Position.x = SDK_Zero_POSITION.x + 0.0f; //期望位置X
			}

			SDK_Control_Speed.y = Default_speed_out - BSpeed_Cor;      //速度Y限幅
			SDK_Control_Speed.x = Default_speed_out - BSpeed_Cor;	  //速度X限幅
			high_expect_record = Total_Controller.High_Position_Control.Expect;
		}

		if (SDK_TIME_CND >= 20)
		{
			SDK_TIME_CND = 0;
			Total_Controller.High_Position_Control.Expect = Total_Controller.High_Position_Control.Expect - 2;

			if ((Total_Controller.High_Position_Control.Expect <= ((high_expect_record - fly_hight_hight) + 5)
				&& Altitude_Position <= ((high_expect_record - fly_hight_hight) + 8)
				&& Controler_State != Lock_Controler)
				|| Shangsuo_Cnd > 15 * 200)
			{
				Controler_State = Lock_Controler;
				MYSDK_State = SDK_NONE;
				LASER1_flag = 2;
				BEEP_flag = 1;
				Anm_Led_flag = 0;
			}
		}

		SDK_SPEED_output(SDK_Control_Speed.x, SDK_Control_Speed.y);
		SDK_Posion_output(SDK_Control_Position.x, SDK_Control_Position.y);
	}	break;

	case SDK_STAY1:
	{
		SDK_SPEED_output(10, 10);
		SDK_Posion_output(SDK_Control_Position.x, SDK_Control_Position.y);

		if (SDK_TIME_CND >= 10 * 200)
		{

			MYSDK_State = SDK_GET_DOWN_2;
			SDK_Turnto_HANG1s(MYSDK_State);
			SDK_TIME_CND = 0;
			/*	MYSDK_State = SDK_HANG;*/
		}
	}	break;

	case SDK_HANG1S:
	{
		SDK_SPEED_output(10, 10);
		SDK_Posion_output(SDK_Control_Position.x, SDK_Control_Position.y);
		if (SDK_TIME_CND >= 1 * 200)
		{
			MYSDK_State = MYSDK_State_Record;
			SDK_TIME_CND = 0;
			/*	MYSDK_State = SDK_HANG;*/
		}
	}	break;

	case SDK_HANG:
	{
		SDK_TIME_CND = 0;
		SDK_OUT_SPEED.x = 0;
		SDK_OUT_SPEED.y = 0;
		SDK_Machine_FIN_Flag = 1;
	}	break;

	case SDK_T265_Gee:
	{
		if (SDK_State_Flag == 1)  //第一次进入该模式时
		{
			SDK_Get_Nowposion(&SDK_Record_POSITION.x, &SDK_Record_POSITION.y);
			//前向
			SDK_Control_Position.y = SDK_Record_POSITION.x;		//期望位置Y
			//右向
			SDK_Control_Position.x = SDK_Record_POSITION.y;		//期望位置X

			Total_Controller.High_Position_Control.Expect = fly_hight_hight;
		}

		if (T265_Confidence == 3)
		{
			MYSDK_State = MYSDK_State_Record;
			SDK_TIME_CND = 0;
		}

		SDK_SPEED_output(10, 10);
		SDK_Posion_output(SDK_Record_POSITION.x, SDK_Record_POSITION.y);
	}	break;

	case SDK_STAY:
	{
		SDK_SPEED_output(10, 10);
		SDK_Posion_output(SDK_Control_Position.x, SDK_Control_Position.y);

		if (SDK_TIME_CND >= 3 * 200)
		{

			MYSDK_State = SDK_GO_target4;
			SDK_Turnto_HANG1s(MYSDK_State);
			SDK_TIME_CND = 0;
			/*	MYSDK_State = SDK_HANG;*/
		}
	}	break;

	case SDK_GO_HOME:
	{
		if (SDK_State_Flag == 1)  //第一次进入该模式时
		{
			//前向
			SDK_Control_Position.y = 0.0f;    //期望位置Y
			SDK_Control_Speed.y = Default_speed_out;      //速度Y限幅
														  //右向
			SDK_Control_Position.x = 0.0f; //期望位置X
			SDK_Control_Speed.x = Default_speed_out;    //速度X限幅

			Total_Controller.High_Position_Control.Expect = fly_hight_hight;
		}
		//*******************位置到达判定********************************/
		if (SDK_Calcuate_Y(SDK_Control_Position.y) > -Possion_error_limit && SDK_Calcuate_Y(SDK_Control_Position.y) < Possion_error_limit
			&&SDK_Calcuate_X(SDK_Control_Position.x) > -Possion_error_limit && SDK_Calcuate_X(SDK_Control_Position.x) < Possion_error_limit)  //位置到达
		{
			MYSDK_State = SDK_GET_DOWN_2;
			SDK_Turnto_HANG1s(MYSDK_State);
		}
		SDK_SPEED_output(SDK_Control_Speed.x, SDK_Control_Speed.y);
		SDK_Posion_output(SDK_Control_Position.x, SDK_Control_Position.y);
	}	break;

	case SDK_GET_DOWN_2:
	{
		SDK_Get_Nowposion(&SDK_Record_POSITION.x, &SDK_Record_POSITION.y);
		if (SDK_State_Flag == 1)  //第一次进入该模式时
		{
			//前向
			SDK_Control_Position.y = SDK_Record_POSITION.y;    //期望位置Y
			SDK_Control_Speed.y = Default_speed_out;      //速度Y限幅
														  //右向
			SDK_Control_Position.x = SDK_Record_POSITION.x; //期望位置X
			SDK_Control_Speed.x = Default_speed_out;    //速度X限幅

			high_expect_record = Total_Controller.High_Position_Control.Expect;
		}

		if (SDK_TIME_CND >= 20)
		{
			SDK_TIME_CND = 0;
			Total_Controller.High_Position_Control.Expect = Total_Controller.High_Position_Control.Expect - 2;
			if (Total_Controller.High_Position_Control.Expect <= high_expect_record - fly_hight_hight 
				&& Altitude_Position <= high_expect_record - fly_hight_hight + 2 
				&& Controler_State != Lock_Controler)
			{
				Controler_State = Lock_Controler;
				MYSDK_State = SDK_NONE;
			}
		}

		SDK_SPEED_output(SDK_Control_Speed.x, SDK_Control_Speed.y);
		SDK_Posion_output(SDK_Control_Position.x, SDK_Control_Position.y);
	}	break;

	case SDK_NONE:
	{
		MYSDK_State = SDK_NONE;
	}	break;

	default:
		MYSDK_State = SDK_HANG;
		break;
	}
}


void Target_Location(void)
{
	if (Delivery_Stage == 1)
	{

	}
	else if (Delivery_Stage == 2)
	{
		if (Openmv_Number == 0)
		{
			First_Loc_Number = 0;
			Second_Loc_Number = 0;
		}
		else if (Openmv_Number == 1)
		{
			First_Loc_Number = 11;
			Second_Loc_Number = 12;
		}
		else if (Openmv_Number == 2)
		{
			First_Loc_Number = 7;
			Second_Loc_Number = 8;
		}
		else if (Openmv_Number == 3)
		{
			First_Loc_Number = 3;
			Second_Loc_Number = 4;
		}
		else if (Openmv_Number == 4)
		{
			First_Loc_Number = 9;
			Second_Loc_Number = 10;
		}
		else if (Openmv_Number == 5)
		{
			First_Loc_Number = 6;
			Second_Loc_Number = 5;
		}
		else if (Openmv_Number == 6)
		{
			First_Loc_Number = 1;
			Second_Loc_Number = 2;
		}
		else
		{
			First_Loc_Number = 0;
			Second_Loc_Number = 0;
		}
	}
	else if (Delivery_Stage == 3)
	{
		First_Loc_Number = 20;
		Second_Loc_Number = 20;
		Third_Loc_Number = 20;
	}
	else if (Delivery_Stage == 4)
	{
		First_Loc_Number = 21;
		Second_Loc_Number = 21;
		Third_Loc_Number = 21;
	}
	else if (Delivery_Stage == 5)
	{
		First_Loc_Number = 22;
		Second_Loc_Number = 22;
		Third_Loc_Number = 22;
	}
	else if (Delivery_Stage == 6)
	{
		First_Loc_Number = 23;
		Second_Loc_Number = 23;
		Third_Loc_Number = 23;
	}
	else
	{

	}


	switch (First_Loc_Number)
	{
	case 1:
		First_Loc_Y = Y_Deliverer[4];
		First_Loc_X = X_Deliverer[0];
		break;
	case 2:
		First_Loc_Y = Y_Deliverer[2];
		First_Loc_X = X_Deliverer[2];
		break;
	case 3:
		First_Loc_Y = Y_Deliverer[3];
		First_Loc_X = X_Deliverer[3];
		break;
	case 4:
		First_Loc_Y = Y_Deliverer[0];
		First_Loc_X = X_Deliverer[4];
		break;
	case 5:
		First_Loc_Y = Y_Deliverer[4];
		First_Loc_X = X_Deliverer[4];
		break;
	case 6:
		First_Loc_Y = Y_Deliverer[1];
		First_Loc_X = X_Deliverer[3];
		break;
	case 7:
		First_Loc_Y = Y_Deliverer[1];
		First_Loc_X = X_Deliverer[1];
		break;
	case 8:
		First_Loc_Y = Y_Deliverer[3];
		First_Loc_X = X_Deliverer[1];
		break;
	case 9:
		First_Loc_Y = Y_Deliverer[2];
		First_Loc_X = X_Deliverer[0];
		break;
	case 10:
		First_Loc_Y = Y_Deliverer[0];
		First_Loc_X = X_Deliverer[2];
		break;
	case 11:
		First_Loc_Y = Y_Deliverer[4];
		First_Loc_X = X_Deliverer[2];
		break;
	case 12:
		First_Loc_Y = Y_Deliverer[2];
		First_Loc_X = X_Deliverer[4];
		break;
	case 20:
		First_Loc_Y = Y_Deliverer[0];
		First_Loc_X = X_Deliverer[2];
		break;
	case 21:
		First_Loc_Y = Y_Deliverer[2];
		First_Loc_X = X_Deliverer[1];
		break;
	case 22:
		First_Loc_Y = Y_Deliverer[1];
		First_Loc_X = X_Deliverer[1];
		break;
	case 23:
		First_Loc_Y = Y_Deliverer[0];
		First_Loc_X = X_Deliverer[1];
		break;
	default:
		First_Loc_Y = 0.0f;
		First_Loc_X = 0.0f;
		break;
	}

	switch (Second_Loc_Number)
	{
	case 1:
		Second_Loc_Y = Y_Deliverer[4];
		Second_Loc_X = X_Deliverer[0];
		break;
	case 2:
		Second_Loc_Y = Y_Deliverer[2];
		Second_Loc_X = X_Deliverer[2];
		break;
	case 3:
		Second_Loc_Y = Y_Deliverer[3];
		Second_Loc_X = X_Deliverer[3];
		break;
	case 4:
		Second_Loc_Y = Y_Deliverer[0];
		Second_Loc_X = X_Deliverer[4];
		break;
	case 5:
		Second_Loc_Y = Y_Deliverer[4];
		Second_Loc_X = X_Deliverer[4];
		break;
	case 6:
		Second_Loc_Y = Y_Deliverer[1];
		Second_Loc_X = X_Deliverer[3];
		break;
	case 7:
		Second_Loc_Y = Y_Deliverer[1];
		Second_Loc_X = X_Deliverer[1];
		break;
	case 8:
		Second_Loc_Y = Y_Deliverer[3];
		Second_Loc_X = X_Deliverer[1];
		break;
	case 9:
		Second_Loc_Y = Y_Deliverer[2];
		Second_Loc_X = X_Deliverer[0];
		break;
	case 10:
		Second_Loc_Y = Y_Deliverer[0];
		Second_Loc_X = X_Deliverer[2];
		break;
	case 11:
		Second_Loc_Y = Y_Deliverer[4];
		Second_Loc_X = X_Deliverer[2];
		break;
	case 12:
		Second_Loc_Y = Y_Deliverer[2];
		Second_Loc_X = X_Deliverer[4];
		break;
	case 20:
		Second_Loc_Y = Y_Deliverer[2];
		Second_Loc_X = X_Deliverer[2];
		break;
	case 21:
		Second_Loc_Y = Y_Deliverer[0];
		Second_Loc_X = X_Deliverer[3];
		break;
	case 22:
		Second_Loc_Y = Y_Deliverer[1];
		Second_Loc_X = X_Deliverer[3];
		break;
	case 23:
		Second_Loc_Y = Y_Deliverer[2];
		Second_Loc_X = X_Deliverer[3];
		break;
	default:
		Second_Loc_Y = 0.0f;
		Second_Loc_X = 0.0f;
		break;
	}

	switch (Third_Loc_Number)
	{
	case 20:
		Third_Loc_Y = Y_Deliverer[2];
		Third_Loc_X = X_Deliverer[0];
		break;
	case 21:
		Third_Loc_Y = Y_Deliverer[0];
		Third_Loc_X = X_Deliverer[2];
		break;
	case 22:
		Third_Loc_Y = Y_Deliverer[1];
		Third_Loc_X = X_Deliverer[1];
		break;
	case 23:
		Third_Loc_Y = Y_Deliverer[2];
		Third_Loc_X = X_Deliverer[0];
		break;
	default:
		Third_Loc_Y = 0.0f;
		Third_Loc_X = 0.0f;
		break;
	}

}

/**************************************  OPENMV1  ********************************************		
	  COLOR			  TARGET			     ANGLE							  ESWN				
	Red		1		FLAG	1			  从X轴正方向						2	|	1			
	Green	2		上		2			  开始的角度				   -------------------
	Blue	3		下		3												3	|	4		
	Yellow  4		左		4				(Xr,Yr)					  (Target_X,Target_Z)		
					右		5	   (0,0)………………………			…………………(160,120)
					左上	6		…					…	----->	…					…
					右上	7		…	   (160,120)	…	----->	…	    (0,0)		…	
					左下	8		…					…	----->	…					…
					右下	9		…………………(320,240)		   (-160,-120)………………	
**********************************************************************************************/

/**************************************  OPENMV2  ********************************************
	  COLOR			  SHAPE   			Target				Go
	Red		1		矩形	1		向左		1		钻圈	1
	Green	2		三角	2		向右		2
	Blue	3		圆形	3		正中心		3
	Yellow  4						
**********************************************************************************************/

/***************************************  V831  **********************************************
	  COLOR			  SHAPE   				(Xr,Yr)					  (Target_X,Target_Y)
	Red		1		矩形	1		(0,0)………………………			…………………(120,120)
	Green	2		三角	2		…					…	----->	…					…
	Blue	3		圆形	3		…	   (120,120)	…	----->	…	    (0,0)		…	
	Yellow  4						…					…	----->	…					…
									…………………(240,240)		   (-120,-120)………………
**********************************************************************************************/

 
#define Openmv1		1
#define Openmv2		2
#define Openmv3		3
#define V831		4
#define K210		5
#define Camera_S	6
#define Crude_Distance	50.0f			
u16 Pre_data1, Pre_data2, Pre_data3, Pre_data4;
u8 Shape_Number;
void Camera_Decoder(u8 Device_Choose)
{
	u8 Radiu_Flag, Arc_Flag, Circle_Flag, Final_Push;
	float Target_X, Target_Y, Target_Z_Pre, Target_Z, Target_X_Special, Target_Z_Special, Special_Angle;
	Rectify_X = 0;
	Rectify_Y = 0;

	if (Device_Choose == Openmv1)
	{
		Color_Number = Camera_data1;

		//识别弧
		if (Camera_data2 == 0 && Camera_data4)
		{
			Arc_Flag = 1;
			if (Camera_data4 == 1 || Camera_data4 == 2)
			{
				Target_X = Crude_Distance * cos(Camera_data3* DEG2RAD);
				Target_Z_Pre = Crude_Distance * sin(Camera_data3* DEG2RAD);
			}
			else if (Camera_data4 == 3 && Camera_data4 == 4)
			{
				Target_X = Crude_Distance * cos(Camera_data3* DEG2RAD);
				Target_Z_Pre = -Crude_Distance * sin(Camera_data3* DEG2RAD);
			}

			if (Target_Z_Pre < -5 || Target_Z_Pre > 5)
				Target_Z = Target_Z_Pre;
			else
				Target_Z = 0;
		}
		else	Arc_Flag = 0;

		//识别圆
		if (Camera_data2 >= 1 && Camera_data2 <= 9)
		{
			Circle_Flag = 1;
			Target_X = (Camera_data3 - 160)*1.0 / 2;
			if (Camera_data4 < 100 || Camera_data4 > 140)
				Target_Z = (120 - Camera_data4)*1.0 / 2;
			else
				Target_Z = 0.0;
		}
		else 
			Circle_Flag = 0;

		//中心点是否正对圆边
		if (Camera_data2 == 1)
			Radiu_Flag = 1;
		else
			Radiu_Flag = 0;

		if (Radiu_Flag)
		{
			Final_Push = 1;
			if (Arc_Flag)
			{
				if (Camera_data4 == 1 || Camera_data4 == 2)
				{
					Target_X_Special = Crude_Distance * cos(Camera_data3* DEG2RAD);
					Target_Z_Special = Crude_Distance * sin(Camera_data3* DEG2RAD);
				}
				else if (Camera_data4 == 3 && Camera_data4 == 4)
				{
					Target_X_Special = Crude_Distance * cos(Camera_data3* DEG2RAD);
					Target_Z_Special = -Crude_Distance * sin(Camera_data3* DEG2RAD);
				}
			}
			else if (Circle_Flag)
			{
				Target_X = Camera_data3 - 160;
				Target_Z = 120 - Camera_data4;
				Special_Angle = atan2(Target_Z, Target_X);
				Target_X_Special = Crude_Distance * cos(Special_Angle*DEG2RAD);
				Target_Z_Special = Crude_Distance * sin(Special_Angle*DEG2RAD);
			}
		}
		if (Final_Push)
		{
			Move_X = Target_X_Special;
			Move_Z = Target_Z_Special;
		}
		else
		{
			Move_X = Target_X;
			Move_Z = Target_Z;
		}
	}

	else if (Device_Choose == V831)
	{

		if (Camera_data1 && Camera_data2)
		{
			//先变成-100到100
			FL_X = -(Camera_data1 - 80)*1.0 / 0.8 * cos(Roll* DEG2RAD);
			FL_Y = -(60 - Camera_data2)*1.0 / 0.6 * cos(Pitch* DEG2RAD);
		}
		else
		{
			FL_X = 0;
			FL_Y = 0;
		}
	}

	else if (Device_Choose == Openmv2)
	{
		Color_Number = Camera_data1;
		Shape_Number = Camera_data2;
		Circle_Tar_Flag = Camera_data3;
		Circle_Gooooo = Camera_data4;
	}

}


#ifndef _FFCY_POSITION_CTRL_H
#define _FFCY_POSITION_CTRL_H

#define OpticalFlow_Speed_Control_Mode 1
#define OpticalFlow_Speed_Control_Max  250

void Loiter_Control(uint8_t force_brake_flag);
void OpticalFlow_SINS_Reset(void);
void OpticalFlow_Ctrl_Reset(void);
void OpticalFlow_Vel_Control(Vector2f target);
void OpticalFlow_Pos_Control(void);
void OpticalFlow_Pos_Control_Y(void);
void OpticalFlow_Pos_Control_X(void);
void OpticalFlow_Y_Vel_Control(float target_y);
extern float OpticalFlow_Expect_Speed_Mapping(float input, uint16_t input_max, float output_max);

extern Vector2f OpticalFlow_Pos_Ctrl_Expect;
extern Vector2f OpticalFlow_Pos_Ctrl_Err;
extern Vector2f OpticalFlow_Pos_Ctrl_Integrate;
extern Vector2f OpticalFlow_Pos_Ctrl_Output;
extern Vector2f OpticalFlow_Ctrl_Err;
extern Vector2f OpticalFlow_Ctrl_Integrate;
extern Vector2f OpticalFlow_Ctrl_Output, OpticalFlow_Ctrl_Expect;
extern int each_place_laser[50];
extern int the_danger_place;
extern int the_number;


void Thr_Scale_Set(Vector_RC *rc_date);
uint16 Althold_Control(void);
float get_stopping_point_z(Vector3f *stopping_point);
uint8_t Thr_Push_Over_Deadband(void);


extern float Yaw_Vel_Feedforward_Output;//竖直速度前馈控制器输出;
extern float Yaw_Vel_Feedforward_Rate;//竖直速度前馈控制器，APM里面为1、0.45;
extern float Yaw_Vel_Feedforward_Delta;//竖直期望速度变化率;
extern float Yaw_Vel_Target;

extern float Altitude_Position;
extern uint16_t  Deadband;//油门中位死区
extern uint16_t  Deadzone_Min;
extern uint16_t  Deadzone_Max;
extern uint8_t Thr_Push_Over_State;


#define Self_Balance_Mode 1//自稳、纯姿态加油门补偿
#define High_Hold_Mode    2//定高模式
#define Pos_Free_Mode 1//水平位置no fixed
#define Pos_Hold_Mode 2//定点模式
//GPS定点下打杆控速模式与直接姿态角
#define Speed_Mode 0  //GPS定点模式下，打杆控速
#define Angle_Mode 1  //GPS定点模式下，打杆直接给姿态期望角

/*****************遥控器行程设置**********************/
#define  Climb_Up_Speed_Max    300//向上最大攀爬速度，cm/s 150 250  400
#define  Climb_Down_Speed_Max  150//向下最大下降速度，cm/s  80 120  180

/****************************************************************
当油门推重比较小时，若上升下降期望加速度比较大，
会导致输出映射的里面的姿态控制量得不到充分输出，
使得快速上升下降时，姿态不平稳，若此过程持续时间长，
会导致姿态长期得不到修正，直致最后炸鸡，故推重比较
小时，可将期望加速度限小一点，或者做控制量优先级处理
*******************************************************************/
#define  Climb_Up_Acceleration_Max     500//向上最大攀爬加速度，cm/s^2 
#define  Climb_Down_Acceleration_Max   300//向下最大下降加速度，cm/s^2 



#define  Thr_Start  1100//起转油门量，油门倾角补偿用，太大会导致过补偿



#define  Nav_Speed_Max  500//最大期望水平速度为5m/s


/*****************一键起飞高度设置，单位为cm，比如100表示原地起飞到相对初始位置1米高的位置**********************/
#define  Auto_Launch_Target 100//一键起飞的目标高度，相对起飞高度，使用超声波时，请勿超过超过声波量程，推荐200以下

//#define  Thr_Min 950  //最低油门 即能使电调初始化的PWM值 未解锁时的PWM值
//#define  Thr_Idle 1050//PWM怠速 怠速时的PWM值
//#define  Thr_Fly_Start  1100//起飞油门量 1250

#define  Thr_Min 1050  //最低油门 即能使电调初始化的PWM值 未解锁时的PWM值
#define  Thr_Idle 1150//PWM怠速 怠速时的PWM值
#define  Thr_Fly_Start  1200//起飞油门量 1250



/*******************************************************************/
#define  Thr_Hover_Default 1480//默认悬停油门，直接定高起飞时用1500


#endif

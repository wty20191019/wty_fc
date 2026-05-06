/*----------------------------------------------------------------------------------------------------------------------/
*               本程序版权著作权属于中国民航大学飞凡创翼团队，
*               购买者要为飞凡创翼团队提供保护，未经作者许可，不得将源代码提供给他人，不得将源代码放到网上供他人免费下载，
*               更不能以此销售牟利，如发现上述行为，飞凡创翼团队将诉之以法律解决！！！
-----------------------------------------------------------------------------------------------------------------------/
*               修改日期:2023/5/6
*               版本：TanlorS V1.1
*               版权所有，盗版必究。
*               Copyright(C) 中国民航大学飞凡创翼团队 2017-2025
*               All rights reserved
----------------------------------------------------------------------------------------------------------------------*/
#include "Headfile.h"

extern int fly_hight_hight;
extern float Default_speed_out;

int main()
{
	Board_Init();//硬件资源+相关外设初始化
	//Pitch_Offset = 0.8;
	//Roll_Offset = -0.6;
	//fly_hight_hight = 110;
	//Default_speed_out = 30.0f;

	while (1)
	{
		State_Led = !State_Led;
		if (AUTO_FLY_FLAG == 1)
		{
			AUTO_FLY_FLAG = 0;
			BEEP_flag = 1;
			LCD_CLS();
			OLED_LStr(40, 2, "fly now");
			OLED_LStr(40, 3, "fly now");
			OLED_LStr(40, 4, "fly now");
			delay_ms(500);
			delay_ms(500);
			MY_SDK_REST();
			key_flag = 1;
			Controler_State = Unlock_Controler;
			Lock_Makesure_Cnt = 0;
			Unlock_Makesure_Cnt = 0;
			Reset_RC_Calibartion(1);
			LCD_CLS();
		}

		Save_Or_Reset_PID_Parameter();//运用地面站，修改控制参数
 		if (ACC_Calibration_Flag == 1)//加速度计标定
 		{
 			Accel_Calibartion();
 		}
 		else if (RC_Calibration_Flag == 1)//遥控器行程标定
 		{
 			RC_Calibration_Check(PPM_Databuf);
 			RC_Calibration_Flag = 0;
 		}
		else
		{
			Lu_Jing();
			Key_Scan(0);//按键扫描
			Quad_DynamicParament_Show(My_page_number);//显示屏刷新
		}
	}
}

u8 Ringbuf_Data = 0;
struct system_TimeCost System_TimeCost;
void  AHRSUpdate();
#define ATTITUDE_ESTIMAT_RATE	200	//姿态解算速率
#define ATTITUDE_ESTIMAT_DT		(1.0/ATTITUDE_ESTIMAT_RATE)
extern void Ground_Station(void);
extern void Radar_StateMachine(void);
u8 U6_Send_Buff6[10];
u32 u6dcnd = 0;


void FFCY_Main_Processing()
{
	/**********************遥控器*****************************/
	RC_Statemachine();
	/*************加速度计、陀螺仪数字量采集******************/
	GET_MPU_DATA();  
	/*********************姿态解算***************************/
	imuUpdate(sensors1.acc, sensors1.gyro, ATTITUDE_ESTIMAT_DT);
	/*********************惯导态机更新*********************/
	SINS_Prepare();								//得到载体相对导航系的三轴运动加速度
	/**********************光流传感器************************/
	PMW3901_Data_Process();
	/********************************************************/
	DMA_USART2_Data_Prase();
	DMA_USART6_Data_Prase();
	DMA_USART3_Data_Prase();
	Radar_StateMachine();
	Location_Get();								//从传感器得到位置
	/*********************总控制器*************************/
	/*水平位置 + 水平速度 + 姿态（角度 + 角速度）控制器，高度位置 + 高度速度 + 高度加速度控制器*/
	Remote_Control();							//遥控器控制											
	Position_Control();							//位置环控制
	Angle_Control();							//角度环控制
	Gyro_Control();								//角加速度环控制
	Control_Output();							//控制量总输出 
	/*********************上位机接收*********************/
	DMA_USART1_Data_Prase();
	while (!RingBuf_Read(&Ringbuf_Data, &Ground_Station_Ringbuf))
		ANO_DT_Data_Receive_Prepare(Ringbuf_Data);
	/*********************上位机发送*********************/
	ANO_SEND_StateMachine();		
	/*********************USB控制台*********************/
	Consol_main();						
	/*********************闪灯*********************/
	Led_Statemachine();
	Ground_Station();

}

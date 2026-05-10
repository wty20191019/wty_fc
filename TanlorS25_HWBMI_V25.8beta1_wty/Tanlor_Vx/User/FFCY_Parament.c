#include "Headfile.h"
#include "FFCY_Parament.h"


//	1偏差限幅标志；		2积分限幅标志；			3积分分离标志；			4期望；
//	5反馈				6偏差；					7上次偏差；				8偏差限幅值；
//	9积分分离偏差值；	10积分值				11积分限幅值；			12控制参数Kp；
//	13控制参数Ki；		14控制参数Kd；			15控制器总输出；		16上次控制器总输出
//	17总输出限幅度；	18变积分控制时的积分增益
const float Control_Unit[18][20] =
{
	/*                                         Kp        Ki        Kd            */
	/*1  2  3  4  5  6   7  8    9   10  11    12        13        14   15  16    17   18*/
	//好盈乐天20A默认固件F330
	{ 1 ,1 ,0 ,0 ,0 ,0 , 0 ,30  ,0  ,0 , 3,    4.80   ,5.0000    ,0.00  ,0  ,0 , 300,  1 ,  1 ,  1 },//Pitch_Angle;俯仰角度
	{ 1 ,1 ,0 ,0 ,0 ,0 , 0 ,200 ,0  ,0 , 20,   0.30   ,1.0000    ,2.70  ,0  ,0 , 500,  1 ,  1 ,  1 },//Pitch_Gyro;俯仰角速度
	{ 1 ,1 ,0 ,0 ,0 ,0 , 0 ,30  ,0  ,0 , 3 ,   4.80   ,5.0000    ,0.00  ,0  ,0 , 300,  1 ,  1 ,  1 },//Roll_Angle;横滚角
	{ 1 ,1 ,0 ,0 ,0 ,0 , 0 ,200 ,0  ,0 , 20,   0.30   ,1.0000    ,2.70  ,0  ,0 , 500,  1 ,  1 ,  1 },//Roll_Gyro;横滚角速度
	{ 1 ,1 ,0 ,0 ,0 ,0 , 0 ,45  ,0  ,0 , 200 , 3.50   ,0.0000    ,0.00  ,0  ,0 , 300,  1 ,  1 ,  1 },//Yaw_Angle;偏航角
	{ 1 ,1 ,0 ,0 ,0 ,0 , 0 ,300 ,0  ,0 , 150,  1.00   ,3.0000    ,0.00  ,0  ,0 , 400,  1 ,  1 ,  1 },//Yaw_Gyro;偏航角速度   

	//定高参数
	//高度单项比例控制，有偏差限幅、总输出即为最大攀升、下降速度400cm/s
	//Z轴速度比例+积分控制，无偏差限幅
	{ 1  ,1 ,0 ,0 ,0 ,0 , 0 ,200 ,0  ,0 ,100 ,  0.8     ,0.000   ,0    ,0  ,0 ,200,  1 ,  1 ,  1 },//High_Position;海拔高度位置
	{ 1  ,1 ,0 ,0 ,0 ,0 , 0 ,400 ,0  ,0 ,500 ,  1.5     ,0.300   ,0	,0  ,0 ,400,  1 ,  1 ,  1 },//High_Speed;海拔攀升速度

	/*                                       Kp        Ki        Kd            */
	/*1  2  3  4  5  6   7  8   9   10  11    12        13        14  15  16  17*/
	{ 1  ,1 ,0 ,0 ,0 ,0 , 0 ,180 ,0  ,0 ,8,   0.000    ,0.000    ,0    ,0    ,0 ,150,  1 ,  1 ,  1 },//Longitude_Position;水平经度位置
	{ 1  ,1 ,0 ,0 ,0 ,0 , 0 ,300 ,0  ,0 ,240, 0.000    ,0.000    ,0    ,0    ,0 ,500,  1 ,  1 ,  1 },//Longitude_Speed;水平经度速度
	{ 1  ,1 ,0 ,0 ,0 ,0 , 0 ,180 ,0  ,0 ,8,   0.000    ,0.000    ,0    ,0    ,0 ,150,  1 ,  1 ,  1 },//Latitude_Position;水平纬度位置
	{ 1  ,1 ,0 ,0 ,0 ,0 , 0 ,300 ,0  ,0 ,240, 0.000    ,0.000    ,0    ,0    ,0 ,500,  1 ,  1 ,  1 },//Latitude_Speed;水平纬度速度
	/*************加速度控制器****************/
	//期望最大加速度500cm/s^2
	{ 1  ,1 ,0 ,0 ,0 ,0 , 0 ,600  ,0  ,0 ,500,0.00    ,0.0000    ,0.0  ,0   ,0 ,600,  1 ,  1 ,  1 },//垂直加速度控制器
	{ 1  ,1 ,0 ,0 ,0 ,0 , 0 ,100  ,0  ,0 ,3,  0.00    ,0.0000    ,0    ,0   ,0 ,150,  1 ,  1 ,  1 },//水平经度方向加速度控制器
	{ 1  ,1 ,0 ,0 ,0 ,0 , 0 ,100  ,0  ,0 ,15, 0.00    ,0.0000    ,0.0  ,0   ,0 ,25 ,  1 ,  1 ,  1 },//水平维度方向加速度控制器

	/*************光流位置、速度控制器****************/
	{ 1  ,1 ,0 ,0 ,0 ,0 , 0 ,100  ,15 ,0 ,15, 0.40    ,0.00   ,0    ,0   ,0 ,30,   1 ,  1 ,  1 },//水平经度方向位置控制器
	{ 1  ,1 ,0 ,0 ,0 ,0 , 0 ,100  ,30 ,0 ,100,0.18    ,0.002  ,0.0  ,0   ,0 ,200 , 1 ,  1 ,  1 },//水平维度方向速度控制器
	{ 1  ,1 ,0 ,0 ,0 ,0 , 0 ,30   ,30 ,0 ,10, 0.00    ,0      ,0.0  ,0   ,0 ,500 , 1 ,  1 ,  1 },//水平维度方向加速度控制器
};


void PID_Init(PID_Controler *Controler, Controler_Label Label)
{
	Controler->Err_Limit_Flag = (uint8)(Control_Unit[Label][0]);//1偏差限幅标志
	Controler->Integrate_Limit_Flag = (uint8)(Control_Unit[Label][1]);//2积分限幅标志
	Controler->Integrate_Separation_Flag = (uint8)(Control_Unit[Label][2]);//3积分分离标志
	Controler->Expect = Control_Unit[Label][3];//4期望
	Controler->FeedBack = Control_Unit[Label][4];//5反馈值
	Controler->Err = Control_Unit[Label][5];//6偏差
	Controler->Last_Err = Control_Unit[Label][6];//7上次偏差
	Controler->Err_Max = Control_Unit[Label][7];//8偏差限幅值
	Controler->Integrate_Separation_Err = Control_Unit[Label][8];//9积分分离偏差值
	Controler->Integrate = Control_Unit[Label][9];//10积分值
	Controler->Integrate_Max = Control_Unit[Label][10];//11积分限幅值
	Controler->Kp = Control_Unit[Label][11];//12控制参数Kp
	Controler->Ki = Control_Unit[Label][12];//13控制参数Ki
	Controler->Kd = Control_Unit[Label][13];//14控制参数Ki
	Controler->Control_OutPut = Control_Unit[Label][14];//15控制器总输出
	Controler->Last_Control_OutPut = Control_Unit[Label][15];//16上次控制器总输出
	Controler->Control_OutPut_Limit = Control_Unit[Label][16];//17上次控制器总输出
	Controler->Scale_Kp = Control_Unit[Label][17];
	Controler->Scale_Ki = Control_Unit[Label][18];
	Controler->Scale_Kd = Control_Unit[Label][19];
}

Vector3f_pid PID_Parameter[14] = { 0 };
void Total_PID_Init(void)
{
	PID_Init(&Total_Controller.Pitch_Angle_Control, Pitch_Angle_Controler);
	PID_Init(&Total_Controller.Pitch_Gyro_Control, Pitch_Gyro_Controler);
	PID_Init(&Total_Controller.Roll_Angle_Control, Roll_Angle_Controler);
	PID_Init(&Total_Controller.Roll_Gyro_Control, Roll_Gyro_Controler);
	PID_Init(&Total_Controller.Yaw_Angle_Control, Yaw_Angle_Controler);
	PID_Init(&Total_Controller.Yaw_Gyro_Control, Yaw_Gyro_Controler);
	PID_Init(&Total_Controller.High_Position_Control, High_Position_Controler);
	PID_Init(&Total_Controller.High_Speed_Control, High_Speed_Controler);
	PID_Init(&Total_Controller.Longitude_Position_Control, Longitude_Position_Controler);
	PID_Init(&Total_Controller.Longitude_Speed_Control, Longitude_Speed_Controler);
	PID_Init(&Total_Controller.Latitude_Position_Control, Latitude_Position_Controler);
	PID_Init(&Total_Controller.Latitude_Speed_Control, Latitude_Speed_Controler);
	PID_Init(&Total_Controller.High_Acce_Control, High_Acce_Controler);
	PID_Init(&Total_Controller.Longitude_Acce_Control, Longitude_Acce_Controler);
	PID_Init(&Total_Controller.Latitude_Acce_Control, Latitude_Acce_Controler);
	PID_Init(&Total_Controller.Optical_Position_Control, Optical_Position_Controler);
	PID_Init(&Total_Controller.Optical_Speed_Control, Optical_Speed_Controler);
	PID_Init(&Total_Controller.SDK_Roll_Position_Control, SDK_Roll_Position_Controler);
	PID_Init(&Total_Controller.SDK_Pitch_Position_Control, SDK_Roll_Position_Controler);

}


volatile FLASH_Status FLASHStatus1 = FLASH_COMPLETE;      //Flash操作状态变量
#define PID_USE_NUM  14

uint8_t Sort_PID_Cnt = 0;
uint8_t Sort_PID_Flag = 0;
/***************************************************
函数名: void Save_Or_Reset_PID_Parameter(void)
说明:	PID参数内部Flash读、写操作
入口:	无
出口:	无
备注:	配合匿名地面站使用
注释者：飞凡小哥 创翼小妹
****************************************************/
void Save_Or_Reset_PID_Parameter()
{
	//将地面站设置PID参数写入Flash
	if (Sort_PID_Flag == 1)
	{
		Save_PID_Parameter();
		Sort_PID_Flag = 0;
	}
	//将复位PID参数，并写入Flash
	else if (Sort_PID_Flag == 2)
	{
		Total_PID_Init();//将PID参数重置为参数Control_Unit表里面参数
		Save_PID_Parameter();
		Sort_PID_Flag = 0;
		ANO_Send_PID_Flag[0] = 1;//回复默认参数后，将更新的数据发送置地面站
		ANO_Send_PID_Flag[1] = 1;
		ANO_Send_PID_Flag[2] = 1;
		ANO_Send_PID_Flag[3] = 1;
		ANO_Send_PID_Flag[4] = 1;
		ANO_Send_PID_Flag[5] = 1;
	}
}

typedef struct
{
	uint8_t No_0xFF[PID_USE_NUM];
}PID_Parameter_Flag;


PID_Parameter_Flag PID_Parameter_Read_Flag;

void PID_Paramter_Init_With_Flash()
{
	uint16_t i = 0;
	Total_PID_Init();
	for (i = 0; i < PID_USE_NUM; i++)
	{
		PID_Parameter_Read_Flag.No_0xFF[i] = ReadFlashParameterThree(PID1_PARAMETER_KP + 3 * i, &PID_Parameter[i].p, &PID_Parameter[i].i, &PID_Parameter[i].d);
	}
	if (PID_Parameter_Read_Flag.No_0xFF[0] != 0x00
		&& PID_Parameter_Read_Flag.No_0xFF[1] != 0x00
		&& PID_Parameter_Read_Flag.No_0xFF[2] != 0x00
		&& PID_Parameter_Read_Flag.No_0xFF[3] != 0x00
		&& PID_Parameter_Read_Flag.No_0xFF[4] != 0x00
		&& PID_Parameter_Read_Flag.No_0xFF[5] != 0x00
		&& PID_Parameter_Read_Flag.No_0xFF[6] != 0x00
		&& PID_Parameter_Read_Flag.No_0xFF[7] != 0x00
		&& PID_Parameter_Read_Flag.No_0xFF[8] != 0x00
		&& PID_Parameter_Read_Flag.No_0xFF[9] != 0x00
		&& PID_Parameter_Read_Flag.No_0xFF[10] != 0x00)//Flash内数据正常，更新PID参数值
	{
		Total_Controller.Pitch_Gyro_Control.Kp = PID_Parameter[0].p;
		Total_Controller.Pitch_Gyro_Control.Ki = PID_Parameter[0].i;
		Total_Controller.Pitch_Gyro_Control.Kd = PID_Parameter[0].d;

		Total_Controller.Roll_Gyro_Control.Kp = PID_Parameter[1].p;
		Total_Controller.Roll_Gyro_Control.Ki = PID_Parameter[1].i;
		Total_Controller.Roll_Gyro_Control.Kd = PID_Parameter[1].d;

		Total_Controller.Yaw_Gyro_Control.Kp = PID_Parameter[2].p;
		Total_Controller.Yaw_Gyro_Control.Ki = PID_Parameter[2].i;
		Total_Controller.Yaw_Gyro_Control.Kd = PID_Parameter[2].d;

		Total_Controller.Pitch_Angle_Control.Kp = PID_Parameter[3].p;
		Total_Controller.Pitch_Angle_Control.Ki = PID_Parameter[3].i;
		Total_Controller.Pitch_Angle_Control.Kd = PID_Parameter[3].d;

		Total_Controller.Roll_Angle_Control.Kp = PID_Parameter[4].p;
		Total_Controller.Roll_Angle_Control.Ki = PID_Parameter[4].i;
		Total_Controller.Roll_Angle_Control.Kd = PID_Parameter[4].d;

		Total_Controller.Yaw_Angle_Control.Kp = PID_Parameter[5].p;
		Total_Controller.Yaw_Angle_Control.Ki = PID_Parameter[5].i;
		Total_Controller.Yaw_Angle_Control.Kd = PID_Parameter[5].d;

		Total_Controller.High_Speed_Control.Kp = PID_Parameter[6].p;
		Total_Controller.High_Speed_Control.Ki = PID_Parameter[6].i;
		Total_Controller.High_Speed_Control.Kd = PID_Parameter[6].d;

		Total_Controller.High_Position_Control.Kp = PID_Parameter[7].p;
		Total_Controller.High_Position_Control.Ki = PID_Parameter[7].i;
		Total_Controller.High_Position_Control.Kd = PID_Parameter[7].d;

		Total_Controller.Latitude_Speed_Control.Kp = PID_Parameter[8].p;
		Total_Controller.Latitude_Speed_Control.Ki = PID_Parameter[8].i;
		Total_Controller.Latitude_Speed_Control.Kd = PID_Parameter[8].d;

		Total_Controller.Latitude_Position_Control.Kp = PID_Parameter[9].p;
		Total_Controller.Latitude_Position_Control.Ki = PID_Parameter[9].i;
		Total_Controller.Latitude_Position_Control.Kd = PID_Parameter[9].d;
		/***********************位置控制：位置、速度参数共用一组PID参数**********************************************************/
		Total_Controller.Longitude_Speed_Control.Kp = PID_Parameter[8].p;
		Total_Controller.Longitude_Speed_Control.Ki = PID_Parameter[8].i;
		Total_Controller.Longitude_Speed_Control.Kd = PID_Parameter[8].d;

		Total_Controller.Longitude_Position_Control.Kp = PID_Parameter[9].p;
		Total_Controller.Longitude_Position_Control.Ki = PID_Parameter[9].i;
		Total_Controller.Longitude_Position_Control.Kd = PID_Parameter[9].d;

		Total_Controller.High_Acce_Control.Kp = PID_Parameter[10].p;
		Total_Controller.High_Acce_Control.Ki = PID_Parameter[10].i;
		Total_Controller.High_Acce_Control.Kd = PID_Parameter[10].d;

		Total_Controller.Optical_Position_Control.Kp = PID_Parameter[11].p;
		Total_Controller.Optical_Position_Control.Ki = PID_Parameter[11].i;
		Total_Controller.Optical_Position_Control.Kd = PID_Parameter[11].d;

		Total_Controller.Optical_Speed_Control.Kp = PID_Parameter[12].p;
		Total_Controller.Optical_Speed_Control.Ki = PID_Parameter[12].i;
		Total_Controller.Optical_Speed_Control.Kd = PID_Parameter[12].d;

		Total_Controller.SDK_Roll_Position_Control.Kp = PID_Parameter[13].p;
		Total_Controller.SDK_Roll_Position_Control.Ki = PID_Parameter[13].i;
		Total_Controller.SDK_Roll_Position_Control.Kd = PID_Parameter[13].d;

		Total_Controller.SDK_Pitch_Position_Control.Kp = PID_Parameter[13].p;
		Total_Controller.SDK_Pitch_Position_Control.Ki = PID_Parameter[13].i;
		Total_Controller.SDK_Pitch_Position_Control.Kd = PID_Parameter[13].d;

	}
	else
	{
		Save_PID_Parameter();
	}
}



void Save_PID_Parameter(void)
{
	PID_Parameter[0].p = Total_Controller.Pitch_Gyro_Control.Kp;
	PID_Parameter[0].i = Total_Controller.Pitch_Gyro_Control.Ki;
	PID_Parameter[0].d = Total_Controller.Pitch_Gyro_Control.Kd;
	PID_Parameter[1].p = Total_Controller.Roll_Gyro_Control.Kp;
	PID_Parameter[1].i = Total_Controller.Roll_Gyro_Control.Ki;
	PID_Parameter[1].d = Total_Controller.Roll_Gyro_Control.Kd;
	PID_Parameter[2].p = Total_Controller.Yaw_Gyro_Control.Kp;
	PID_Parameter[2].i = Total_Controller.Yaw_Gyro_Control.Ki;
	PID_Parameter[2].d = Total_Controller.Yaw_Gyro_Control.Kd;
	PID_Parameter[3].p = Total_Controller.Pitch_Angle_Control.Kp;
	PID_Parameter[3].i = Total_Controller.Pitch_Angle_Control.Ki;
	PID_Parameter[3].d = Total_Controller.Pitch_Angle_Control.Kd;
	PID_Parameter[4].p = Total_Controller.Roll_Angle_Control.Kp;
	PID_Parameter[4].i = Total_Controller.Roll_Angle_Control.Ki;
	PID_Parameter[4].d = Total_Controller.Roll_Angle_Control.Kd;
	PID_Parameter[5].p = Total_Controller.Yaw_Angle_Control.Kp;
	PID_Parameter[5].i = Total_Controller.Yaw_Angle_Control.Ki;
	PID_Parameter[5].d = Total_Controller.Yaw_Angle_Control.Kd;
	PID_Parameter[6].p = Total_Controller.High_Speed_Control.Kp;
	PID_Parameter[6].i = Total_Controller.High_Speed_Control.Ki;
	PID_Parameter[6].d = Total_Controller.High_Speed_Control.Kd;
	PID_Parameter[7].p = Total_Controller.High_Position_Control.Kp;
	PID_Parameter[7].i = Total_Controller.High_Position_Control.Ki;
	PID_Parameter[7].d = Total_Controller.High_Position_Control.Kd;
	PID_Parameter[8].p = Total_Controller.Latitude_Speed_Control.Kp;
	PID_Parameter[8].i = Total_Controller.Latitude_Speed_Control.Ki;
	PID_Parameter[8].d = Total_Controller.Latitude_Speed_Control.Kd;
	PID_Parameter[9].p = Total_Controller.Latitude_Position_Control.Kp;
	PID_Parameter[9].i = Total_Controller.Latitude_Position_Control.Ki;
	PID_Parameter[9].d = Total_Controller.Latitude_Position_Control.Kd;
	PID_Parameter[10].p = Total_Controller.High_Acce_Control.Kp;
	PID_Parameter[10].i = Total_Controller.High_Acce_Control.Ki;
	PID_Parameter[10].d = Total_Controller.High_Acce_Control.Kd;
	PID_Parameter[11].p = Total_Controller.Optical_Position_Control.Kp;
	PID_Parameter[11].i = Total_Controller.Optical_Position_Control.Ki;
	PID_Parameter[11].d = Total_Controller.Optical_Position_Control.Kd;
	PID_Parameter[12].p = Total_Controller.Optical_Speed_Control.Kp;
	PID_Parameter[12].i = Total_Controller.Optical_Speed_Control.Ki;
	PID_Parameter[12].d = Total_Controller.Optical_Speed_Control.Kd;
	PID_Parameter[13].p = Total_Controller.SDK_Roll_Position_Control.Kp;
	PID_Parameter[13].i = Total_Controller.SDK_Roll_Position_Control.Ki;
	PID_Parameter[13].d = Total_Controller.SDK_Roll_Position_Control.Kd;

	int16_t i = 0;

	TIM_Cmd(TIM2, DISABLE);
	TIM_Cmd(TIM4, DISABLE);
	EXTI->IMR &= ~(EXTI_Line8);
	USART_Cmd(USART1, DISABLE);

	ReadFlashParameterALL(&Table_Parameter);//先把片区内的所有数据都都出来
	for (i = 0; i < PID_USE_NUM; i = i + 1)
	{
		Table_Parameter.Parameter_Table[PID1_PARAMETER_KP + i * 3] = PID_Parameter[i].p;//将需要更改的字段赋新值
		Table_Parameter.Parameter_Table[PID1_PARAMETER_KP + i * 3 + 1] = PID_Parameter[i].i;//将需要更改的字段赋新值
		Table_Parameter.Parameter_Table[PID1_PARAMETER_KP + i * 3 + 2] = PID_Parameter[i].d;//将需要更改的字段赋新值
	}

	FLASH_Unlock();									//解锁 
	FLASH_DataCacheCmd(DISABLE);					//FLASH擦除期间,必须禁止数据缓存

	Parameter_Table_FLASHStatus = FLASH_EraseSector(STMFLASH_GetFlashSector(PARAMETER_TABLE_STARTADDR), VoltageRange_3);
	if (Parameter_Table_FLASHStatus == FLASH_COMPLETE)
	{
		for (i = 0; i < FLIGHT_PARAMETER_TABLE_NUM; i++)
		{
			Parameter_Table_FLASHStatus = FLASH_ProgramWord(PARAMETER_TABLE_STARTADDR + 4 * i, *(uint32_t *)(&Table_Parameter.Parameter_Table[i]));
		}
	}

	Flash_Bug();

	FLASH_DataCacheCmd(ENABLE);	//FLASH擦除结束,开启数据缓存
	FLASH_Lock();//上锁
	TIM_Cmd(TIM2, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
	EXTI->IMR |= EXTI_Line8;

	USART_Cmd(USART1, ENABLE);  //使能串口1 
	USART_ClearITPendingBit(USART1, USART_IT_RXNE);

}

u8 Data_False_Flag = 0;
void Flash_Bug(void)//解决Flash擦除后，数据读取为0xFF的情况，增加了一个标志位，擦除后如果读取到的数据不为0xFF，则继续擦除，直到读取到的数据为0xFF
{
	ReadFlashParameterOne(DATA_TRUE_FLAG, &Data_True_data); 
	if (Data_True_data == Para_Version)
		return;
	else 
		Data_False_Flag = 1; 
		
	while (Data_False_Flag)
	{
		ReadFlashParameterOne(DATA_TRUE_FLAG, &Data_True_data);
		if (Data_True_data == Para_Version)
			break;

		Parameter_Table_FLASHStatus = FLASH_EraseSector(STMFLASH_GetFlashSector(PARAMETER_TABLE_STARTADDR), VoltageRange_3);
		if (Parameter_Table_FLASHStatus == FLASH_COMPLETE)
		{
			for (int16_t i = 0; i < FLIGHT_PARAMETER_TABLE_NUM; i++)
			{
				Parameter_Table_FLASHStatus = FLASH_ProgramWord(PARAMETER_TABLE_STARTADDR + 4 * i, *(uint32_t *)(&Table_Parameter.Parameter_Table[i]));
			}
		}
	}
}

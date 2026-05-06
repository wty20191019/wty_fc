/*----------------------------------------------------------------------------------------------------------------------/
*               本程序版权著作权属于中国民航大学飞凡创翼团队，
*               购买者要为飞凡创翼团队提供保护，未经作者许可，不得将源代码提供给他人，不得将源代码放到网上供他人免费下载，
*               更不能以此销售牟利，如发现上述行为，飞凡创翼团队将诉之以法律解决！！！
-----------------------------------------------------------------------------------------------------------------------/
*               修改日期:2023/5/6
*               版本：TanlorS V1.11
*               版权所有，盗版必究。
*               Copyright(C) 中国民航大学飞凡创翼团队 2017-2025
*               All rights reserved
----------------------------------------------------------------------------------------------------------------------*/
#include "Headfile.h"
#include "FFCY_Menu.h"

uint8 QuadShow_Cnt = 0;

void QuadInit(u8 Head_line)
{
	switch (Head_line)
	{
	case 1:
		OLED_LStr(0, 0, "OLED_Init_OK");

		break;
	default:
		break;
	}
}




struct Key_input key;
/***************************************************
函数名: void Quad_DynamicParament_Show()
说明:	显示屏显示系统运行时的重要动态参数
入口:	无
出口:	无
备注:	在主函数While(1)里面，利用中断空隙时间一直运行
****************************************************/
int QuadKey4_count = 0;
int QuadKey3_count = 0;
int QuadKey2_count = 0;
int QuadKey1_count = 0;
int the_out_color_number = 0;
int the_out_colour[5];
int the_run_number = 0;
int the_run_number_2 = 0;
u8 QuadKey4_off = 0;
u16 QuadKey4_off_delay = 0;
u8 QuadKey3_off = 0;
u16 QuadKey3_off_delay = 0;
u16 the_number_to_change = 0;
int the_number_bling_count = 0;
u8 the_number_bling_flag = 0;
u8 change_time_flag = 0;
extern float the_longth[20];
extern u16 MYSDK_State;
extern int First_Loc_Number, Second_Loc_Number, Third_Loc_Number;
extern int Delivery_Stage;
u8 Cnt_Parament = 0;
u8 Cnt_Parament1 = 0;
u8 Start_Parament = 0;
u8 Start_Parament1 = 0;
u8 Line_Cnt_Parament = 0;
u8 Line_Cnt_Parament1 = 0;
u8 Line_Flag_Parament = 0;
u8 Line_Flag_Parament1 = 0;
u8 LineIN_Flag_Parament = 0;
u8 LineIN_Flag_Parament1 = 0;
extern int fly_hight_hight, fly_hight;
u8 Save_Para_cnt = 0;
u8 Save_Para_cnt1 = 0;
extern float Altitude_Position;
extern float Default_speed_out;
extern float addK;
extern float Back_XZ;
extern float Anm_GoK;
extern float BSpeed_Cor;

void Save_SDK_Parameter(void)
{
	int16_t i = 0;

	TIM_Cmd(TIM2, DISABLE);
	ReadFlashParameterALL(&Table_Parameter);//先把片区内的所有数据都都出来

	//将需要更改的字段赋新值
	Table_Parameter.Parameter_Table[OPENMV_CORRECT] = addK;
	Table_Parameter.Parameter_Table[FIRE_HEAD] = Back_XZ;
	Table_Parameter.Parameter_Table[flyhight] = fly_hight_hight;
	Table_Parameter.Parameter_Table[PITCH_OFFSET] = Pitch_Offset;
	Table_Parameter.Parameter_Table[ROLL_OFFSET] = Roll_Offset;
	Table_Parameter.Parameter_Table[FLY_speed] = Default_speed_out;
	Table_Parameter.Parameter_Table[FIRE_BACK] = Anm_GoK;
	Table_Parameter.Parameter_Table[FIRE_LEFT_1] = BSpeed_Cor;
	
	FLASH_Unlock();									//解锁 
	FLASH_DataCacheCmd(DISABLE);						//FLASH擦除期间,必须禁止数据缓存

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
}

void Load_saved_sdk_parament(void)
{
	ReadFlashParameterALL(&Table_Parameter);//先把片区内的所有数据都都出来
	Pitch_Offset = Table_Parameter.Parameter_Table[PITCH_OFFSET];
	Roll_Offset = Table_Parameter.Parameter_Table[ROLL_OFFSET];
	fly_hight_hight = Table_Parameter.Parameter_Table[flyhight];
	Default_speed_out = Table_Parameter.Parameter_Table[FLY_speed];
	addK = Table_Parameter.Parameter_Table[OPENMV_CORRECT];
	Back_XZ = Table_Parameter.Parameter_Table[FIRE_HEAD];
	Anm_GoK = Table_Parameter.Parameter_Table[FIRE_BACK];
	BSpeed_Cor = Table_Parameter.Parameter_Table[FIRE_LEFT_1];
}

int Page_Danger_line = 0;
int Page_Danger_Data1 = 0;
int Page_Danger_Data2 = 0;
int Page_colour_line = 0;
int Page_Colour_Data1 = 0;
int Page_Colour_Data2 = 0;
int Page_Colour_Data3 = 0;
int Page_Colour_Data4 = 0;

u8 key_flag = 0;
#define Key_Time 400
int Save_para_Colour_cnt = 0;
extern float New_Yaw_Posion;
extern float New_Yaw_speed;
u8 last_Start_Parament = 0;
u8 last_Start_Parament1 = 0;
extern u8 T265_Confidence;
u8 My_page_number_Record = 0;
u8 Time_Sys_Record;
u8 T265_Show_Flag = 1;
extern float Altitude_Speed;
extern u16 Camera_data1, Camera_data2, Camera_data3, Camera_data4;
extern u8 Color_Number;
extern float Move_X, Move_Z;
extern float Rectify_X, Rectify_Y;
extern u8 Camera_Cnt;
extern u8 Circle_Gooooo;
extern float Targ_Pos[3][30];
extern u8 Anml_Num;
extern u8 Fst_Anml;
extern float Fst_Anml_X;
extern float Fst_Anml_Y;
extern u8 Gezi_x;
extern u8 Gezi_y;
extern u8 Targ_Stage;
extern u8 Sec_Anml;
extern float Sec_Anml_X;
extern float Sec_Anml_Y;
extern u8 Thr_Anml;
extern float Thr_Anml_X;
extern float Thr_Anml_Y;
extern u8 Been_Suanshu;
extern u8 Simu_T265_Wrong_Flag;
extern u8 Been_Traverse[10][10];
u8 BT_ShowX = 1, BT_ShowY = 1;
extern float radar_speed_x;
extern float radar_speed_y;

void Quad_DynamicParament_Show(u16 Head_line)
{
	u8 i = 0, n = 0;
	u8 DynamicPara_Show_X1 = 0;				//显示的位置
	u8 DynamicPara_Show_X2 = 41;
	u8 DynamicPara_Show_X3 = 65;
	u8 DynamicPara_Show_X4 = 35;
	u8 DynamicPara_Show_Y = 0;

	/*********************第0页********************/
	if (My_page_number == 0)
	{
		//OLED_P6x8StrBW(DynamicPara_Show_X1, DynamicPara_Show_Y, "-----Console_Main-----", 1);
		OLED_LStr(DynamicPara_Show_X1, 0, "Time0");		Neat_Data_Show(DynamicPara_Show_X2 - 1, 0, System_TimeCost.Timer_Delta.Time_Delta);			Neat_Data_Show(DynamicPara_Show_X3 + 15, 0, System_TimeCost.Timer_Cost.Time_Delta);
		OLED_LStr(DynamicPara_Show_X1, 2, "XX:");		Neat_Data_Show(DynamicPara_Show_X2 - 1, 2, ps_data.velocity.y);								Neat_Data_Show(DynamicPara_Show_X3 + 15, 2, ld_data.translation.x);
		OLED_LStr(DynamicPara_Show_X1, 3, "YY:");		Neat_Data_Show(DynamicPara_Show_X2 - 1, 3, ps_data.velocity.x);								Neat_Data_Show(DynamicPara_Show_X3 + 15, 3, ld_data.translation.y);
		OLED_LStr(DynamicPara_Show_X1, 4, "Dlv_Stg:");	Neat_Data_Show(DynamicPara_Show_X2 - 1, 4, Delivery_Stage);									Neat_Data_Show(DynamicPara_Show_X3 + 15, 4, New_Yaw_Posion);
		OLED_LStr(DynamicPara_Show_X1, 5, "GE:");		Neat_Data_Show(DynamicPara_Show_X2 - 1, 5, Gezi_x);											Neat_Data_Show(DynamicPara_Show_X3 + 15 ,5, Gezi_y);
		OLED_LStr(DynamicPara_Show_X1, 6, "Sdk:");		Neat_Data_Show(DynamicPara_Show_X2 - 1, 6, MYSDK_State);									Neat_Data_Show(DynamicPara_Show_X3 + 15, 6, T265_Confidence);
		
		if (T265_Confidence == 3 && T265_Show_Flag)
			OLED_P6x8StrBW(0, 7, "                      ", 0);
		else
		{
			if (Time_Sys_Record > 58)
			{
				if (Time_Sys[Second] >= 58)
				{
					T265_Show_Flag = 0;
					OLED_P6x8StrBW(0, 7, "T265_Just_Geeeee!!!!!", 1);
				}
				else
					T265_Show_Flag = 1;
			}
			else
			{
				if (Time_Sys[Second] - Time_Sys_Record <= 2)
				{
					T265_Show_Flag = 0;
					OLED_P6x8StrBW(0, 7, "T265_Just_Geeeee!!!!!", 1);
				}
				else
					T265_Show_Flag = 1;
			}
		}

		/************************************************************************/
		/* 页面控制                                                             */
		/************************************************************************/
		if (key.key_down == 1)
		{
			My_page_number = 1;
			LCD_CLS();
		}
		else if (key.Key_up == 1)
		{
			My_page_number = 3;
			LCD_CLS();
		}

	}

	/*********************第1页********************/
	else if (My_page_number == 1)
	{

		OLED_P6x8StrBW(0, 0, "---Parament_Change---", 1);

		if (Line_Cnt_Parament > 6)
		{
			Cnt_Parament = Line_Cnt_Parament - 6;
		}
		else
		{
			Cnt_Parament = 0;
		}
		if (Line_Cnt_Parament <6)
		{
			Start_Parament = Line_Cnt_Parament;
		}
		else
		{
			Start_Parament = 6;
		}
		for (i = 0; i < 7; i++)
		{
			DynamicPara_Show_Y = i;
			switch (Cnt_Parament + i)
			{
			case 0:		OLED_LStr(DynamicPara_Show_X1 + 10, DynamicPara_Show_Y + 1, "addK");			break;
			case 1:		OLED_LStr(DynamicPara_Show_X1 + 10, DynamicPara_Show_Y + 1, "X1");				break;
			case 2:		OLED_LStr(DynamicPara_Show_X1 + 10, DynamicPara_Show_Y + 1, "Y1");				break;
			case 3:		OLED_LStr(DynamicPara_Show_X1 + 10, DynamicPara_Show_Y + 1, "X2");				break;
			case 4:		OLED_LStr(DynamicPara_Show_X1 + 10, DynamicPara_Show_Y + 1, "Y2");				break;
			case 5:		OLED_LStr(DynamicPara_Show_X1 + 10, DynamicPara_Show_Y + 1, "BT_Show");			break;
			case 6:		OLED_LStr(DynamicPara_Show_X1 + 10, DynamicPara_Show_Y + 1, "Show_X");			break;
			case 7:		OLED_LStr(DynamicPara_Show_X1 + 10, DynamicPara_Show_Y + 1, "Show_Y");			break;
			case 8:		OLED_LStr(DynamicPara_Show_X1 + 10, DynamicPara_Show_Y + 1, "BS_Cor");			break;
			case 9:		OLED_LStr(DynamicPara_Show_X1 + 10, DynamicPara_Show_Y + 1, "Correct");			break;
			case 10:	OLED_LStr(DynamicPara_Show_X1 + 10, DynamicPara_Show_Y + 1, "Beishu");			break;
			case 11:	OLED_LStr(DynamicPara_Show_X1 + 10, DynamicPara_Show_Y + 1, "Fly_Higt");		break;
			case 12:	OLED_LStr(DynamicPara_Show_X1 + 10, DynamicPara_Show_Y + 1, "flySpeed");		break;
			case 13:	OLED_LStr(DynamicPara_Show_X1 + 10, DynamicPara_Show_Y + 1, "Pitch_OfS");		break;
			case 14:	OLED_LStr(DynamicPara_Show_X1 + 10, DynamicPara_Show_Y + 1, "ROLL_OfS");		break;
			}

			if (Line_Flag_Parament == 1)
			{
				if (LineIN_Flag_Parament == 0)
				{
					OLED_P6x8StrBW(0, last_Start_Parament + 1, " ", 0);
					OLED_P6x8StrBW(0, Start_Parament + 1, "*", 1);
					OLED_P6x8StrBW(DynamicPara_Show_X3, Start_Parament + 1, " ", 0);
				}
				else
				{
					OLED_P6x8StrBW(DynamicPara_Show_X3, Start_Parament + 1, "*", 1);
					OLED_P6x8StrBW(0, Start_Parament + 1, " ", 0);
				}
			}

			switch (Cnt_Parament + i)
			{
			case 0:		Neat_Data_Show(DynamicPara_Show_X3 + 10, DynamicPara_Show_Y + 1, addK);						break;
			case 1:		Neat_Data_Show(DynamicPara_Show_X3 + 10, DynamicPara_Show_Y + 1, Targ_Pos[0][1]);			break;
			case 2:		Neat_Data_Show(DynamicPara_Show_X3 + 10, DynamicPara_Show_Y + 1, Targ_Pos[1][1]);			break;
			case 3:		Neat_Data_Show(DynamicPara_Show_X3 + 10, DynamicPara_Show_Y + 1, Targ_Pos[0][2]);			break;
			case 4:		Neat_Data_Show(DynamicPara_Show_X3 + 10, DynamicPara_Show_Y + 1, Targ_Pos[1][2]);			break;
			case 5:		Neat_Data_Show(DynamicPara_Show_X3 + 10, DynamicPara_Show_Y + 1, Been_Traverse[BT_ShowX][BT_ShowY]);			break;
			case 6:		Neat_Data_Show(DynamicPara_Show_X3 + 10, DynamicPara_Show_Y + 1, BT_ShowX);					break;
			case 7:		Neat_Data_Show(DynamicPara_Show_X3 + 10, DynamicPara_Show_Y + 1, BT_ShowY);					break;
			case 8:		Neat_Data_Show(DynamicPara_Show_X3 + 10, DynamicPara_Show_Y + 1, BSpeed_Cor);				break;
			case 9:		Neat_Data_Show(DynamicPara_Show_X3 + 10, DynamicPara_Show_Y + 1, Back_XZ);					break;
			case 10:	Neat_Data_Show(DynamicPara_Show_X3 + 10, DynamicPara_Show_Y + 1, Anm_GoK);					break;
			case 11:	Neat_Data_Show(DynamicPara_Show_X3 + 10, DynamicPara_Show_Y + 1, fly_hight_hight);			break;
			case 12:	Neat_Data_Show(DynamicPara_Show_X3 + 10, DynamicPara_Show_Y + 1, Default_speed_out);		break;
			case 13:	Neat_Data_Show(DynamicPara_Show_X3 + 10, DynamicPara_Show_Y + 1, Pitch_Offset);				break;
			case 14:	Neat_Data_Show(DynamicPara_Show_X3 + 10, DynamicPara_Show_Y + 1, Roll_Offset);				break;
			}
		}

		/************************************************************************/
		/* 页面控制                                                             */
		/************************************************************************/
		//不在行内
		if (Line_Flag_Parament == 0)
		{
			if (key.Key_plus == 1)
			{
				if (Line_Flag_Parament == 0)
					Line_Flag_Parament = 1;
			}
			if (key.key_del == 1)
			{
				if (Line_Flag_Parament == 1)
					Line_Flag_Parament = 0;
			}
			if (key.key_down == 1)
			{
				My_page_number = 2;
				LCD_CLS();
			}
			else if (key.Key_up == 1)
			{
				My_page_number = 0;
				LCD_CLS();
			}
		}
		else
		{
			if (LineIN_Flag_Parament == 0)
			{
				if (key.key_down == 1)
				{
					last_Start_Parament = Start_Parament;
					if (Line_Cnt_Parament != 0)
						Line_Cnt_Parament--;

					Save_Para_cnt = 0;
				}
				else if (key.Key_up == 1)
				{
					last_Start_Parament = Start_Parament;
					Line_Cnt_Parament++;
					if (Line_Cnt_Parament > 14)
					{
						Line_Cnt_Parament = 14;
						Save_Para_cnt++;
					}
					if (Save_Para_cnt > 5)
					{
						LCD_CLS();
						OLED_LStr(40, 0, "SAVING!!!!!!");
						OLED_LStr(40, 1, "SAVING!!!!!!");
						OLED_LStr(40, 2, "SAVING!!!!!!");
						OLED_LStr(40, 3, "SAVING!!!!!!");
						OLED_LStr(40, 4, "SAVING!!!!!!");
						OLED_LStr(40, 5, "SAVING!!!!!!");
						OLED_LStr(40, 6, "SAVING!!!!!!");
						OLED_LStr(40, 7, "SAVING!!!!!!");
						Save_SDK_Parameter();
						LCD_CLS();
					}
					else
					{
						//LCD_CLS();
					}
				}
				if (key.Key_plus == 1)
				{
					LineIN_Flag_Parament = 1;
				}
				if (key.key_del == 1)
				{
					Line_Flag_Parament = 0;
					LCD_CLS();
				}
			}
			if (LineIN_Flag_Parament == 1)
			{
				if (key.key_del == 1)
				{
					LineIN_Flag_Parament = 0;
				}
				if (key.Key_up == 1)
				{
					if (Line_Cnt_Parament == 0)
						addK--;
					else if (Line_Cnt_Parament == 1)
						Lonth_Head--;
					else if (Line_Cnt_Parament == 2)
						Lonth_Left1--;
					else if (Line_Cnt_Parament == 3)
						Lonth_Back--;
					else if (Line_Cnt_Parament == 4)
						Lonth_Left2--;
					else if (Line_Cnt_Parament == 5)
						BT_ShowX--;
					else if (Line_Cnt_Parament == 6)
						BT_ShowX--;
					else if (Line_Cnt_Parament == 7)
						BT_ShowY--;
					else if (Line_Cnt_Parament == 8)
						BSpeed_Cor--;
					else if (Line_Cnt_Parament == 9)
						Back_XZ--;
					else if (Line_Cnt_Parament == 10)
						Anm_GoK -= 0.1;
					else if (Line_Cnt_Parament == 11)
						fly_hight_hight--;
					else if (Line_Cnt_Parament == 12)
						Default_speed_out--;
					else if (Line_Cnt_Parament == 13)
						Pitch_Offset -= 0.1;
					else if (Line_Cnt_Parament == 14)
						Roll_Offset -= 0.1;
				}
				else if (key.key_down == 1)
				{
					if (Line_Cnt_Parament == 0)
						addK++;
					else if (Line_Cnt_Parament == 1)
						Lonth_Head++;
					else if (Line_Cnt_Parament == 2)
						Lonth_Left1++;
					else if (Line_Cnt_Parament == 3)
						Lonth_Back++;
					else if (Line_Cnt_Parament == 4)
						Lonth_Left2++;
					else if (Line_Cnt_Parament == 5)
						BT_ShowX++;
					else if (Line_Cnt_Parament == 6)
						BT_ShowX++;
					else if (Line_Cnt_Parament == 7)
						BT_ShowY++;
					else if (Line_Cnt_Parament == 8)
						BSpeed_Cor++;
					else if (Line_Cnt_Parament == 9)
						Back_XZ++;
					else if (Line_Cnt_Parament == 10)
						Anm_GoK += 0.1;
					else if (Line_Cnt_Parament == 11)
						fly_hight_hight++;
					else if (Line_Cnt_Parament == 12)
						Default_speed_out++;
					else if (Line_Cnt_Parament == 13)
						Pitch_Offset += 0.1;
					else if (Line_Cnt_Parament == 14)
						Roll_Offset += 0.1;
				}
			}
		}
	}

	/*********************第2页********************/
	else  if (My_page_number == 2)
	{

		OLED_P6x8StrBW(DynamicPara_Show_X1, DynamicPara_Show_Y, "-----Uav_Attitude-----", 1);
		for (i = 0; i < 4; i++)
		{
			DynamicPara_Show_Y = i;
			//LCD_clear_L(0, i);
			switch (0 + i)
			{
			case 0:	OLED_LStr(DynamicPara_Show_X1, DynamicPara_Show_Y + 2, "Time0");		Neat_Data_Show(DynamicPara_Show_X2, DynamicPara_Show_Y + 2, System_TimeCost.Timer_Delta.Time_Delta);	Neat_Data_Show(DynamicPara_Show_X3 + 15, DynamicPara_Show_Y + 2, System_TimeCost.Timer_Cost.Time_Delta);		break;
			case 1:	OLED_LStr(DynamicPara_Show_X1, DynamicPara_Show_Y + 2, "Roll:");		Neat_Data_Show(DynamicPara_Show_X2, DynamicPara_Show_Y + 2, Roll);							break;
			case 2:	OLED_LStr(DynamicPara_Show_X1, DynamicPara_Show_Y + 2, "Pitch:");		Neat_Data_Show(DynamicPara_Show_X2, DynamicPara_Show_Y + 2, Pitch);							break;
			case 3:	OLED_LStr(DynamicPara_Show_X1, DynamicPara_Show_Y + 2, "Yaw:");			Neat_Data_Show(DynamicPara_Show_X2, DynamicPara_Show_Y + 2, Yaw);							break;
			}
		}
		/************************************************************************/
		/* 页面控制                                                             */
		/************************************************************************/
		if (key.key_down == 1)
		{
			My_page_number = 3;
			LCD_CLS();
		}
		else if (key.Key_up == 1)
		{
			My_page_number = 1;
			LCD_CLS();
		}
	}

	/*********************第3页********************/
	else  if (My_page_number == 3)
	{

		OLED_P6x8StrBW(DynamicPara_Show_X1, DynamicPara_Show_Y, "-----Camera_Data-----", 1);
		for (i = 0; i < 6; i++)
		{
			DynamicPara_Show_Y = i;
			//LCD_clear_L(0, i);
			switch (0 + i)
			{
			case 0:	OLED_LStr(DynamicPara_Show_X1, DynamicPara_Show_Y + 2, "Num:");			Neat_Data_Show(DynamicPara_Show_X2, DynamicPara_Show_Y + 2, Anml_Num);			Neat_Data_Show(DynamicPara_Show_X3 + 15, DynamicPara_Show_Y + 2, Fst_Anml);			break;
			case 1:	OLED_LStr(DynamicPara_Show_X1, DynamicPara_Show_Y + 2, "F_XY:");		Neat_Data_Show(DynamicPara_Show_X2, DynamicPara_Show_Y + 2, Fst_Anml_X);		Neat_Data_Show(DynamicPara_Show_X3 + 15, DynamicPara_Show_Y + 2, Fst_Anml_Y);		break;
			case 2:	OLED_LStr(DynamicPara_Show_X1, DynamicPara_Show_Y + 2, "Sec:");			Neat_Data_Show(DynamicPara_Show_X2, DynamicPara_Show_Y + 2, Sec_Anml);			break;
			case 3:	OLED_LStr(DynamicPara_Show_X1, DynamicPara_Show_Y + 2, "S_XY:");		Neat_Data_Show(DynamicPara_Show_X2, DynamicPara_Show_Y + 2, Sec_Anml_X);		Neat_Data_Show(DynamicPara_Show_X3 + 15, DynamicPara_Show_Y + 2, Sec_Anml_Y);		break;
			case 4:	OLED_LStr(DynamicPara_Show_X1, DynamicPara_Show_Y + 2, "Thr:");			Neat_Data_Show(DynamicPara_Show_X2, DynamicPara_Show_Y + 2, Thr_Anml);			break;
			case 5:	OLED_LStr(DynamicPara_Show_X1, DynamicPara_Show_Y + 2, "T_XY:");		Neat_Data_Show(DynamicPara_Show_X2, DynamicPara_Show_Y + 2, Thr_Anml_X);		Neat_Data_Show(DynamicPara_Show_X3 + 15, DynamicPara_Show_Y + 2, Thr_Anml_Y);		break;
			}
		}
		/************************************************************************/
		/* 页面控制                                                             */
		/************************************************************************/
		if (key.key_down == 1)
		{
			My_page_number = 0;
			LCD_CLS();
		}
		else if (key.Key_up == 1)
		{
			My_page_number = 2;
			LCD_CLS();
		}
	}

}


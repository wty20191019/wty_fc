/*----------------------------------------------------------------------------------------------------------------------/
*               本程序只供购买者学习使用，版权著作权属于飞凡创翼团队，飞凡创翼团队将飞控程序源码提供给购买者，
*               购买者要为飞凡创翼团队提供保护，未经作者许可，不得将源代码提供给他人，不得将源代码放到网上供他人免费下载，
*               更不能以此销售牟利，如发现上述行为，飞凡创翼团队将诉之以法律解决！！！
-----------------------------------------------------------------------------------------------------------------------/
*               修改日期:2023/5/6
*               版本：TanlorS V1.0
*               版权所有，盗版必究。
*               Copyright(C) 中国民航大学飞凡创翼团队 2017-2025
*               All rights reserved
----------------------------------------------------------------------------------------------------------------------*/
#include "Headfile.h"
#include "Calibration.h"
#include "CalibrationRoutines.h"


u8 RC_Calibration_Flag = 0;
u8 ACC_Calibration_Flag = 0;

uint8_t Accel_Calibration_Finished[6] = { 0,0,0,0,0,0 };//对应面校准完成标志位
float Acce_sample_sum[3] = { 0,0,0 };//加速度和数据
u8 Acc_det_cnt[6] = { 0,0,0,0,0,0 };
extern uint32_t APP_Rx_ptr_in;
void Consol_write(char *buff)
{
	usb_printf(buff);
}


u8 Consoal_state = 0;
char *String_output1 = "欢迎进入校准控制台菜单 输入:\n 1:加速度计校准 \n 2:遥控器校准\n";
char *String_output2 = "进行加速度计校准\n";
char *String_output3 = "进行遥控器校准\n";
char *String_output4 = "输入错误 正在回到主菜单.....\n";
char *String_output5 = "正在进行加速度计六面校准 输入C取消\n";
char *String_output6 = "正在进行遥控器校准 输入C取消\n";
u8 Consol_main_count = 0;
u8 Return_main_flag = 0;
u8 Consoal_reboot_flag = 0;

char read_buff[10];


extern u8 Hid_RxData[50];
extern u8 USB_ReceiveFlg ;

void Consol_main()
{
	Consol_main_count++;
	u8 i = 0;
	if (Consol_main_count >= 10)
	{
		Consol_main_count = 0;
	}
	else
		return;

	if (USB_ReceiveFlg == TRUE)
	{
		USB_ReceiveFlg = 0x00;
	}
	else
		return;

	Consoal_reboot_flag = 1;
	while (Consoal_reboot_flag == 1)
	{
		Consoal_reboot_flag = 0;
		//输入C回到主菜单

		if (Hid_RxData[0] == 'C' || Return_main_flag == 1)
		{
				Consoal_state = 1;
				Consol_write(String_output1);
				Return_main_flag = 0;
				ACC_Calibration_Flag = 0;

				for (i = 0; i < 6; i++)
				{
					Accel_Calibration_Finished[i] = 0;//对应面标志位清零
					Acc_det_cnt[i] = 0;
				}
				for (i = 0; i < 3; i++)
				 Acce_sample_sum[i] = 0;
				return;
		}

		if (Consoal_state == 1)
		{
			if (Hid_RxData[0] == '1')
			{
				Consol_write(String_output2);
				Consoal_state = 2;
			}
			else if (Hid_RxData[0] == '2')
			{
				Consol_write(String_output3);
				Consoal_state = 3;
			}
			else
			{
				Consol_write(String_output4);
				Return_main_flag = 1;
				Consoal_reboot_flag = 1;
			}
		}
		if (Consoal_state == 2)
		{
			Consol_write(String_output5);
			ACC_Calibration_Flag = 1;
		}
		if (Consoal_state == 3)
		{
			Consol_write(String_output6);
			RC_Calibration_Flag = 1;
		}
	}
}
int16_t Mag_Offset[3]={0,0,0};
float Mag_Data[3]={0};
Vector2f MagN={0,0};
float HMC5883L_Yaw=0;
/***************加速度计6面矫正，参考APM代码，配合遥控器进行现场矫正**************************/
void Calibrate_Reset_Matrices(float dS[6], float JS[6][6])
{
  int16_t j,k;
  for( j=0; j<6; j++ )
  {
    dS[j] = 0.0f;
    for( k=0; k<6; k++ )
    {
      JS[j][k] = 0.0f;
    }
  }
}

void Calibrate_Find_Delta(float dS[6], float JS[6][6], float delta[6])
{
  int16_t i,j,k;
  float mu;
  //make upper triangular
  for( i=0; i<6; i++ ) {
    //eliminate all nonzero entries below JS[i][i]
    for( j=i+1; j<6; j++ ) {
      mu = JS[i][j]/JS[i][i];
      if( mu != 0.0f ) {
        dS[j] -= mu*dS[i];
        for( k=j; k<6; k++ ) {
          JS[k][j] -= mu*JS[k][i];
        }
      }
    }
  }
  //back-substitute
  for( i=5; i>=0; i-- ) {
    dS[i] /= JS[i][i];
    JS[i][i] = 1.0f;
    
    for( j=0; j<i; j++ ) {
      mu = JS[i][j];
      dS[j] -= mu*dS[i];
      JS[i][j] = 0.0f;
    }
  }
  for( i=0; i<6; i++ ) {
    delta[i] = dS[i];
  }
}

void Calibrate_Update_Matrices(float dS[6],
                               float JS[6][6],
                               float beta[6],
                               float data[3])
{
  int16_t j, k;
  float dx, b;
  float residual = 1.0;
  float jacobian[6];
  for(j=0;j<3;j++)
  {
    b = beta[3+j];
    dx = (float)data[j] - beta[j];
    residual -= b*b*dx*dx;
    jacobian[j] = 2.0f*b*b*dx;
    jacobian[3+j] = -2.0f*b*dx*dx;
  }
  
  for(j=0;j<6;j++)
  {
    dS[j]+=jacobian[j]*residual;
    for(k=0;k<6;k++)
    {
      JS[j][k]+=jacobian[j]*jacobian[k];
    }
  }
}

uint8 Calibrate_accel(Acce_Unit accel_sample[6],
                      Acce_Unit *accel_offsets,
                      Acce_Unit *accel_scale)
{
  int16_t i;
  int16_t num_iterations = 0;
  float eps = 0.000000001;
  float change = 100.0;
  float data[3]={0};
  float beta[6]={0};
  float delta[6]={0};
  float ds[6]={0};
  float JS[6][6]={0};
  u8 success = TRUE;
  // reset
  beta[0] = beta[1] = beta[2] = 0;
  beta[3] = beta[4] = beta[5] = 1.0f/GRAVITY_MSS;
  while( num_iterations < 20 && change > eps ) {
    num_iterations++;
    Calibrate_Reset_Matrices(ds, JS);
    
    for( i=0; i<6; i++ ) {
      data[0] = accel_sample[i].x;
      data[1] = accel_sample[i].y;
      data[2] = accel_sample[i].z;
      Calibrate_Update_Matrices(ds, JS, beta, data);
      
    }
    Calibrate_Find_Delta(ds, JS, delta);
    change =    delta[0]*delta[0] +
      delta[0]*delta[0] +
        delta[1]*delta[1] +
          delta[2]*delta[2] +
            delta[3]*delta[3] / (beta[3]*beta[3]) +
              delta[4]*delta[4] / (beta[4]*beta[4]) +
                delta[5]*delta[5] / (beta[5]*beta[5]);
    for( i=0; i<6; i++ ) {
      beta[i] -= delta[i];
    }
  }
  // copy results out
  accel_scale->x = beta[3] * GRAVITY_MSS;
  accel_scale->y = beta[4] * GRAVITY_MSS;
  accel_scale->z = beta[5] * GRAVITY_MSS;
  accel_offsets->x = beta[0] * accel_scale->x;
  accel_offsets->y = beta[1] * accel_scale->y;
  accel_offsets->z = beta[2] * accel_scale->z;
  
  // sanity check scale
  if(fabsf(accel_scale->x-1.0f) > 0.5f|| fabsf(accel_scale->y-1.0f) > 0.5f|| fabsf(accel_scale->z-1.0f) > 0.5f )
  {
    success = FALSE;
  }
  // sanity check offsets (3.5 is roughly 3/10th of a G, 5.0 is roughly half a G)
  if(fabsf(accel_offsets->x) > 5.0f|| fabsf(accel_offsets->y) > 5.0f|| fabsf(accel_offsets->z) > 5.0f )
  {
    success = FALSE;
  }
  // return success or failure
  return success;
}



float Aoco[6]={1,1,1};
float Aoc[6][6]={1,1,1};
Acce_Unit new_offset={
  0,0,0,
};
Acce_Unit new_scales={
  1.0,1.0,1.0,
};

Acce_Unit Accel_Offset_Read={
  0,0,0,
};
Acce_Unit Accel_Scale_Read={
  0,0,0,
};
uint8_t Cal_Flag=0;


uint8_t flight_direction=6;
uint8_t Accel_Calibration_Flag=0;//加速度计校准模式

uint8_t Accel_Calibration_All_Finished=0;//6面校准全部校准完成标志位
uint16_t Accel_Calibration_Makesure_Cnt=0;
uint16_t Accel_flight_direction_cnt=0;

Acce_Unit acce_sample[6]={0};//三行6列，保存6面待矫正数据
uint8_t Flash_Buf[12]={0};
/***************************************************
函数名: void Accel_Calibartion()
说明:	加速度基本标定、利用遥控器直接进入
入口:	无
出口:	无
备注:	在主函数While(1)里面，利用中断空隙时间一直运行
****************************************************/
u8 Acc_Cal_Falg=0;

u32 Acc_cal_cnt = 0;
#define rate_zero 3

float Acc_det_mat[6][6] = { rate_zero,-rate_zero,rate_zero,-rate_zero,10+ rate_zero,10- rate_zero,
10 + rate_zero,10 - rate_zero,rate_zero,-rate_zero,rate_zero,-rate_zero,
-10 + rate_zero,-10 - rate_zero,rate_zero,-rate_zero,rate_zero,-rate_zero,
rate_zero,-rate_zero,-10 + rate_zero,-10 - rate_zero,rate_zero,-rate_zero,
rate_zero,-rate_zero,10 + rate_zero,10 - rate_zero,rate_zero,-rate_zero,
rate_zero,-rate_zero,rate_zero,-rate_zero,-10 + rate_zero,-10 - rate_zero,
							};
char print_buff[100];
u8 check_move_flag = 0;


uint8_t Accel_Calibartion(void)
{
  uint16 i,j=0;

  /*第一面飞控平放，Z轴正向朝着正上方，Z axis is about 1g,X、Y is about 0g*/
  /*第二面飞控平放，X轴正向朝着正上方，X axis is about 1g,Y、Z is about 0g*/
  /*第三面飞控平放，X轴正向朝着正下方，X axis is about -1g,Y、Z is about 0g*/
  /*第四面飞控平放，Y轴正向朝着正下方，Y axis is about -1g,X、Z is about 0g*/
  /*第五面飞控平放，Y轴正向朝着正上方，Y axis is about 1g,X、Z is about 0g*/
  /*第六面飞控平放，Z轴正向朝着正下方，Z axis is about -1g,X、Y is about 0g*/
  float  X, Y, Z;
  delay_ms(10);
  if (Acc_Cal_Falg== 0)
  {
	  X = Acce_Correct[0] * ACCEL_TO_1G;
	  Y = Acce_Correct[1] * ACCEL_TO_1G;
	  Z = Acce_Correct[2] * ACCEL_TO_1G;
	  sprintf(print_buff, "[7]检测中 Gyro_Length:%5.5f  X:%2.3f  Y:%2.3f  Z:%2.3f  计数 %3d %3d %3d %3d %3d %3d  完成情况 %3d %3d %3d %3d %3d %3d \n", 
							Gyro_Length, X, Y, Z,
							Acc_det_cnt[0], Acc_det_cnt[1], Acc_det_cnt[2], Acc_det_cnt[3], Acc_det_cnt[4], Acc_det_cnt[5], 
							Accel_Calibration_Finished[0], Accel_Calibration_Finished[1], Accel_Calibration_Finished[2], Accel_Calibration_Finished[3], Accel_Calibration_Finished[4], Accel_Calibration_Finished[5]);
	  Consol_write(print_buff);
	  for (i = 0; i < 6; i++)
	  {
		  if ((X<Acc_det_mat[i][0]&& X>Acc_det_mat[i][1]) && (Y< Acc_det_mat[i][2] && Y>Acc_det_mat[i][3]) && (Z < Acc_det_mat[i][4] && Z>Acc_det_mat[i][5]) && Gyro_Length < 5)
		  {
			  Acc_det_cnt[i]++;
		  }
		  else
		  {
			  Acc_det_cnt[i] = 0;
		  }
	  }
	  for (i = 0; i < 6; i++)
	  {
		  if (Acc_det_cnt[i] > 30)
		  {
			  if (Accel_Calibration_Finished[i] == 1)
			  {
				  Consol_write("此面以校准完成 请换面！！\n");
				  Acc_det_cnt[0] = 0;
				  Acc_det_cnt[1] = 0;
				  Acc_det_cnt[2] = 0;
				  Acc_det_cnt[3] = 0;
				  Acc_det_cnt[4] = 0;
				  Acc_det_cnt[5] = 0;
			  }
			  else
			  {
				  Consol_write("检测到静止 正在校准中请保持静止\n");
				  Acc_Cal_Falg = 1;
				  flight_direction = i;
				  Acc_det_cnt[0] = 0;
				  Acc_det_cnt[1] = 0;
				  Acc_det_cnt[2] = 0;
				  Acc_det_cnt[3] = 0;
				  Acc_det_cnt[4] = 0;
				  Acc_det_cnt[5] = 0;
			  }
		  }
	  }
  }

  if (Acc_Cal_Falg == 1)
  {
	  sprintf(print_buff, "[%d]正在校准第(%d)面:%d Gyro_Length:%5.5f  X:%2.3f  Y:%2.3f  Z:%2.3f\n ", flight_direction, flight_direction, Acc_cal_cnt,Gyro_Length, Acce_Correct[0] * ACCEL_TO_1G, Acce_Correct[1] * ACCEL_TO_1G, Acce_Correct[2] * ACCEL_TO_1G);
	  Consol_write(print_buff);
	  if (Gyro_Length <= 20.0f)//通过陀螺仪模长来确保机体静止
	  {
		  for (j = 0; j < 3; j++) {
			  Acce_sample_sum[j] += Acce_Correct[j] * ACCEL_TO_1G;//加速度计转化为1g量程下
		  }
		  Acc_cal_cnt++;
	  }
	  else
	  {
		  Acc_cal_cnt = 0;
		  Acc_Cal_Falg = 0;
		  Consol_write("单面数据采集失败  正在重新检测\n");
	  }

	  if (Acc_cal_cnt >= 1000)
	  {
		  Consol_write("单面数据采集成功\n");
		  Acc_Cal_Falg = 0;
		  acce_sample[flight_direction].x = Acce_sample_sum[0] / Acc_cal_cnt; //保存对应面的加速度计量
		  acce_sample[flight_direction].y = Acce_sample_sum[1] / Acc_cal_cnt; //保存对应面的加速度计量
		  acce_sample[flight_direction].z = Acce_sample_sum[2] / Acc_cal_cnt; //保存对应面的加速度计量
		  Accel_Calibration_Finished[flight_direction] = 1;//对应面校准完成标志位置1
		  Acce_sample_sum[0] = 0;
		  Acce_sample_sum[1] = 0;
		  Acce_sample_sum[2] = 0;
		  Acc_cal_cnt = 0;
	  }
  }
  

  //6面全部校准完毕
  if((Accel_Calibration_Finished[0] &Accel_Calibration_Finished[1] &Accel_Calibration_Finished[2]
          &Accel_Calibration_Finished[3] &Accel_Calibration_Finished[4] &Accel_Calibration_Finished[5]))
  {
	 
    Cal_Flag=Calibrate_accel(acce_sample,&new_offset, &new_scales);//将所得6面数据
    for(i=0;i<6;i++)
    {
      Accel_Calibration_Finished[i]=0;//对应面标志位清零
    }
    if(Cal_Flag==TRUE)//加速度计校准成功
    {

		Consol_write("校准完成\n");
		sprintf(print_buff, "X_offset:%5.5f Y_offset:%5.5f  Z_offset:%5.5f \n", new_offset.x, new_offset.y, new_offset.z);		Consol_write(print_buff);
		sprintf(print_buff, "X_scales:%5.5f Y_scales:%5.5f  Z_scales:%5.5f \n", new_scales.x, new_scales.y, new_scales.z);		Consol_write(print_buff);
		Consol_write("写入参数中............\n");
		WriteFlashParameter_Three(ACCEL_X_OFFSET,new_offset.x, new_offset.y,new_offset.z,&Table_Parameter);
		WriteFlashParameter_Three(ACCEL_X_SCALE,new_scales.x, new_scales.y, new_scales.z,&Table_Parameter);
		Consol_write("写入完成  写入的数据：\n");
		IMU_Acc_Cal_Init();//读取写入参数
		sprintf(print_buff, "X_offset:%5.5f Y_offset:%5.5f  Z_offset:%5.5f \n", IMU_B[0], IMU_B[1], IMU_B[2]);		Consol_write(print_buff);
		sprintf(print_buff, "X_scales:%5.5f Y_scales:%5.5f  Z_scales:%5.5f \n", IMU_K[0], IMU_K[1], IMU_K[2]);		Consol_write(print_buff);

	  ACC_Calibration_Flag = 0;
	  Return_main_flag = 1;
    }
    else//加速度计校准失败
    {
		Consol_write("校准失败\n");
		ACC_Calibration_Flag = 0;
		Return_main_flag = 1;
    }
    return TRUE;
  }
  return FALSE;
}



typedef struct
{
  uint8_t accel_off;
  uint8_t accel_scale;
  uint8_t mag;
}Parameter_Flag;

Parameter_Flag Parameter_Read_Flag;

u8 IMU_Acc_Cal_Init(void)
{
	u8 success=TRUE;
  /************加速度计零偏与标度值*******/
  Parameter_Read_Flag.accel_off		=ReadFlashParameterThree(ACCEL_X_OFFSET,&Accel_Offset_Read.x, &Accel_Offset_Read.y,&Accel_Offset_Read.z);
  Parameter_Read_Flag.accel_scale	=ReadFlashParameterThree(ACCEL_X_SCALE,&Accel_Scale_Read.x,&Accel_Scale_Read.y,&Accel_Scale_Read.z);
  // sanity check scale
  if(ABS(Accel_Scale_Read.x-1.0f)>0.5 || ABS(Accel_Scale_Read.y-1.0f)>0.5f || ABS(Accel_Scale_Read.z-1.0f)>0.5f)
	success = FALSE; 
  // sanity check offsets (3.5 is roughly 3/10th of a G, 5.0 is roughly half a G)
  if(ABS(Accel_Offset_Read.x) > 5.0f|| ABS(Accel_Offset_Read.y) > 5.0f|| ABS(Accel_Offset_Read.z) > 5.0f)
    success = FALSE;
  //Flash内数据正常，更新加速度校正值
  if(success==TRUE &&Parameter_Read_Flag.accel_off!=0x00&&Parameter_Read_Flag.accel_scale!=0x00)
  {
	  IMU_B[0]=Accel_Offset_Read.x;
	  IMU_B[1]=Accel_Offset_Read.y;
	  IMU_B[2]=Accel_Offset_Read.z;
	  IMU_K[0]=Accel_Scale_Read.x;
	  IMU_K[1]=Accel_Scale_Read.y;
	  IMU_K[2]=Accel_Scale_Read.z;
  }

  return success;
}
/************加速度计6面矫正结束***********************/


#define  RC_TOP_DEFAULT       1600
#define  RC_BUTTOM_DEFAULT    1400
#define  RC_MIDDLE_DEFAULT    1500
#define  RC_DEADBAND_DEFAULT  20
#define  RC_DEADBAND_PERCENT  0.1
#define  RC_RESET_DEFAULT  1500

Vector_RC  RC_Calibration[8]={0};
uint8_t RC_Read_Flag[8] = { 0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 };
void RC_Calibration_Init()
{
  uint16_t i=0;
  float max_read[8]={0},min_read[8]={0};


#ifdef Reload_RC_Calibration
  WriteFlashParameter_Two(RC_CH1_MAX + 2 * i, RC_Calibration[i].max, RC_Calibration[i].min, &Table_Parameter);
#endif
  
  for(i=0;i<8;i++)
  {
    RC_Read_Flag[i]=ReadFlashParameterTwo(RC_CH1_MAX+2*i,&max_read[i],&min_read[i]);
  }
  
  if(RC_Read_Flag[0]!=0x00
     &&RC_Read_Flag[1]!=0x00
       &&RC_Read_Flag[2]!=0x00
         &&RC_Read_Flag[3]!=0x00
           &&RC_Read_Flag[4]!=0x00
             &&RC_Read_Flag[5]!=0x00
               &&RC_Read_Flag[6]!=0x00
                 &&RC_Read_Flag[7]!=0x00)//flash中存在数据
  {
    for(i=0;i<8;i++)
    {
      RC_Calibration[i].max=max_read[i];
      RC_Calibration[i].min=min_read[i];
      RC_Calibration[i].middle=(max_read[i]+min_read[i])/2;
      RC_Calibration[i].deadband=(uint16_t)((max_read[i]-min_read[i])*RC_DEADBAND_PERCENT);
    }
  }
  else//flash中不存在数据
  {
    for(i=0;i<8;i++)
    {
      RC_Calibration[i].max=RC_TOP_DEFAULT;
      RC_Calibration[i].min=RC_BUTTOM_DEFAULT;
      RC_Calibration[i].middle=RC_MIDDLE_DEFAULT;
      RC_Calibration[i].deadband=RC_DEADBAND_DEFAULT;
    }
  }
}

void RC_Calibration_RESET()
{
  uint16_t i=0;
  for(i=0;i<8;i++)
  {
    RC_Calibration[i].max=RC_RESET_DEFAULT;
    RC_Calibration[i].min=RC_RESET_DEFAULT;
  }
}


uint8_t RC_Calibration_Trigger_Flag=0;
void RC_Calibration_Trigger(void)
{
  RC_Calibration_Init();//直接从flash里面（或者DEFAULT值）获取遥控器行程输出
  RC_Calibration_Trigger_Flag = 0;
}
/***************************************************
函数名: void RC_Calibration_Check(uint16 *rc_date)
说明:	遥控器行程校准检测，涉及内部Flash读、写操作
入口:	无
出口:	无
备注:	结束时需按下按键，才会保存
注释者：飞凡小哥
****************************************************/
u16 RC_Calibration_cnt = 0;


Testime RC_Calibration_Delta;
u8 RC_Calibration_Check(uint16 *rc_date)
{
  uint16_t i=0;
  RC_Calibration_cnt = 0;


  for (i = 0; i < 8; i++)
  {
	  RC_Calibration[i].max = 1600;
	  RC_Calibration[i].min = 1400;
	  RC_Calibration[i].middle = 1500;
	  RC_Calibration[i].deadband = 20;
  }

  while (1)
  {
	  RC_Calibration_cnt++;
	  
	  
	  if (RC_Calibration_cnt > 500)
		  break;
	  
	  sprintf(print_buff, "遥控器数据[%d] %d %d %d %d %d %d %d %d\n\0\0\0",
		  RC_Calibration_cnt, rc_date[0], rc_date[1], rc_date[2], rc_date[3], rc_date[4], rc_date[5], rc_date[6], rc_date[7]);
	  delay_ms(20); 
	  Consol_write(print_buff);


	  for (i = 0; i < 8; i++)
	  {
		  //计算最大行程值
		  if (rc_date[i] > RC_Calibration[i].max+2)
		  {
			  RC_Calibration[i].max = rc_date[i];
			  RC_Calibration_cnt = 0;
		  }

		  //计算最小行程值
		  if (rc_date[i] < RC_Calibration[i].min-2)
		  {
			  RC_Calibration[i].min = rc_date[i];
			  RC_Calibration_cnt = 0;
		  }
		  RC_Calibration[i].middle = (uint16_t)((RC_Calibration[i].max + RC_Calibration[i].min) / 2);//行程中位
		  RC_Calibration[i].deadband = (uint16_t)((RC_Calibration[i].max - RC_Calibration[i].min)*RC_DEADBAND_PERCENT);//设置满量程的百分之RC_DEADBAND_PERCENT为中位死区
	  }   
  }
	  sprintf(print_buff, "行程数据：最大值 %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f\n\0\0\0",
		  RC_Calibration[0].max, RC_Calibration[1].max, RC_Calibration[2].max, RC_Calibration[3].max,
		  RC_Calibration[4].max, RC_Calibration[5].max, RC_Calibration[6].max, RC_Calibration[7].max);
	  Consol_write(print_buff);
	  delay_ms(100);
	  sprintf(print_buff, "行程数据：最小值 %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f\n\0\0\0",
		  RC_Calibration[0].min, RC_Calibration[1].min, RC_Calibration[2].min, RC_Calibration[3].min,
		  RC_Calibration[4].min, RC_Calibration[5].min, RC_Calibration[6].min, RC_Calibration[7].min);
	  Consol_write(print_buff);
	  
 

      for(i=0;i<8;i=i+1)
      {
        WriteFlashParameter_Two(RC_CH1_MAX+2*i,RC_Calibration[i].max,RC_Calibration[i].min, &Table_Parameter);
      }
    

  return TRUE;
}

void Reset_RC_Calibartion(uint8_t Type)
{
  if(Type==1)  
  {
    RC_Calibration_Trigger_Flag=0;
    Key_Right_Release=0;
  }
}


float ESC_Calibration_Flag=0;
#define Thr_Chl_Num  2
void ESC_HardWave_Init()//只初始化校准电调的必要资源
{

  NVIC_InitTypeDef NVIC_InitStructure;//定义NVIC初始化结构体
  SystemInit();
  delay_init(72);
  //OLED_Init();
  TIM4_Configuration_Cnt();//TIM2程序计时定时器
  PPM_Init();//PPM遥控器接收初始化
  //SBUS_USART5_Init();//串口5、SBUS解析

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//优先级组别2，具体参见misc.h line80
  //飞控系统定时器
  NVIC_InitStructure.NVIC_IRQChannel =TIM4_IRQn ;//计数定时器
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  //PPM接收机
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 

  ////SBUS解析串口 
  //NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn; //中断号；
  //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //抢占优先级；
  //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //响应优先级；
  //NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  //NVIC_Init(&NVIC_InitStructure);
  //delay_ms(100);  

  WriteFlashParameter(ESC_CALIBRATION_FLAG, 0,  &Table_Parameter);//写零避免下次上电再次进入
  delay_ms(500);
  delay_ms(500);
  delay_ms(500);
  delay_ms(500);
  PWM_Init();//PWM初始化—TIM4

  while (1)
  {
	  //NRF24L01_RC();		//遥控器查询接收，非中端方式
	  //ESC_Calibration();
	  PWM_Set(PPM_Databuf[Thr_Chl_Num], PPM_Databuf[Thr_Chl_Num], PPM_Databuf[Thr_Chl_Num],PPM_Databuf[Thr_Chl_Num]);

  }
}


uint8_t Check_Calibration_Flag(void)
{
  uint8_t cal_flag=0x00; 
  if(Key_Right_Release==1)      cal_flag|=0x01;//遥控器校准
  if(Accel_Calibration_Flag==1) cal_flag|=0x02;//加速度计校准
  return cal_flag;
}



void Horizontal_Calibration_Init(void)
{    
  ReadFlashParameterOne(PITCH_OFFSET,&Pitch_Offset);
  ReadFlashParameterOne(ROLL_OFFSET,&Roll_Offset);
}

/**********************************
函数名：void IMU_Calibration(void)
说明：MPU6050标定
入口：无
出口：无
备注：用来开机时设定陀螺仪的零值
**********************************/

s32 g_Gyro_xoffset = 0, g_Gyro_yoffset = 0, g_Gyro_zoffset = 0;
float  X_w_off = 0, Y_w_off = 0, Z_w_off = 0;

#define GYRO_Calibration_Times 200
void GYRO_Calibration(void)
{
	u32 GYRO_Calibration_times_cnt=0;
	while(GYRO_Calibration_times_cnt<GYRO_Calibration_Times)
	{
		//读取MPU6050的值
		ImuSensor_ReadReg_BuffAll();
		g_Gyro_xoffset += MPU_Data.GyroX;			
		g_Gyro_yoffset += MPU_Data.GyroY;			
		g_Gyro_zoffset += MPU_Data.GyroZ;

		MPitch_Gyro = MPU_Data.GyroX*GYRO_CALIBRATION_COFF;
		MRoll_Gyro = MPU_Data.GyroY*GYRO_CALIBRATION_COFF;
		MYaw_Gyro = MPU_Data.GyroZ*GYRO_CALIBRATION_COFF; 

		//角速度模长
		if (sqrt(MYaw_Gyro*MYaw_Gyro + MPitch_Gyro*MPitch_Gyro + MRoll_Gyro*MRoll_Gyro) > 1.0f)
		{
			GYRO_Calibration_times_cnt = 0;
			g_Gyro_xoffset = 0;	g_Gyro_yoffset = 0;	g_Gyro_zoffset = 0;
		}
		GYRO_Calibration_times_cnt++;
		delay_ms(5);
	}

	//得到标定偏移
	X_w_off = ((float)g_Gyro_xoffset / GYRO_Calibration_Times); 
	Y_w_off = ((float)g_Gyro_yoffset / GYRO_Calibration_Times);
	Z_w_off = ((float)g_Gyro_zoffset / GYRO_Calibration_Times);

}
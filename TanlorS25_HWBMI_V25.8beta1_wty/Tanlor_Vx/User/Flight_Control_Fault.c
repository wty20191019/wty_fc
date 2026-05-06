/*----------------------------------------------------------------------------------------------------------------------/
*               本程序只供购买者学习使用，版权著作权属于无名科创团队，无名科创团队将飞控程序源码提供给购买者，
*               购买者要为无名科创团队提供保护，未经作者许可，不得将源代码提供给他人，不得将源代码放到网上供他人免费下载， 
*               更不能以此销售牟利，如发现上述行为，无名科创团队将诉之以法律解决！！！
-----------------------------------------------------------------------------------------------------------------------/
*               生命不息、奋斗不止；前人栽树，后人乘凉！！！
*               开源不易，且学且珍惜，祝早日逆袭、进阶成功！！！
*               学习优秀者，简历可推荐到DJI、ZEROTECH、XAG、AEE、GDU、AUTEL、EWATT、HIGH GREAT等公司就业
*               求职简历请发送：15671678205@163.com，需备注求职意向单位、岗位、待遇等
*               无名科创开源飞控QQ群：540707961
*               CSDN博客：http://blog.csdn.net/u011992534
*               优酷ID：NamelessCotrun无名小哥
*               B站教学视频：https://space.bilibili.com/67803559/#/video
*               客户使用心得、改进意见征集贴：http://www.openedv.com/forum.php?mod=viewthread&tid=234214&extra=page=1
*               淘宝店铺：https://shop348646912.taobao.com/?spm=2013.1.1000126.2.5ce78a88ht1sO2
*               百度贴吧:无名科创开源飞控
*               修改日期:2018/8/30
*               版本：慧飞者——V1.0.1
*               版权所有，盗版必究。
*               Copyright(C) 武汉科技大学无名科创团队 2017-2025
*               All rights reserved
----------------------------------------------------------------------------------------------------------------------*/
#include "Headfile.h"
#include "Flight_Control_Fault.h"

Flight_Control_Fault_State  NCQ_Fault;


#define DIVIDER_RES_MAX  100//分压电阻1
#define DIVIDER_RES_MIN  10//分压电阻2
#define DIVIDER_RES_SCALE (DIVIDER_RES_MAX+DIVIDER_RES_MIN)/DIVIDER_RES_MIN

float Battery_Valtage=0;
void Battery_Fault_Check(void)
{
  static uint16_t battery_fault_cnt=0;
  Battery_Valtage=(DIVIDER_RES_SCALE*ADC_Value[1]*3.3f)/4096;
  if(Battery_Valtage<11.2)
  {
    battery_fault_cnt++;
    if(battery_fault_cnt>=100)//持续100*5ms=500ms
    {
      NCQ_Fault.Low_Voltage_Fault_Flag=TRUE;
    }
  }
  else
  {
    battery_fault_cnt/=2;
    if(battery_fault_cnt==0)  NCQ_Fault.Low_Voltage_Fault_Flag=FALSE; 
  }
}

void Baro_Fault_Check(float baropress)
{
  static uint16_t baro_fault_cnt=0;
  static float last_baropress=0;
  static uint16_t baro_gap_cnt=0;
  baro_gap_cnt++;
  if(baro_gap_cnt>=40)//每200ms检测一次，因为气压计更新周期大于5ms
  {
    baro_gap_cnt=0;
    if(last_baropress==baropress)
    {
      baro_fault_cnt++;
      if(baro_fault_cnt>5)  NCQ_Fault.Baro_Fault_Flag=TRUE;   
    }
    else
    {
      baro_fault_cnt/=2;
      if(baro_fault_cnt==0)  NCQ_Fault.Baro_Fault_Flag=FALSE; 
    }
    last_baropress=baropress; 
  }
}

void Accel_Fault_Check(Vector3i accel)
{
  static uint16_t accel_fault_cnt=0;
  static Vector3i last_accel={0};
  if(last_accel.x==accel.x
     &&last_accel.y==accel.y
       &&last_accel.z==accel.z)
  {
    accel_fault_cnt++;
    if(accel_fault_cnt>20)  NCQ_Fault.Accel_Fault_Flag=TRUE;   
  }
  else
  {
    accel_fault_cnt/=2;
    if(accel_fault_cnt==0)  NCQ_Fault.Accel_Fault_Flag=FALSE; 
  }
  last_accel=accel; 
}


void Gyro_Fault_Check(Vector3i gyro)
{
  static uint16_t gyro_fault_cnt=0;
  static Vector3i last_gyro={0};
  if(last_gyro.x==gyro.x
     &&last_gyro.y==gyro.y
       &&last_gyro.z==gyro.z)
  {
    gyro_fault_cnt++;
    if(gyro_fault_cnt>20)  NCQ_Fault.Gyro_Fault_Flag=TRUE;   
  }
  else
  {
    gyro_fault_cnt/=2;
    if(gyro_fault_cnt==0)  NCQ_Fault.Gyro_Fault_Flag=FALSE; 
  }
  last_gyro=gyro; 
}


void Compass_Fault_Check(Vector3i compass)
{
  static uint16_t compass_fault_cnt=0;
  static Vector3i last_compass={0};
  static uint16_t compass_gap_cnt=0;
  compass_gap_cnt++;
  if(compass_gap_cnt>=40)//每200ms检测一次，因为磁力计更新周期大于5ms
  {
    compass_gap_cnt=0;
    if(last_compass.x==compass.x
       &&last_compass.y==compass.y
         &&last_compass.z==compass.z)
    {
      compass_fault_cnt++;
      if(compass_fault_cnt>10)  NCQ_Fault.Compass_Fault_Flag=TRUE;   
    }
    else
    {
      compass_fault_cnt/=2;
      if(compass_fault_cnt==0)  NCQ_Fault.Compass_Fault_Flag=FALSE; 
    }
    last_compass=compass;
  }
}



void Ultrasonic_Fault_Check(float ultrasonic)
{
  static uint16_t ultrasonic_fault_cnt=0;
  static float last_ultrasonic;
  if(last_ultrasonic==ultrasonic)
  {
    ultrasonic_fault_cnt++;
    if(ultrasonic_fault_cnt>20)  NCQ_Fault.Ultrasonic_Fault_Flag=TRUE;   
  }
  else
  {
    ultrasonic_fault_cnt/=2;
    if(ultrasonic_fault_cnt==0)  NCQ_Fault.Ultrasonic_Fault_Flag=FALSE; 
  }
  last_ultrasonic=ultrasonic; 
}



void Opticalflow_Fault_Check(int16_t opticalflow_x,int16_t opticalflow_y)
{
  static uint16_t opticalflow_fault_cnt=0;
  static uint16_t opticalflow_fault_zero_cnt=0; 
  static int16_t last_opticalflow_x,last_opticalflow_y;
  if(last_opticalflow_x==opticalflow_x
     &&last_opticalflow_y==opticalflow_y
       &&last_opticalflow_x!=0
         &&last_opticalflow_y!=0)
  {
    opticalflow_fault_cnt++;
    if(opticalflow_fault_cnt>20)  NCQ_Fault.Opticalflow_Fault_Flag=TRUE;   
  }
  else if(last_opticalflow_x==opticalflow_x
          &&last_opticalflow_y==opticalflow_y
            &&last_opticalflow_x==0
              &&last_opticalflow_y==0)
  {
    opticalflow_fault_zero_cnt++;
    if(opticalflow_fault_zero_cnt>100)  NCQ_Fault.Opticalflow_Fault_Flag=TRUE;   
  }
  else
  {
    opticalflow_fault_cnt/=2;
    opticalflow_fault_zero_cnt/=2;
    if(opticalflow_fault_cnt==0)  NCQ_Fault.Opticalflow_Fault_Flag=FALSE; 
  }
  last_opticalflow_x=opticalflow_x;
  last_opticalflow_y=opticalflow_y;
}


Vector3i Accel,Gyro,Compass;
void Flight_Control_Fault_ALL(void)
{
  Battery_Fault_Check();
  Baro_Fault_Check(Altitude_Estimate);
  Accel_Fault_Check(Accel);
  Gyro_Fault_Check(Gyro);
  Compass_Fault_Check(Compass);
}



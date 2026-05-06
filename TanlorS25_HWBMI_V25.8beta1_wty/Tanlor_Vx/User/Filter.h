/*----------------------------------------------------------------------------------------------------------------------/
*               本程序只供购买者学习使用，版权著作权属于飞凡创翼团队，飞凡创翼团队将飞控程序源码提供给购买者，
*               购买者要为飞凡创翼团队提供保护，未经作者许可，不得将源代码提供给他人，不得将源代码放到网上供他人免费下载，
*               更不能以此销售牟利，如发现上述行为，飞凡创翼团队将诉之以法律解决！！！
-----------------------------------------------------------------------------------------------------------------------/
*               生命不息、奋斗不止；前人栽树，后人乘凉！！！
*               开源不易，且学且珍惜，祝早日逆袭、进阶成功！！！
*               飞凡创翼开源飞控QQ群：
*               CSDN博客：
*               优酷ID：
*               B站教学视频：
*               淘宝店铺：
*               百度贴吧:
*               修改日期:2020/7/19
*               版本：Baby-Z V1.1
*               版权所有，盗版必究。
*               Copyright(C) 中国民航大学飞凡创翼团队 2017-2025
*               All rights reserved
----------------------------------------------------------------------------------------------------------------------*/

#ifndef _FILTER_H_
#define _FILTER_H_


typedef struct
{
 float Input_Butter[3];
 float Output_Butter[3];
}Butter_BufferData;


typedef struct
{
 float Output_Butter[3];
}Notch_Filter_BufferData;


typedef struct
{
  float a[3];
  float b[3];
}Butter_Parameter;


extern Butter_BufferData Butter_5HZ_Buffer_Acce[3];

extern float Filter_data[4];
extern Butter_Parameter Bandstop_Filter_Parameter_30_98,Bandstop_Filter_Parameter_30_94;

extern Butter_Parameter Butter_80HZ_Parameter_Acce,Butter_60HZ_Parameter_Acce,Butter_51HZ_Parameter_Acce,
                 Butter_30HZ_Parameter_Acce,Butter_20HZ_Parameter_Acce,Butter_15HZ_Parameter_Acce,
                 Butter_10HZ_Parameter_Acce,Butter_5HZ_Parameter_Acce,Butter_2HZ_Parameter_Acce;


float BPF_Butterworth(float curr_input, Butter_BufferData *Buffer, Butter_Parameter *Parameter);
float Butterworth_Filter(float curr_inputer, Butter_BufferData *Buffer, Butter_Parameter *Parameter);


float set_lpf_alpha(int16_t cutoff_frequency, float time_step);
float LPButterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter);


float set_lpf_alpha(int16_t cutoff_frequency, float time_step);
void Set_Cutoff_Frequency(float sample_frequent, float cutoff_frequent,Butter_Parameter *LPF);
void Butterworth_Parameter_Init(void);
void Test_Filter(void);


#endif



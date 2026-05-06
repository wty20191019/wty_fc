
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
#ifndef _FFCY_MENU_H_
#define _FFCY_MENU_H_

#include "stm32f4xx.h"


typedef struct Key_input
{
	u8 Key_plus;
	u8 Key_up;
	u8 key_del;
	u8 key_down;
} sadasd;

extern int16_t Page_Number;
extern int My_page_number;
extern u8 key_flag;
extern int Page_Danger_Data2;
void QuadInit(u8 Head_line);
void Quad_DynamicParament_Show(u16 Head_line);

void Load_saved_sdk_parament(void);
void Save_SDK_Parameter(void);
#endif
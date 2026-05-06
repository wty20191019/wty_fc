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
#ifndef _KEY_H_
#define _KEY_H_


extern u8 AUTO_FLY_FLAG;

#define Key_Up		GPIO_ReadInputDataBit(GPIOA , GPIO_Pin_1)
#define Key_Down	GPIO_ReadInputDataBit(GPIOA , GPIO_Pin_0)
#define Key_Del		GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_2)
#define Key_Plus	GPIO_ReadInputDataBit(GPIOA , GPIO_Pin_4)
#define Key_Plus2	GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_5)

void Key_Init(void);
u8 Key_Scan(uint8_t release);

extern uint8_t Key_Right_Release;


#endif


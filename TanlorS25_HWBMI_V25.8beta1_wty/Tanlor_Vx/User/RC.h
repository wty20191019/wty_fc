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
#ifndef __RC_H
#define __RC_H

#define  Scale_Pecent_Max  0.75   //最大解锁幅值量程因子
#define  Pit_Rol_Max 30           //最大俯仰、横滚期望
#define  Yaw_Max     100          //最大偏航期望


extern u8 Data_buf[32];
extern uint8 RC_Control[32];
extern uint16 QuadData[2];
extern uint16 Throttle_Control,Last_Throttle_Control;
extern int16 Pitch_Control,Roll_Control,Yaw_Control;
extern int16 Target_Angle[2];
extern uint8 QuadRemoteFlag[4];
extern int16 RC_NewData[4];
extern int16 RC3_Origal_Value;   //第三通道原始数据
extern uint16_t PPM_LPF_Databuf[4];
//-----函数声明-----//

void RC_Statemachine(void);

#define Lock_Controler  0
#define Unlock_Controler  1
extern uint16 Controler_State;
extern int16_t Throttle_Rate;
extern uint16_t Auto_ReLock_Cnt;//自动上锁计数器
extern uint16_t Unlock_Makesure_Cnt,Lock_Makesure_Cnt;

#endif

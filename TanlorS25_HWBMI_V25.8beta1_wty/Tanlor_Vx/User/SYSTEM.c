#include "Headfile.h"

USB_OTG_CORE_HANDLE    USB_OTG_dev;
extern void Parament_init_check_andLoadDafault();
u8 Moto_Ride_CNT = 0;


//电调校准及电机测试
//上电时若遥控器油门置高，则进入电调校准
//上电时若俯仰轴置低则进入电机怠速测试，1-4号电机会依次怠速转动
void Motor_Calibration()
{
	if (PPM_Databuf[2] > 1800)
	{
		PWM_Set(PPM_Databuf[2], PPM_Databuf[2], PPM_Databuf[2], PPM_Databuf[2]);
		TIM_Cmd(TIM3, ENABLE);  //使能TIM14
		while (1)
		{
			PWM_Set(PPM_Databuf[2], PPM_Databuf[2], PPM_Databuf[2], PPM_Databuf[2]);
			if (PPM_Databuf[3] > 1800)
			{
				TIM_Cmd(TIM3, DISABLE);  //使能TIM14 
				break;
			}
		}
	}

	if (PPM_Databuf[1] > 1800)
	{
		TIM_Cmd(TIM3, ENABLE);  //使能TIM14
		PWM_Set(Thr_Min, Thr_Min, Thr_Min, Thr_Min);
		delay_ms(1000); delay_ms(1000); delay_ms(1000); delay_ms(1000);
		while (1)
		{
			Moto_Ride_CNT++;
			if (Moto_Ride_CNT > 4)
				Moto_Ride_CNT = 1;
			if (Moto_Ride_CNT == 4) { PWM_Set(Thr_Idle, Thr_Min, Thr_Min, Thr_Min); 	delay_ms(1000);	delay_ms(1000); }
			if (Moto_Ride_CNT == 3) { PWM_Set(Thr_Min, Thr_Idle, Thr_Min, Thr_Min);  	delay_ms(1000); delay_ms(1000); }
			if (Moto_Ride_CNT == 2) { PWM_Set(Thr_Min, Thr_Min, Thr_Idle, Thr_Min);  	delay_ms(1000);	delay_ms(1000); }
			if (Moto_Ride_CNT == 1) { PWM_Set(Thr_Min, Thr_Min, Thr_Min, Thr_Idle);  	delay_ms(1000);	delay_ms(1000); }

			if (PPM_Databuf[1] < 1200)
			{
				PWM_Set(Thr_Min, Thr_Min, Thr_Min, Thr_Min);
				TIM_Cmd(TIM3, DISABLE);  //使能TIM14 
				break;
			}
		}
	}
}
void Board_Init(void)
{
	/*************基本功能初始化***************/
	SystemInit();					//系统时钟初始化
	delay_init(168);				//滴答延时初始化

	USBD_Init(&USB_OTG_dev, USB_OTG_FS_CORE_ID, &USR_desc, &USBD_CDC_cb, &USR_cb);
	/************中断优先级设置***************/
	NVIC_Configuration();
	TIM4_Configuration_Cnt();    //TIM2程序计时定时器

	PPM_Init();					//PPM遥控器接收初始化
	
	/************硬件MPU6050初始化***************/
	ImuSensor_Init();
	GYRO_Calibration();           //陀螺仪零偏标定
	/************PWM部分     ***************/
	PWM_Init();					//PWM初始化(不输出)
	Motor_Calibration();		//电调校准及电机测试

	Parament_init_check_andLoadDafault();
	/*************串口初始化***************/
	RingBuff_Init(&Ground_Station_Ringbuf);
	//USART1_Init(921600);
	//DMA_Use_USART1_Tx_Init();
	DMA_USART1_Init(921600);
	DMA_USART4_Init(115200);
	DMA_USART3_Init(115200);
	DMA_USART2_Init(115200);
	DMA_USART6_Init(115200);

	/*************外设初始化***************/
	VL53LX_init();				//激光测距初始化
	PMW3901_Init();				//光流初始化
	/***********参数初始化***************/
	IMU_Acc_Cal_Init();						//加速度计校准值初始化
	Quaternion_Init();						//初始四元数初始化
	Butterworth_Parameter_Init();			//滤波器参数初始化
	RC_Calibration_Trigger();				//遥控器标定参数初始化
	Horizontal_Calibration_Init();			//OFFSET水平标定参数初始化
	PID_Paramter_Init_With_Flash();			//PID控制器初始化，可以通过地面站修改参数
	Load_saved_sdk_parament();				//水平标定参数初始化


	PWM_Set(Thr_Min, Thr_Min, Thr_Min, Thr_Min);
	TIM_Cmd(TIM3, ENABLE);  //使能TIM14

	OLED_Init();					//显示屏初始化
	Draw_Logo1();
	Bling_Init();				//指示灯、测试IO初始化
	Key_Init();                   //按键初始化
	LCD_CLS();

	/*************定时器初始化***************/
	TIM2_Configuration();
}




/***************************************************
函数名: void NVIC_Configuration(void)
说明:	中断优先级定义
入口:	无
出口:	无
备注:	系统中断时间在整体设计时，已全部测试过最大开销，
        确保程序运动的稳健性        
注释者：
****************************************************/
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;//定义NVIC初始化结构体
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//优先级组别2，具体参见misc.h line80

  NVIC_InitStructure.NVIC_IRQChannel = OTG_FS_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//抢占优先级0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;//子优先级0
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能通道
  NVIC_Init(&NVIC_InitStructure);//配置  

  //上位机
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
  NVIC_Init(&NVIC_InitStructure);

  //飞控系统定时器 计数定时器
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
  NVIC_Init(&NVIC_InitStructure);

  //PPM接收机
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;//外部中断0
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//抢占优先级0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x04;//子优先级3
  NVIC_Init(&NVIC_InitStructure);//配置

  //DMA中断优先级
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
  NVIC_Init(&NVIC_InitStructure);

  //Camera
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//串口2中断通道
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;//抢占优先级1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;		//子优先级0
  NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

  //T265
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
  NVIC_Init(&NVIC_InitStructure);

  //UWB
  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;
  NVIC_Init(&NVIC_InitStructure);

  //dimianzhan
  NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x04;
  NVIC_Init(&NVIC_InitStructure);

  //飞控任务调度定时器
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
  NVIC_Init(&NVIC_InitStructure);


}

float Data_True_data = 0;
u8 Data_true_flag = 0;
void Parament_init_check_andLoadDafault()
{
	//Save_SDK_Parameter();  //重置SDK参数
	//读取机身参数  仅在新机器开始使用时参数不正确 需要重新重置flash参数
	ReadFlashParameterOne(DATA_TRUE_FLAG, &Data_True_data);  //Data_True_FLAG=506
	if (Data_True_data == Para_Version)
		Data_true_flag = 1;

	if (Data_true_flag == 0)
	{
		//写参数有效标志
		WriteFlashParameter(DATA_TRUE_FLAG, Para_Version, &Table_Parameter);

		int i = 0;
		Total_PID_Init();//将PID参数重置为参数Control_Unit表里面参数
		Save_PID_Parameter();//重置PID参数
		Save_SDK_Parameter();  //重置SDK参数
		//重置加速度计参数
		WriteFlashParameter_Three(ACCEL_X_OFFSET, 0, 0, 0, &Table_Parameter);
		WriteFlashParameter_Three(ACCEL_X_SCALE, 1, 1, 1, &Table_Parameter);
		//水平标定参数初始化
		WriteFlashParameter(PITCH_OFFSET, 0.0f, &Table_Parameter);
		WriteFlashParameter(ROLL_OFFSET, 0.0f, &Table_Parameter);
		//重置遥控器参数
		for (i = 0; i < 8; i++)
		{
			RC_Calibration[i].max = 1600;
			RC_Calibration[i].min = 1400;
			RC_Calibration[i].middle = 1500;
			RC_Calibration[i].deadband = 20;
			WriteFlashParameter_Two(RC_CH1_MAX + 2 * i, RC_Calibration[i].max, RC_Calibration[i].min, &Table_Parameter);
		}
		Data_true_flag = 1;
	}

	//ReadFlashParameterOne(ESC_CALIBRATION_FLAG, &ESC_Calibration_Flag);
	ESC_Calibration_Flag = 0;
}





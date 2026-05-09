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

#ifndef _Headfile_H
#define _Headfile_H

#define FALSE 0
#define TRUE 1

#define PI 3.1415926535898
#define CNTLCYCLE  0.005f
#define AHRS_DT  0.005f
#define AT10_MID 1507
#define _YAW    0
#define _PITCH  1
#define _ROLL   2

#define Sampling_Freq 200//200hz


//  陀螺仪 ------------------------------------
#define BMI088 1
#define MPU6050 2
#define ICM42688 3
#define IMU_SENSOR BMI088 

#if IMU_SENSOR==MPU6050
#define AcceMax_1G      4096
#define GRAVITY_MSS     9.80665f
#define GYRO_CALIBRATION_COFF  1000.0f/32768.0f		//1000 deg/s

#elif IMU_SENSOR == BMI088
#define AcceMax_1G      5461.3333f
#define GRAVITY_MSS     9.80665f
#define GYRO_CALIBRATION_COFF  2000.0f/32768.0f       //1000 deg/s
#endif

#define NEW_IMU   //使用新版四元数

/*低通滤波参数*/
#define GYRO_LPF_CUTOFF_FREQ  25
#define ACCEL_LPF_CUTOFF_FREQ 30
//80,30

#define DEG2RAD (PI / 180.0f)
#define RAD2DEG (180.0f / PI)
#define ACCEL_TO_1G     GRAVITY_MSS/AcceMax_1G
#define One_G_TO_Accel  AcceMax_1G/GRAVITY_MSS
//-----------------------------------

#define Int_Sort    (int16_t)
#define ABS(X)  (((X)>0)?(X):-(X))
#define FFCY_MAX(a,b)  ((a)>(b)?(a):(b))
#define FFCY_MIN(a,b)  ((a)<(b)?(a):(b))
/////////////////////////////////////////////////////////////////////////////////////
//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)      ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )
//#define BYTE3(x)  (uint8_t)((x>>24)&0x00ff)
//#define BYTE2(x)  (uint8_t)((x>>16)&0x00ff)
//#define BYTE1(x)  (uint8_t)((x>>8)&0x00ff)
//#define BYTE0(x)  (uint8_t)((x)&0x00ff)


#include "math.h"
#include "stdio.h"
#include "stdint.h"
#include "stm32f4xx.h"
//#include "stm32f4xx_it.h"
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_conf.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_flash.h"
#include "stm32f4xx_spi.h"
//#include "stm32f4xx_adc.h"
#include "stm32f4xx_i2c.h"
#include "copyright.h"
//#include "arm_math.h"
//#include "HW_Config.h"
//#include "Queue.h"
#include "ringbuf.h"

#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usb_conf.h"
#include "usbd_desc.h"
#include "usbd_cdc_vcp.h"
#include "usb_dcd_int.h"


//==========================
#include <pa0_LED_toggle.h>



#include "string.h"

typedef   signed           char int8;
typedef unsigned           char uint8;
typedef unsigned           char byte;
typedef   signed short     int int16;
typedef unsigned short     int uint16;




typedef struct
{
  int32_t x;
  int32_t y;
}Vector2i;

typedef struct
{
  float x;
  float y;
}Vector2f;


typedef struct
{
  float x;
  float y;
  float z;
}Vector3f;

typedef struct
{
	float x[10];
	float y[10];
	float z[10];
}Vector3f_History;

typedef struct
{
  float q0;
  float q1;
  float q2;
  float q3;
}Vector4q;


typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;
}Vector3i;



typedef struct
{
  float E;
  float N;
  float U;
}Vector3_Nav;

typedef struct
{
  float E;
  float N;
}Vector2f_Nav;



typedef struct
{
  int32_t lat;
  int32_t lng;
}Vector2_Nav;



typedef struct
{
  float x;
  float y;
  float z;
}Vector3_Body;


typedef struct
{
  float Pit;
  float Rol;
}Vector2_Ang;


typedef struct
{
  float Pit;
  float Rol;
}Vector2_Body;


typedef struct
{
  float Pit;
  float Rol;
  float Yaw;
}Vector3f_Body;


typedef struct
{
  float North;
  float East;
}Vector2_Earth;

typedef struct
{
  Vector3f a;
  Vector3f b;
  Vector3f c;
}Matrix3f;

typedef struct
{
  float Last_Time;
  float Now_Time;
  float Time_Delta;
  uint16 Time_Delta_INT;//单位ms
}Testime;


typedef struct
{
  float max;
  float min;
  float middle;
  float deadband;
}Vector_RC;


struct Sensor_unHandle
{

	u16 vl53l0x_high;
	s16 OFlow_X;
	s16 OFlow_Y;
	s16 OFlow_X_ADD;
	s16 OFlow_Y_ADD;

};


struct system_TimeCost
{
	Testime Timer_Delta;
	Testime Timer_Cost;
};
extern struct system_TimeCost System_TimeCost;
extern struct sensor_HealthFlag Sensor_HealthFlag;

typedef struct
{
	float Raw_Pressure;
	float Filter_Pressure;
	float Offset_Pressure;
	float High;
	float Last_High;
	float Climbrate;
}_Baro;


typedef struct
{
	uint8_t Mpu_Health;
	uint8_t Baro_Health;
	uint8_t Mag_Health;
	uint8_t Gps_Health;
	uint8_t Hcsr04_Health;
}Sensor_Health;

// 光流数据结构体
typedef struct {
	float flow_x;          // X轴光流速率 (rad/s)
	float flow_y;          // Y轴光流速率 (rad/s)
	float distance;        // 测量距离 (m)
	uint8_t valid;         // 数据有效性 (0x00无效, 0xF5有效)
	uint8_t confidence;    // 置信度 (0-100%)
} OpticalFlowData;

extern Testime Time1_Delta;
extern _Baro Baro_Show;


/*************板载硬件驱动头文件***********************/
#include "SBUS.h"
#include "PPM.h"
#include "FLASH.h"
#include "USART.h"
#include "DELAY.h"
#include "SYSTEM.h"
#include "OLED.h"
#include "My_TIME.h"
#include "PWM.h"
#include "ADC.h"
#include "Time_Cnt.h"
#include "KEY.h"
#include "SPI1.h"
#include "SPI2.h"
#include "RC.H"
#include "DMA_UART1.h"
#include "DMA_UART2.h"
#include "DMA_UART3.h"
#include "DMA_UART4.h"
#include "DMA_UART6.h"



/************IMU传感器头文件**************/
#include "FFCY_NEW_IMUSensor.h"
#include "IMU_NEW.h"
/*************姿态解算+滤波器+惯导+控制算法类头文件***********************/
#include "IMU.h"//姿态解算
#include "PID.h"//PID控制器
#include "FFCY_Parament.h"
#include "Total_Control.h"//飞控总控制器（位置+速度+加速度+姿态）
#include "SINS.h"//惯导融合
#include "Calibration.h"//传感器标定
#include "Filter.h"//传感器滤波
#include "Bling.h"//状态指示

#include "FFCY_OpticalFlow.h"
#include "FFCY_SDK.h"
#include "alt_ekf.h"
#include "control_config.h"
#include "control_althold.h"
#include "Flight_Parameter_Table.h"
#include "Flight_Control_Fault.h"
#include "NamelessCotrun_SDK.h"
#include "FFCY_Position_Ctrl.h"




#include "FFCY_Menu.h"
#include "FFCY_Math.h"
#include "FFCY_vl53l0x.h"
#include "PMW3901.h"
#include "FFCY_ANO_Link.h"

#include "T265.h"

#define Sensor_Selection  0	//T265的yaw给1，其余给0
/*********Mavlink**********************/


#define LTC_LED  GPIO_Pin_12
#define RDY_LED  GPIO_Pin_13
#define GPS_LED  GPIO_Pin_14
#define WORK_LED GPIO_Pin_15


//#define RC_NRF24L01  0
#define RC_PPM  1

typedef struct
{
  uint16 NRF24L01_Okay:1;
  uint16 MPU6050_Okay:1;
  uint16 Mag_Okay:1;
  uint16 Baro_Okay:1;
}Sensor_Okay_Flag;





extern float AirPresure_Altitude;
extern u32 Test_Cnt1[2],Test_Cnt2[2];




#define RECEIVE_BUF_MAX_SIZE 	100   //DMA单次最大搬运数据量
#define TRANSMIT_BUF_MAX_SIZE   100   //DMA单次最大搬运数据量
///定义数据接收结构体
typedef struct _ReceiveBuffer {
	uint8_t Buffer[RECEIVE_BUF_MAX_SIZE];//用于接收DMA搬运的接收数据
	uint16_t Lenth;//接收的数据长度
}ReceiveBuffer_t;
///定义数据发送结构体
typedef struct _TransmitBuffer {
	uint8_t Buffer[TRANSMIT_BUF_MAX_SIZE];//用于接收DMA搬运的发送数据
	uint16_t Lenth;//发送的数据长度
}TransmitBuffer_t;

static ReceiveBuffer_t ReceiveBuffer;//数据接收结构体
static TransmitBuffer_t TransmitBuffer;//数据发送结构体
									   /*DMA接收配置结构体*/
static DMA_InitTypeDef DMA_TransmitInitStruct;

void UART1Init(void);
void DMA_Use_USART1_Tx_Init(void);
void DMA_Use_USART1_Rx_Init(void);
void USART1_IRQFuc(void);
uint8_t deal_irq_rx_end(uint8_t *buf);
void Use_DMA_tx(uint8_t *data, uint16_t size);
void DMA2_Stream7_IRQFuc(void);
void deal_irq_tx_end(void);
void DMA_Send(uint8_t* data, uint32_t writeLen);

extern float Data_True_data;
extern u8 Data_true_flag;
#define Para_Version 96.00f

#endif



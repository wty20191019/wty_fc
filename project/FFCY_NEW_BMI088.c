#include "Headfile.h"
#include "stm32f4xx_i2c.h"
#include "FFCY_NEW_BMI088.h"

#if IMU_SENSOR == BMI088

MPU6050_Data MPU_Data;
Vector3f Accel, Gyro;
float	X_Origion, Y_Origion, Z_Origion;								//经过椭球校正后的三轴加速度量
int16_t Acce_Correct[3] = { 0 };										//用于矫正加速度量，截至频率很低
Vector3f accel_filter, gyro_filter_QU;
Vector3f gyro_nofilter;

float IMU_K[3] = { 1.0,1.0,1.0 };//默认标度(量程)误差
float IMU_B[3] = { 0,0,0 };//默认零位误差

Axis3f Gyro_feedback;



Butter_Parameter Accel_Parameter, Butter_1HZ_Parameter_Acce, Gyro_Parameter, Gyro_Parameter_QU, Ins_Accel_Parameter;
Butter_BufferData Butter_Buffer_SINS[3], Butter_Buffer_Correct[3], gyro_filter_buf[3], gyro_filter_QU_buf[3], accel_filter_buf[3];

u32 IIC_Timeout_Cnt = 0;
u32 IIC_Timeout_Cnt_noTimesClear = 0;
void BMI088_WaitEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT)
{

	uint32_t Timeout = 2000;									//给定超时计数时间
	while (I2C_CheckEvent(I2Cx, I2C_EVENT) != SUCCESS)	//循环等待指定事件
	{
		Timeout--;										//等待时，计数值自减
		if (Timeout == 0)								//自减到0后，等待超时
		{
			IIC_Timeout_Cnt++;
			IIC_Timeout_Cnt_noTimesClear++;
			/*超时的错误处理代码，可以添加到此处*/
			break;										//跳出等待，不等了
		}
	}
}

//指定地址写
void BMI088_WriteReg(uint8_t DevAddress, uint8_t RegAddress, uint8_t Data)
{

	I2C_GenerateSTART(I2C1, ENABLE);//通讯开始
	BMI088_WaitEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT);	//等待事件EV5发生
	I2C_Send7bitAddress(I2C1, DevAddress, I2C_Direction_Transmitter);	//指定设备地址
	BMI088_WaitEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);	//等待事件EV6发生->设置为发送模式
	I2C_SendData(I2C1, RegAddress);	//指定寄存器地址
	BMI088_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING);	//接受应答位
	I2C_SendData(I2C1, Data);	//写数据
	BMI088_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED);	//不接受应答位
	I2C_GenerateSTOP(I2C1, ENABLE);	//通讯结束


}

uint8_t BMI088_ReadReg(uint8_t DevAddress, uint8_t RegAddress)
{
	I2C_GenerateSTART(I2C1, ENABLE);	//通讯开始
	BMI088_WaitEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT);	//等待事件EV5发生
	I2C_Send7bitAddress(I2C1, DevAddress, I2C_Direction_Transmitter);	//指定设备地址
	BMI088_WaitEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);	//等待事件EV6发生->设置为发送模式
	I2C_SendData(I2C1, RegAddress);	//指定寄存器地址
	BMI088_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED);	//等待事件EV8发生->发送中
	I2C_GenerateSTART(I2C1, ENABLE);	//开始时序
	BMI088_WaitEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT);//等待事件EV5发生
	I2C_Send7bitAddress(I2C1, DevAddress, I2C_Direction_Receiver);	//指定设备地址 改成读地址
	BMI088_WaitEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);	//等待事件EV6发生->设置为接受模式

	I2C_AcknowledgeConfig(I2C1, DISABLE);	//提前给Ack置0 STOP置1
	I2C_GenerateSTOP(I2C1, ENABLE);
	BMI088_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED);	//等待事件EV7发生->接受一个字节
	uint8_t Data = I2C_ReceiveData(I2C1);	//缓存数据
	I2C_AcknowledgeConfig(I2C1, ENABLE);	//ACK置1
	return Data;
}


int16_t GetData_Gyro(uint8_t REG_Address)
{
	uint8_t Hd, Ld;
	Ld = BMI088_ReadReg(0xD0, REG_Address);
	Hd = BMI088_ReadReg(0xD0, REG_Address + 1);
	return (Hd << 8) + Ld;
}

int16_t GetData_Acc(uint8_t REG_Address)
{

	uint8_t Hd, Ld;
	Ld = BMI088_ReadReg(0x30, REG_Address);
	Hd = BMI088_ReadReg(0x30, REG_Address + 1);
	return (Hd << 8) + Ld;

}




void ImuSensor_ReadReg_BuffAll(void)
{
	IIC_Timeout_Cnt = 0;
	int16_t ImuSensor_buff[6];

	ImuSensor_buff[0] = GetData_Gyro(BMI088_GYRO_XOUT);
	ImuSensor_buff[1] = GetData_Gyro(BMI088_GYRO_YOUT);
	ImuSensor_buff[2] = GetData_Gyro(BMI088_GYRO_ZOUT);
	ImuSensor_buff[3] = GetData_Acc(BMI088_ACCEL_XOUT);
	ImuSensor_buff[4] = GetData_Acc(BMI088_ACCEL_YOUT);
	ImuSensor_buff[5] = GetData_Acc(BMI088_ACCEL_ZOUT);

	if (IIC_Timeout_Cnt == 0)
	{
		MPU_Data.GyroX = ImuSensor_buff[0]; 	MPU_Data.GyroY = ImuSensor_buff[1];		MPU_Data.GyroZ = ImuSensor_buff[2];
		MPU_Data.AccX = ImuSensor_buff[3];		MPU_Data.AccY = ImuSensor_buff[4];		MPU_Data.AccZ = ImuSensor_buff[5];
	}

}

#define BMI088_GYRO_ODR BMI088_GYRO_ODR_200hz  //频率 
#define BMI088_GYRO_RNG BMI088_GYRO_RNG_2000	//量程
#define BMI088_ACC_ODR BMI088_ACC_ODR_200hz   //频率
#define BMI088_ACC_RNG BMI088_ACC_RNG_6G     //量程
#define BMI088_ACC_OSR BMI088_ACC_OSR_NULL  //滤波


void ImuSensor_Init(void)
{
	I2C_DeInit(I2C1);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOB时钟

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化


	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1);

	I2C_InitTypeDef I2C_InitStructure;
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;	//I2C模式
	I2C_InitStructure.I2C_ClockSpeed = 400000;	//I2C时钟速度 
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;	//I2C时钟占空比2:1 ，I2C时钟速度>100kHz时有效
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;	//应答位使能
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;	//寄存器地址宽度
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;	//从机地址 这里作为主机
	I2C_Init(I2C1, &I2C_InitStructure);
	I2C_AcknowledgeConfig(I2C1, ENABLE);
	I2C_Cmd(I2C1, ENABLE);

	while (BMI088_ReadReg(0xD0, 0x00) != 0x0f) {};
	while (BMI088_ReadReg(0x30, 0x00) != 0x1E) {};

	//角速度计初始化
	BMI088_WriteReg(0xD0, 0x10, BMI088_GYRO_ODR);
	//陀螺仪量程1000deg/s
	BMI088_WriteReg(0xD0, 0x0F, BMI088_GYRO_RNG);

	//加速度计初始化
	BMI088_WriteReg(0x30, 0x7D, 0x04);   //acc_enable
	BMI088_WriteReg(0x30, 0x7C, 0x00); //pwr_save_mode
									   //在200Hz的ODR下，加速度计每秒输出200个数据样本，加速度计的3dB截止频率为80Hz（正常模式），如果使用OSR2模式，则为38Hz；使用OSR4模式，则为20Hz。
									   //对于频率低于80Hz（正常模式）的信号，可以较好地通过滤波器，而高于该频率的信号会被衰减。
	BMI088_WriteReg(0x30, 0x40, (BMI088_ACC_OSR_NORMAL << 4) | (BMI088_ACC_ODR)); //acc_bwp  acc_odr
																				  //BMI088的加速度计满量程输出为-32768-+32768,若设置最大量程为3G则 1g 对应32768/6=5461.3333
	BMI088_WriteReg(0x30, 0x41, BMI088_ACC_RNG); //acc_range

	BMI088_WriteReg(0x30, 0x7D, 0x04);   //acc_enable
	BMI088_WriteReg(0x30, 0x7C, 0x00); //pwr_save_mode
	BMI088_WriteReg(0x30, 0x40, (BMI088_ACC_OSR_NORMAL << 4) | (BMI088_ACC_ODR)); //acc_bwp  acc_odr																		
	BMI088_WriteReg(0x30, 0x41, BMI088_ACC_RNG); //acc_range


	for (u8 i = 0; i < 3; i++)// 初始化加速计和陀螺二阶低通滤波
	{
		lpf2pInit(&gyroLpf[i], 200, GYRO_LPF_CUTOFF_FREQ);
		lpf2pInit(&accLpf[i], 200, ACCEL_LPF_CUTOFF_FREQ);
		//lpf2pInit(&gyroFeedBackLPF[i], 200, 30);

	}

	Set_Cutoff_Frequency(Sampling_Freq, 10, &Ins_Accel_Parameter);		//惯性导航加速度
	Set_Cutoff_Frequency(Sampling_Freq, 1, &Butter_1HZ_Parameter_Acce);//传感器校准加计滤波值
	Set_Cutoff_Frequency(Sampling_Freq, 10, &Accel_Parameter);			//姿态解算加计修正滤波值  
	Set_Cutoff_Frequency(Sampling_Freq, 10, &Gyro_Parameter_QU);			//四元数使用的角速度
	Set_Cutoff_Frequency(Sampling_Freq, 8, &Gyro_Parameter);			//角速度反馈滤波参数

}



void  GET_MPU_DATA(void)
{

	ImuSensor_ReadReg_BuffAll();

	int16_t ay = MPU_Data.AccX;
	int16_t ax = MPU_Data.AccY;
	int16_t az = MPU_Data.AccZ;

	int16_t gy = MPU_Data.GyroX;
	int16_t gx = MPU_Data.GyroY;
	int16_t gz = MPU_Data.GyroZ;


	sensors.gyro.x = -(gx - X_w_off) * SENSORS_DEG_PER_LSB_CFG;	/*单位 °/s */
	sensors.gyro.y = (gy - Y_w_off) * SENSORS_DEG_PER_LSB_CFG;
	sensors.gyro.z = (gz - Z_w_off) * SENSORS_DEG_PER_LSB_CFG;

	sensors.acc.x = -(ax)* SENSORS_G_PER_LSB_CFG;	/*单位 g(9.8m/s^2)*/
	sensors.acc.y = (ay)* SENSORS_G_PER_LSB_CFG;	/*重力加速度缩放因子accScale 根据样本计算得出*/
	sensors.acc.z = (az)* SENSORS_G_PER_LSB_CFG;


	applyAxis3fLpf(gyroLpf, &sensors.gyro);
	applyAxis3fLpf(accLpf, &sensors.acc);

	sensors1.gyro.x = sensors.gyro.x;
	sensors1.gyro.y = sensors.gyro.y;
	sensors1.gyro.z = sensors.gyro.z;

	sensors1.acc.x = sensors.acc.x;
	sensors1.acc.y = sensors.acc.y;
	sensors1.acc.z = sensors.acc.z;


	MPitch_Gyro = sensors.gyro.x;
	MRoll_Gyro = sensors.gyro.y;
	MYaw_Gyro = sensors.gyro.z;

	Accel.y = MPU_Data.AccX;			Accel.x = -MPU_Data.AccY;				Accel.z = MPU_Data.AccZ;
	Gyro.y = MPU_Data.GyroX - X_w_off;	Gyro.x = -(MPU_Data.GyroY - Y_w_off);	Gyro.z = MPU_Data.GyroZ - Z_w_off;


	/*Acce_Correct_Filter*/											//ACC矫正用的数据的滤波处理  
	Acce_Correct[0] = (int16_t)(LPButterworth(Accel.x, &Butter_Buffer_Correct[0], &Butter_1HZ_Parameter_Acce));
	Acce_Correct[1] = (int16_t)(LPButterworth(Accel.y, &Butter_Buffer_Correct[1], &Butter_1HZ_Parameter_Acce));
	Acce_Correct[2] = (int16_t)(LPButterworth(Accel.z, &Butter_Buffer_Correct[2], &Butter_1HZ_Parameter_Acce));

	///*椭球矫正*/
	X_Origion = IMU_K[0] * Accel.x - IMU_B[0] * One_G_TO_Accel;		//经过椭球校正后的三轴加速度量
	Y_Origion = IMU_K[1] * Accel.y - IMU_B[1] * One_G_TO_Accel;
	Z_Origion = IMU_K[2] * Accel.z - IMU_B[2] * One_G_TO_Accel;


	/*椭球矫正---END*/
	accel_filter.x = LPButterworth(X_Origion, &accel_filter_buf[0], &Accel_Parameter);
	accel_filter.y = LPButterworth(Y_Origion, &accel_filter_buf[1], &Accel_Parameter);
	accel_filter.z = LPButterworth(Z_Origion, &accel_filter_buf[2], &Accel_Parameter);

	//用作惯导融合的加速度计量
	Body_Frame.x = LPButterworth(X_Origion, &Butter_Buffer_SINS[0], &Ins_Accel_Parameter);
	Body_Frame.y = LPButterworth(Y_Origion, &Butter_Buffer_SINS[1], &Ins_Accel_Parameter);
	Body_Frame.z = LPButterworth(Z_Origion, &Butter_Buffer_SINS[2], &Ins_Accel_Parameter);

	MBody_Frame.x = sensors.acc.x / SENSORS_G_PER_LSB_CFG;
	MBody_Frame.y = sensors.acc.y / SENSORS_G_PER_LSB_CFG;
	MBody_Frame.z = sensors.acc.z / SENSORS_G_PER_LSB_CFG;

	/*********************角速度采集\滤波\处理***************************/


	gyro_nofilter.x = Gyro.x*GYRO_CALIBRATION_COFF;
	gyro_nofilter.y = Gyro.y*GYRO_CALIBRATION_COFF;
	gyro_nofilter.z = Gyro.z*GYRO_CALIBRATION_COFF;

}
#endif 
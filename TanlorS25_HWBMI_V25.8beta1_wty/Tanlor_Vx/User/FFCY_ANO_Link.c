#include"Headfile.h"
#include"FFCY_ANO_Link.h"


u8 data_to_send[64];
uint8_t ANO_Send_PID_Flag[6] = { 0 };
uint16_t usb_test[3] = { 0 };


uint32_t ANO_TxWrite(uint8_t *buffter, uint32_t writeLen)
{
	DMA_USART1_Send_Buff(data_to_send, writeLen);
	//DMA_Send(data_to_send, writeLen);
}

/***********************************************************************/
static void ANO_DT_Send_Check(u8 head, u8 check_sum)
{
	u8 sum = 0, i = 0;
	data_to_send[0] = 0xAA;
	data_to_send[1] = 0xAA;
	data_to_send[2] = 0xEF;
	data_to_send[3] = 2;
	data_to_send[4] = head;
	data_to_send[5] = check_sum;
	for (i = 0; i<6; i++)
		sum += data_to_send[i];
	data_to_send[6] = sum;
	ANO_TxWrite(data_to_send, 7);
}
u8 RxBuffer[50];
u8 _data_len = 0, _data_cnt = 0;
u8 state = 0;
u8 rem_state = 0;
u8 rec_cnt = 0;
void ANO_DT_Data_Receive_Prepare(u8 data)//ANO地面站数据解析
{
	rec_cnt++;
	if (data == 0xAA)
	{
		rem_state = 1;
	}
	else if (rem_state == 1)
	{
		if (data == 0xAF)
		{
			rem_state = 2;
		}
		else
			rem_state = 0;
	}

	if (state == 0 && data == 0xAA)//帧头1
	{
		state = 1;
		RxBuffer[0] = data;
	}
	else if (state == 1 && data == 0xAF)//帧头2
	{
		state = 2;
		RxBuffer[1] = data;
	}
	else if (state == 2 && data < 0XF1)//功能字节
	{
		state = 3;
		RxBuffer[2] = data;
	}
	else if (state == 3 && data < 50)//有效数据长度
	{
		state = 4;
		RxBuffer[3] = data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if (state == 4 && _data_len > 0)//数据接收
	{
		_data_len--;
		RxBuffer[4 + _data_cnt++] = data;
		if (_data_len == 0)
			state = 5;
	}
	else if (state == 5)//校验和
	{
		state = 0;
		RxBuffer[4 + _data_cnt] = data;
		ANO_DT_Data_Receive_Anl(RxBuffer, _data_cnt + 5);//数据解析
		rec_cnt = 0;
	}
	else state = 0;
}

void ANO_DT_Data_Receive_Anl(u8 *data_buf, u8 num)
{
	u8 sum = 0, i = 0;
	for (i = 0; i<(num - 1); i++)
		sum += *(data_buf + i);
	if (!(sum == *(data_buf + num - 1))) { usb_test[0]++; return; } //判断sum
	if (!(*(data_buf) == 0xAA && *(data_buf + 1) == 0xAF)) { usb_test[1]++; return; }//判断帧头
	usb_test[2]++;
	if (*(data_buf + 2) == 0X01)
	{
		if (*(data_buf + 4) == 0X01)
			;//mpu6050.Acc_CALIBRATE = 1;
		if (*(data_buf + 4) == 0X02)
			;//mpu6050.Gyro_CALIBRATE = 1;
		if (*(data_buf + 4) == 0X03)
		{
			;//mpu6050.Acc_CALIBRATE = 1;
			;//mpu6050.Gyro_CALIBRATE = 1;
		}
	}

	if (*(data_buf + 2) == 0X02)
	{
		if (*(data_buf + 4) == 0X01)
		{
			ANO_Send_PID_Flag[0] = 1;
			ANO_Send_PID_Flag[1] = 1;
			ANO_Send_PID_Flag[2] = 1;
			ANO_Send_PID_Flag[3] = 1;
			ANO_Send_PID_Flag[4] = 1;
			ANO_Send_PID_Flag[5] = 1;
		}
		if (*(data_buf + 4) == 0X02)
		{
		}
		if (*(data_buf + 4) == 0XA0)     //读取版本信息
		{
		}
		if (*(data_buf + 4) == 0XA1)     //恢复默认参数
		{
			Sort_PID_Flag = 2;
		}
	}

	if (*(data_buf + 2) == 0X10)                             //PID1
	{
		Total_Controller.Roll_Gyro_Control.Kp = 0.001*((vs16)(*(data_buf + 4) << 8) | *(data_buf + 5));
		Total_Controller.Roll_Gyro_Control.Ki = 0.001*((vs16)(*(data_buf + 6) << 8) | *(data_buf + 7));
		Total_Controller.Roll_Gyro_Control.Kd = 0.01*((vs16)(*(data_buf + 8) << 8) | *(data_buf + 9));
		Total_Controller.Pitch_Gyro_Control.Kp = 0.001*((vs16)(*(data_buf + 10) << 8) | *(data_buf + 11));
		Total_Controller.Pitch_Gyro_Control.Ki = 0.001*((vs16)(*(data_buf + 12) << 8) | *(data_buf + 13));
		Total_Controller.Pitch_Gyro_Control.Kd = 0.01*((vs16)(*(data_buf + 14) << 8) | *(data_buf + 15));
		Total_Controller.Yaw_Gyro_Control.Kp = 0.001*((vs16)(*(data_buf + 16) << 8) | *(data_buf + 17));
		Total_Controller.Yaw_Gyro_Control.Ki = 0.001*((vs16)(*(data_buf + 18) << 8) | *(data_buf + 19));
		Total_Controller.Yaw_Gyro_Control.Kd = 0.01*((vs16)(*(data_buf + 20) << 8) | *(data_buf + 21));
		ANO_DT_Send_Check(*(data_buf + 2), sum);

	}
	if (*(data_buf + 2) == 0X11)                             //PID2
	{
		Total_Controller.Roll_Angle_Control.Kp = 0.001*((vs16)(*(data_buf + 4) << 8) | *(data_buf + 5));
		Total_Controller.Roll_Angle_Control.Ki = 0.001*((vs16)(*(data_buf + 6) << 8) | *(data_buf + 7));
		Total_Controller.Roll_Angle_Control.Kd = 0.01*((vs16)(*(data_buf + 8) << 8) | *(data_buf + 9));
		Total_Controller.Pitch_Angle_Control.Kp = 0.001*((vs16)(*(data_buf + 10) << 8) | *(data_buf + 11));
		Total_Controller.Pitch_Angle_Control.Ki = 0.001*((vs16)(*(data_buf + 12) << 8) | *(data_buf + 13));
		Total_Controller.Pitch_Angle_Control.Kd = 0.01*((vs16)(*(data_buf + 14) << 8) | *(data_buf + 15));
		Total_Controller.Yaw_Angle_Control.Kp = 0.001*((vs16)(*(data_buf + 16) << 8) | *(data_buf + 17));
		Total_Controller.Yaw_Angle_Control.Ki = 0.001*((vs16)(*(data_buf + 18) << 8) | *(data_buf + 19));
		Total_Controller.Yaw_Angle_Control.Kd = 0.01*((vs16)(*(data_buf + 20) << 8) | *(data_buf + 21));
		ANO_DT_Send_Check(*(data_buf + 2), sum);

	}
	if (*(data_buf + 2) == 0X12)                             //PID3
	{
		Total_Controller.High_Speed_Control.Kp = 0.001*((vs16)(*(data_buf + 4) << 8) | *(data_buf + 5));
		Total_Controller.High_Speed_Control.Ki = 0.001*((vs16)(*(data_buf + 6) << 8) | *(data_buf + 7));
		Total_Controller.High_Speed_Control.Kd = 0.01*((vs16)(*(data_buf + 8) << 8) | *(data_buf + 9));
		Total_Controller.High_Position_Control.Kp = 0.001*((vs16)(*(data_buf + 10) << 8) | *(data_buf + 11));
		Total_Controller.High_Position_Control.Ki = 0.001*((vs16)(*(data_buf + 12) << 8) | *(data_buf + 13));
		Total_Controller.High_Position_Control.Kd = 0.01*((vs16)(*(data_buf + 14) << 8) | *(data_buf + 15));
		Total_Controller.Latitude_Speed_Control.Kp = 0.001*((vs16)(*(data_buf + 16) << 8) | *(data_buf + 17));
		Total_Controller.Latitude_Speed_Control.Ki = 0.001*((vs16)(*(data_buf + 18) << 8) | *(data_buf + 19));
		Total_Controller.Latitude_Speed_Control.Kd = 0.01*((vs16)(*(data_buf + 20) << 8) | *(data_buf + 21));
		/***********************位置控制：位置、速度参数共用一组PID参数**********************************************************/
		Total_Controller.Longitude_Speed_Control.Kp = Total_Controller.Latitude_Speed_Control.Kp;
		Total_Controller.Longitude_Speed_Control.Ki = Total_Controller.Latitude_Speed_Control.Ki;
		Total_Controller.Longitude_Speed_Control.Kd = Total_Controller.Latitude_Speed_Control.Kd;
		ANO_DT_Send_Check(*(data_buf + 2), sum);

	}
	if (*(data_buf + 2) == 0X13)                             //PID4
	{
		Total_Controller.Latitude_Position_Control.Kp = 0.001*((vs16)(*(data_buf + 4) << 8) | *(data_buf + 5));
		Total_Controller.Latitude_Position_Control.Ki = 0.001*((vs16)(*(data_buf + 6) << 8) | *(data_buf + 7));
		Total_Controller.Latitude_Position_Control.Kd = 0.01*((vs16)(*(data_buf + 8) << 8) | *(data_buf + 9));
		Total_Controller.High_Acce_Control.Kp = 0.001*((vs16)(*(data_buf + 10) << 8) | *(data_buf + 11));
		Total_Controller.High_Acce_Control.Ki = 0.001*((vs16)(*(data_buf + 12) << 8) | *(data_buf + 13));
		Total_Controller.High_Acce_Control.Kd = 0.01*((vs16)(*(data_buf + 14) << 8) | *(data_buf + 15));
		/***********************位置控制：位置、速度参数共用一组PID参数**********************************************************/
		Total_Controller.Longitude_Position_Control.Kp = Total_Controller.Latitude_Position_Control.Kp;
		Total_Controller.Longitude_Position_Control.Ki = Total_Controller.Latitude_Position_Control.Ki;
		Total_Controller.Longitude_Position_Control.Kd = Total_Controller.Latitude_Position_Control.Kd;
		ANO_DT_Send_Check(*(data_buf + 2), sum);

	}
	if (*(data_buf + 2) == 0X14)                             //PID5
	{
		Total_Controller.Optical_Position_Control.Kp = 0.001*((vs16)(*(data_buf + 4) << 8) | *(data_buf + 5));
		Total_Controller.Optical_Position_Control.Ki = 0.001*((vs16)(*(data_buf + 6) << 8) | *(data_buf + 7));
		Total_Controller.Optical_Position_Control.Kd = 0.01*((vs16)(*(data_buf + 8) << 8) | *(data_buf + 9));
		Total_Controller.Optical_Speed_Control.Kp = 0.001*((vs16)(*(data_buf + 10) << 8) | *(data_buf + 11));
		Total_Controller.Optical_Speed_Control.Ki = 0.001*((vs16)(*(data_buf + 12) << 8) | *(data_buf + 13));
		Total_Controller.Optical_Speed_Control.Kd = 0.01*((vs16)(*(data_buf + 14) << 8) | *(data_buf + 15));

		Total_Controller.SDK_Roll_Position_Control.Kp = 0.001*((vs16)(*(data_buf + 16) << 8) | *(data_buf + 17));
		Total_Controller.SDK_Roll_Position_Control.Ki = 0.001*((vs16)(*(data_buf + 18) << 8) | *(data_buf + 19));
		Total_Controller.SDK_Roll_Position_Control.Kd = 0.01*((vs16)(*(data_buf + 20) << 8) | *(data_buf + 21));

		Total_Controller.SDK_Pitch_Position_Control.Kp = 0.001*((vs16)(*(data_buf + 16) << 8) | *(data_buf + 17));
		Total_Controller.SDK_Pitch_Position_Control.Ki = 0.001*((vs16)(*(data_buf + 18) << 8) | *(data_buf + 19));
		Total_Controller.SDK_Pitch_Position_Control.Kd = 0.01*((vs16)(*(data_buf + 20) << 8) | *(data_buf + 21));

		ANO_DT_Send_Check(*(data_buf + 2), sum);
	}
	if (*(data_buf + 2) == 0X15)                             //PID6
	{
		ANO_DT_Send_Check(*(data_buf + 2), sum);
		Sort_PID_Cnt++;
		Sort_PID_Flag = 1;
		//Bling_Set(&Light_1,1000,50,0.5,0,GPIOB,GPIO_Pin_12,0);
		//Bling_Set(&Light_2,1000,50,0.5,0,GPIOB,GPIO_Pin_13,0);
		//Bling_Set(&Light_3,1000,50,0.5,0,GPIOB,GPIO_Pin_14,0);
	}
}


//1：发送基本信息（姿态、锁定状态）
void ANO_Data_Send_Status(void)
{
	u8 _cnt = 0;
	vs16 _temp;
	vs32 _temp2;
	u8 sum = 0;
	u8 i;

	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0x01;
	data_to_send[_cnt++] = 0;

	_temp = (int)(Roll * 100);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = (int)(Pitch * 100);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = (int)(-Yaw * 100);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);

	_temp2 = (vs32)(100 * NamelessQuad.Location.z);//单位cm
	data_to_send[_cnt++] = BYTE3(_temp2);
	data_to_send[_cnt++] = BYTE2(_temp2);
	data_to_send[_cnt++] = BYTE1(_temp2);
	data_to_send[_cnt++] = BYTE0(_temp2);

	data_to_send[_cnt++] = 0x01;//飞行模式
	data_to_send[_cnt++] = Controler_State;//上锁0、解锁1

	data_to_send[3] = _cnt - 4;
	sum = 0;
	for (i = 0; i<_cnt; i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	ANO_TxWrite(data_to_send, _cnt);
	
}

void ANO_DT_Send_Senser(s16 a_x, s16 a_y, s16 a_z, s16 g_x, s16 g_y, s16 g_z, s16 m_x, s16 m_y, s16 m_z)
{
	u8 _cnt = 0;
	vs16 _temp;
	u8 sum = 0;
	u8 i = 0;
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0x02;
	data_to_send[_cnt++] = 0;

	_temp = a_x;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = a_y;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = a_z;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);

	_temp = g_x;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = g_y;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = g_z;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);

	_temp = m_x;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = m_y;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = m_z;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);

	data_to_send[3] = _cnt - 4;

	sum = 0;
	for (i = 0; i<_cnt; i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	ANO_TxWrite(data_to_send, _cnt);
}
void ANO_DT_Send_RCData(u16 thr, u16 yaw, u16 rol, u16 pit, u16 aux1, u16 aux2, u16 aux3, u16 aux4, u16 aux5, u16 aux6)
{
	u8 _cnt = 0;
	u8 i = 0;
	u8 sum = 0;
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0x03;
	data_to_send[_cnt++] = 0;
	data_to_send[_cnt++] = BYTE1(thr);
	data_to_send[_cnt++] = BYTE0(thr);
	data_to_send[_cnt++] = BYTE1(yaw);
	data_to_send[_cnt++] = BYTE0(yaw);
	data_to_send[_cnt++] = BYTE1(rol);
	data_to_send[_cnt++] = BYTE0(rol);
	data_to_send[_cnt++] = BYTE1(pit);
	data_to_send[_cnt++] = BYTE0(pit);
	data_to_send[_cnt++] = BYTE1(aux1);
	data_to_send[_cnt++] = BYTE0(aux1);
	data_to_send[_cnt++] = BYTE1(aux2);
	data_to_send[_cnt++] = BYTE0(aux2);
	data_to_send[_cnt++] = BYTE1(aux3);
	data_to_send[_cnt++] = BYTE0(aux3);
	data_to_send[_cnt++] = BYTE1(aux4);
	data_to_send[_cnt++] = BYTE0(aux4);
	data_to_send[_cnt++] = BYTE1(aux5);
	data_to_send[_cnt++] = BYTE0(aux5);
	data_to_send[_cnt++] = BYTE1(aux6);
	data_to_send[_cnt++] = BYTE0(aux6);

	data_to_send[3] = _cnt - 4;

	sum = 0;
	for (i = 0; i<_cnt; i++)
		sum += data_to_send[i];

	data_to_send[_cnt++] = sum;
	ANO_TxWrite(data_to_send, _cnt);
}

void ANO_DT_Send_GPSData(u8 Fixstate,u8 GPS_Num,u32 log,u32 lat,int16 gps_head)
{
	u8 sum = 0;
	u8 _cnt = 0;
	u8 i = 0;
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0x04;
	data_to_send[_cnt++] = 0;
	data_to_send[_cnt++] = Fixstate;
	data_to_send[_cnt++] = GPS_Num;

	data_to_send[_cnt++] = BYTE3(log);
	data_to_send[_cnt++] = BYTE2(log);
	data_to_send[_cnt++] = BYTE1(log);
	data_to_send[_cnt++] = BYTE0(log);

	data_to_send[_cnt++] = BYTE3(lat);
	data_to_send[_cnt++] = BYTE2(lat);
	data_to_send[_cnt++] = BYTE1(lat);
	data_to_send[_cnt++] = BYTE0(lat);

	data_to_send[_cnt++] = BYTE1(gps_head);
	data_to_send[_cnt++] = BYTE0(gps_head);

	data_to_send[3] = _cnt - 4;

	sum = 0;
	for (i = 0; i<_cnt; i++)
		sum += data_to_send[i];

	data_to_send[_cnt++] = sum;
	ANO_TxWrite(data_to_send, _cnt);
	//  UART3_Send(data_to_send, _cnt);
}


void ANO_DT_Send_PID(u8 group, float p1_p, float p1_i, float p1_d, float p2_p, float p2_i, float p2_d, float p3_p, float p3_i, float p3_d)
{
	u8 _cnt = 0;
	u8 sum = 0, i = 0;
	int16_t _temp;

	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0x10 + group - 1;
	data_to_send[_cnt++] = 0;


	_temp = (int16_t)(p1_p * 1000);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = (int16_t)(p1_i * 1000);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = (int16_t)(p1_d * 100);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = (int16_t)(p2_p * 1000);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = (int16_t)(p2_i * 1000);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = (int16_t)(p2_d * 100);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = (int16_t)(p3_p * 1000);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = (int16_t)(p3_i * 1000);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = (int16_t)(p3_d * 100);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);

	data_to_send[3] = _cnt - 4;


	for (i = 0; i<_cnt; i++)
		sum += data_to_send[i];

	data_to_send[_cnt++] = sum;
	ANO_TxWrite(data_to_send, _cnt);
	//UART3_Send(data_to_send, _cnt);
}

/***********************************************************
@函数名：ANO_DT_Send_RCData
@入口参数：
@出口参数：无
功能描述：匿名地面站发送遥控器各个通道数据，发送遥控器通道数据
@作者：孙艺东
@日期：2021年7月27日
*************************************************************/
void ANO_DT_Send_USERData(float d1, float d2, float d3, float d4, float d5, float d6, float d7, float d8)
{
	u8 _cnt = 0;
	u8 i = 0;
	u8 sum = 0;
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0xF1;
	data_to_send[_cnt++] = 0;

	data_to_send[_cnt++] = BYTE3(d1);
	data_to_send[_cnt++] = BYTE2(d1);
	data_to_send[_cnt++] = BYTE1(d1);
	data_to_send[_cnt++] = BYTE0(d1);

	data_to_send[_cnt++] = BYTE3(d2);
	data_to_send[_cnt++] = BYTE2(d2);
	data_to_send[_cnt++] = BYTE1(d2);
	data_to_send[_cnt++] = BYTE0(d2);

	data_to_send[_cnt++] = BYTE3(d3);
	data_to_send[_cnt++] = BYTE2(d3);
	data_to_send[_cnt++] = BYTE1(d3);
	data_to_send[_cnt++] = BYTE0(d3);

	data_to_send[_cnt++] = BYTE3(d4);
	data_to_send[_cnt++] = BYTE2(d4);
	data_to_send[_cnt++] = BYTE1(d4);
	data_to_send[_cnt++] = BYTE0(d4);

	data_to_send[_cnt++] = BYTE3(d5);
	data_to_send[_cnt++] = BYTE2(d5);
	data_to_send[_cnt++] = BYTE1(d5);
	data_to_send[_cnt++] = BYTE0(d5);

	data_to_send[_cnt++] = BYTE3(d6);
	data_to_send[_cnt++] = BYTE2(d6);
	data_to_send[_cnt++] = BYTE1(d6);
	data_to_send[_cnt++] = BYTE0(d6);

	data_to_send[_cnt++] = BYTE3(d7);
	data_to_send[_cnt++] = BYTE2(d7);
	data_to_send[_cnt++] = BYTE1(d7);
	data_to_send[_cnt++] = BYTE0(d7);

	data_to_send[_cnt++] = BYTE3(d8);
	data_to_send[_cnt++] = BYTE2(d8);
	data_to_send[_cnt++] = BYTE1(d8);
	data_to_send[_cnt++] = BYTE0(d8);

	data_to_send[3] = _cnt - 4;

	sum = 0;
	for (i = 0; i < _cnt; i++)
		sum += data_to_send[i];

	data_to_send[_cnt++] = sum;
	ANO_TxWrite(data_to_send, _cnt);
}


extern float New_Yaw_speed ;
extern float New_Yaw_Posion ;
extern float Altitude_Speed;
extern Vector2f SDK_Control_Position;
extern u8 T265_Wrong_Stage;
extern Vector3f gyro_filter;
extern float euler_roll_init, euler_pitch_init, euler_yaw_init;
extern float Ax, Ay, Az, Gx, Gy, Gz, Roll_uart, Yaw_uart, Pitch_uart;

void ANO_Data_Send_All(void)
{
	//姿态用


	////定高用
	//ANO_DT_Send_USERData(
	//	New_Yaw_Posion,
	//	Altitude_Speed,
	//	Total_Controller.High_Speed_Control.Expect,
	//	Total_Controller.High_Speed_Control.Err,
	//	Total_Controller.High_Speed_Control.Integrate,
	//	Total_Controller.High_Speed_Control.Dis_Error_History[0],
	//	Total_Controller.High_Speed_Control.Control_OutPut, 
	//	Altitude_Estimate);	

	////定点用
	//ANO_DT_Send_USERData(
	//	UWB_X_Filter,
	//	UWB_Y_Filter,
	//	PosSenser_SINS.Location.x,
	//	PosSenser_SINS.Location.y,
	//	Altitude_Position,
	//	0,
	//	0,
	//	0);
	
}

extern u8 Gezi_x;
extern u8 Gezi_y;
extern u16 MYSDK_State;
int16_t ANO_Cnt = 0;
extern Vector3f gyro_filter, accel_filter;
extern u8 Rec_Anm_Gx[10];
extern u8 Rec_Anm_Gy[10];
extern float radar_speed_x;
extern float radar_speed_y;

void ANO_SEND_StateMachine(void)
{
	ANO_Cnt++;
	if (!(ANO_Send_PID_Flag[0] == 1 || ANO_Send_PID_Flag[1] == 1 || ANO_Send_PID_Flag[2] == 1 ||
		ANO_Send_PID_Flag[3] == 1 || ANO_Send_PID_Flag[4] == 1 || ANO_Send_PID_Flag[5] == 1))
	{
		ANO_Data_Send_Status();


		if (ANO_Cnt == 1)
		{
			ANO_DT_Send_USERData(
				ps_data.velocity.y,
				radar_speed_x,
				ps_data.velocity.x,
				radar_speed_y,
				0,
				0,
				0,
				0);
		}
		else if (ANO_Cnt == 2)
		{
			//ANO_DT_Send_Senser((int16_t)accel_filter.x, (int16_t)accel_filter.y, (int16_t)accel_filter.z, (int16_t)gyro_filter.x, (int16_t)gyro_filter.y, (int16_t)gyro_filter.z, 0, 0, 0);
		}
		else if (ANO_Cnt == 3)
		{
			//ANO_DT_Send_RCData(PPM_Databuf[2], PPM_Databuf[3], PPM_Databuf[0], PPM_Databuf[1], PPM_Databuf[4], PPM_Databuf[5], PPM_Databuf[6], PPM_Databuf[7], 0, 0);
		}
		else if (ANO_Cnt == 4 && ANO_Send_PID_Flag[0] == 0 && ANO_Send_PID_Flag[1] == 0 && ANO_Send_PID_Flag[2] == 0 && ANO_Send_PID_Flag[3] == 0 && ANO_Send_PID_Flag[4] == 0 && ANO_Send_PID_Flag[5] == 0)
		{
			//ANO_DT_Send_GPSData(1, GPS_Sate_Num, Longitude_Origion, Latitude_Origion, 10);
			ANO_Cnt = 0;
		}
	}

	if (ANO_Send_PID_Flag[0] >= 1)
	{
		ANO_DT_Send_PID(1,Total_Controller.Roll_Gyro_Control.Kp,Total_Controller.Roll_Gyro_Control.Ki,Total_Controller.Roll_Gyro_Control.Kd,Total_Controller.Pitch_Gyro_Control.Kp,Total_Controller.Pitch_Gyro_Control.Ki,
									Total_Controller.Pitch_Gyro_Control.Kd,Total_Controller.Yaw_Gyro_Control.Kp,Total_Controller.Yaw_Gyro_Control.Ki,Total_Controller.Yaw_Gyro_Control.Kd);
		ANO_Send_PID_Flag[0] ++;
		if (ANO_Send_PID_Flag[0] >= 6)
			ANO_Send_PID_Flag[0] = 0;
	}
   if (ANO_Send_PID_Flag[1] >= 1)
	{
		ANO_DT_Send_PID(2,Total_Controller.Roll_Angle_Control.Kp,Total_Controller.Roll_Angle_Control.Ki,Total_Controller.Roll_Angle_Control.Kd,Total_Controller.Pitch_Angle_Control.Kp,Total_Controller.Pitch_Angle_Control.Ki,
									Total_Controller.Pitch_Angle_Control.Kd,Total_Controller.Yaw_Angle_Control.Kp,Total_Controller.Yaw_Angle_Control.Ki,Total_Controller.Yaw_Angle_Control.Kd);
		ANO_Send_PID_Flag[1] ++;
		if (ANO_Send_PID_Flag[1] >= 6)
			ANO_Send_PID_Flag[1] = 0;
	}
	 if ( ANO_Send_PID_Flag[2] >= 1)
	{
		ANO_DT_Send_PID(3,Total_Controller.High_Speed_Control.Kp,Total_Controller.High_Speed_Control.Ki,Total_Controller.High_Speed_Control.Kd,Total_Controller.High_Position_Control.Kp,Total_Controller.High_Position_Control.Ki,
									Total_Controller.High_Position_Control.Kd,Total_Controller.Latitude_Speed_Control.Kp,Total_Controller.Latitude_Speed_Control.Ki,Total_Controller.Latitude_Speed_Control.Kd);
		ANO_Send_PID_Flag[2] ++;
		if (ANO_Send_PID_Flag[2] >= 6)
			ANO_Send_PID_Flag[2] = 0;
	}
	 if (ANO_Send_PID_Flag[3] >= 1)
	{
		ANO_DT_Send_PID(4,Total_Controller.Latitude_Position_Control.Kp,Total_Controller.Latitude_Position_Control.Ki,Total_Controller.Latitude_Position_Control.Kd,Total_Controller.High_Acce_Control.Kp,Total_Controller.High_Acce_Control.Ki,
									Total_Controller.High_Acce_Control.Kd,0, 0, 0);
		ANO_Send_PID_Flag[3] ++;
		if (ANO_Send_PID_Flag[3] >= 6)
			ANO_Send_PID_Flag[3] = 0;
	}
	 if (ANO_Send_PID_Flag[4] >= 1)
	{
		ANO_DT_Send_PID(5,Total_Controller.Optical_Position_Control.Kp, Total_Controller.Optical_Position_Control.Ki, Total_Controller.Optical_Position_Control.Kd, Total_Controller.Optical_Speed_Control.Kp, Total_Controller.Optical_Speed_Control.Ki
									, Total_Controller.Optical_Speed_Control.Kd, Total_Controller.SDK_Roll_Position_Control.Kp, Total_Controller.SDK_Roll_Position_Control.Ki, Total_Controller.SDK_Roll_Position_Control.Kd);
		ANO_Send_PID_Flag[4] ++;
		if (ANO_Send_PID_Flag[4] >= 6)
			ANO_Send_PID_Flag[4] = 0;
	}
	 if (ANO_Send_PID_Flag[5] >= 1)
	{
		ANO_DT_Send_PID(6, 0, 0, 0,0, 0, 0,0, 0, 0);
		ANO_Send_PID_Flag[5] ++;
		if (ANO_Send_PID_Flag[5] >= 6)
			ANO_Send_PID_Flag[5] = 0;
		ANO_Cnt = 0;
	}
}
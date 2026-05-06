#ifndef __FFCY_USART_H
#define __FFCY_USART_H

extern RingBuff_t Ground_Station_Ringbuf;
void USART1_Init(unsigned long bound);
void USART1_Send(unsigned char *tx_buf, int len);


void wust_sendccd(unsigned char *ccdaddr, int16_t ccdsize);
void wust_sendware(unsigned char *wareaddr, int16_t waresize);
void Quad_DMA1_USART1_SEND(u32 SendBuff,u16 len);//DMA---USART1传输
void DMA_Send_StateMachine(void);
void USART2_Init(unsigned long bound);


void USART2_Send(unsigned char *tx_buf, int len);


void USART4_Init(unsigned long bound);
void USART4_Send(unsigned char tx_buf);
void UART4_printf(char* fmt, ...);
void UART4_Send(unsigned char *tx_buf, int len);

void SBUS_USART2_Init(void);

void ANO_Data_Send_Status(void);
void ANO_SEND_StateMachine(void);


extern uint8_t RecBag[3];
extern uint8 US_100_Cnt;


extern unsigned int GPS_Data_Cnt;
extern u16 GPS_ISR_CNT;
extern  u8 GPS_Buf[2][100];
extern uint8 Ublox_Data[95];
extern uint8_t ANO_Send_PID_Flag[6];
extern uint8_t ANO_Send_PID_Flag_USB[6];
extern int the_new_number;
extern uint16 GPS_Update_finished,GPS_Update_finished_Correct_Flag;
extern Testime GPS_Time_Delta;
extern RingBuff_t SBUS_Ringbuf;


typedef struct
{
	Vector3f translation;
	Vector3f velocity;
	u8 Is_normal;				//数据是否正常
}pose_data;

extern pose_data ps_data;
extern pose_data ld_data;
void UART2_Send(unsigned char tx_buf);
extern u8 ldca;

#endif



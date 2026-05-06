#include "Headfile.h"
#include "SBUS.h"

const float _sbusScale = 0.625749f; //Matlab拟合出的量程因子
const float _sbusBias =  875.23353f;//Matlab拟合出的零偏
uint16_t SBUS_Channel[16]={0};
uint16_t SBUS_Channel_User[16]={0};
/***************************************************
函数名: void Process(uint8_t *raw,uint16_t *result)
说明:	SBUS数据解析
入口:	原始数据、解析数据
出口:	无
备注:	无
注释者：无名小哥
****************************************************/
void Process(uint8_t *raw,uint16_t *result)
{
  uint8_t bitsToRead=3; // bitsToRead表示需要从下一个字节中读取多少bit。规律：bitsToRead 每次总是增加 3
  uint8_t bitsToShift;
  uint8_t startByte=21;
  uint8_t channelId=15;
  do
  {
    result[channelId]=raw[startByte];
    if(bitsToRead<=8)
    {
      result[channelId]<<=bitsToRead;
      bitsToShift=8-bitsToRead;
      result[channelId]+=(raw[startByte-1]>>bitsToShift);
    }
    else
    {
      result[channelId]<<=8;
      result[channelId]+=raw[startByte-1];
      startByte--;
      bitsToRead-=8;
      result[channelId]<<=bitsToRead;
      bitsToShift=8-bitsToRead;
      result[channelId]+=(raw[startByte-1]>>bitsToShift);
    }
    result[channelId]&=0x7FF;
    channelId--;
    startByte--;
    bitsToRead+=3;
  }while(startByte>0);
}


uint8_t Sbus_Receive_Flag=0;
/***************************************************
函数名: bool SBUS_Linear_Calibration(void)
说明:	SBUS数据线性化处理
入口:	无
出口:	无
备注:	无
注释者：无名小哥
****************************************************/
u8 SBUS_Linear_Calibration(void)
{
  for(uint16_t i=0;i<=25;i++)
  {
    if(SBUS_Ringbuf.Ring_Buff[i]==0x0f&&SBUS_Ringbuf.Ring_Buff[i+24]==0x00)
    {
      Process((uint8_t *)(&SBUS_Ringbuf.Ring_Buff[i+1]),SBUS_Channel);
      for(uint8_t i = 0; i < 16; i++)//linear calibration
      {     
        SBUS_Channel_User[i] = (uint16_t)(SBUS_Channel[i] * _sbusScale + _sbusBias);
        //SBUS_Channel_User[i] = (uint16_t)(1000*SBUS_Channel[i]/2048)+1000;
      }    
      memcpy(PPM_Databuf,SBUS_Channel_User,10*sizeof(uint16));//将SBUS解析完毕的数据拷贝到PPM数据区  
      Sbus_Receive_Flag=1;
      return TRUE;
    }
  }
  Sbus_Receive_Flag=0;
  return FALSE;
}




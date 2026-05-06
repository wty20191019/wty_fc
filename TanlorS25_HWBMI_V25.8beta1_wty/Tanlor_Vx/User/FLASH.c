#include "Headfile.h"
#include "stm32f4xx_flash.h"//flash操作接口文件（在库文件中），必须要包含
#include "FLASH.h"


union {
  float Bit32;
  unsigned char Bit8[4];
}flash;

/****************************************************************
*Function:	STM32F103系列内部Flash读写操作
*Author:    ValerianFan
*Date:		2014/04/09
*E-Mail:	fanwenjingnihao@163.com
*Other:		该程序不能直接编译运行，只包含了Flash读写操作
****************************************************************/
//#define  STARTADDR  0x08010000                   	 //STM32F103RB 其他型号基本适用，未测试
#define  STARTADDR  0x0803A000//0x080350A8   2K=2048=0x800
volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;      //Flash操作状态变量
/****************************************************************
*Name:		ReadFlashNBtye
*Function:	从内部Flash读取N字节数据
*Input:		ReadAddress：数据地址（偏移地址）ReadBuf：数据指针	ReadNum：读取字节数
*Output:	读取的字节数
*Author:    ValerianFan
*Date:		2014/04/09
*E-Mail:	fanwenjingnihao@163.com
*Other:
****************************************************************/
int ReadFlashNBtye(uint32_t ReadAddress, uint8_t *ReadBuf, int32_t ReadNum)
{
  int DataNum = 0;
  ReadAddress = (uint32_t)STARTADDR + ReadAddress;
  while(DataNum < ReadNum)
  {
    *(ReadBuf + DataNum) = *(__IO uint8_t*) ReadAddress++;
    DataNum++;
  }
  return DataNum;
}


uint8_t ReadFlashOneWord(uint32_t ReadAddress,uint32_t *ReadData)
{
  uint16_t i=0;
  uint8_t buf[4];
  uint32_t temp_data=0;
  temp_data=*(uint32_t *)(ReadAddress);
  //FLASH_LockBank1();
  for(i=0;i<4;i++)//单字节数据
  {
    *(buf+i)=*(__IO uint8_t*) ReadAddress++;
  }
  if((buf[0]==0xff&&buf[1]==0xff&&buf[2]==0xff&&buf[3]==0xff))
    return 0;
  else
  {
    *ReadData=temp_data;
    return 1;
  }
}
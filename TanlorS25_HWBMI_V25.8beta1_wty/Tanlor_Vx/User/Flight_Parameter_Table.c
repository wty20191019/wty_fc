#include "Headfile.h"
#include "Flight_Parameter_Table.h"

volatile FLASH_Status Parameter_Table_FLASHStatus = FLASH_COMPLETE;      //Flash操作状态变量

																		 //FLASH起始地址
#define STM32_FLASH_BASE 0x08000000 	//STM32 FLASH的起始地址





uint16_t STMFLASH_GetFlashSector(u32 addr)
{
	if (addr < ADDR_FLASH_SECTOR_1)return FLASH_Sector_0;
	else if (addr < ADDR_FLASH_SECTOR_2)return FLASH_Sector_1;
	else if (addr < ADDR_FLASH_SECTOR_3)return FLASH_Sector_2;
	else if (addr < ADDR_FLASH_SECTOR_4)return FLASH_Sector_3;
	else if (addr < ADDR_FLASH_SECTOR_5)return FLASH_Sector_4;
	else if (addr < ADDR_FLASH_SECTOR_6)return FLASH_Sector_5;
	else if (addr < ADDR_FLASH_SECTOR_7)return FLASH_Sector_6;
	else if (addr < ADDR_FLASH_SECTOR_8)return FLASH_Sector_7;
	else if (addr < ADDR_FLASH_SECTOR_9)return FLASH_Sector_8;
	else if (addr < ADDR_FLASH_SECTOR_10)return FLASH_Sector_9;
	else if (addr < ADDR_FLASH_SECTOR_11)return FLASH_Sector_10;
	return FLASH_Sector_11;
}

void ReadFlashParameterALL(FLIGHT_PARAMETER *WriteData)
{
 	uint16_t i = 0;
 	uint32_t ReadAddress = (uint32_t)PARAMETER_TABLE_STARTADDR;
 	for (i = 0; i < FLIGHT_PARAMETER_TABLE_NUM; i++)
 	{
 		WriteData->Parameter_Table[i] = *(float *)(ReadAddress + 4 * i);
 	}
 	FLASH_Lock();
}


uint8_t ReadFlashParameterOne(uint16_t Label,float *ReadData)
{
   uint16_t i=0;
   uint8_t buf[4];
   float temp_data=0;
   uint32_t ReadAddress = (uint32_t)PARAMETER_TABLE_STARTADDR+Label*4;
   temp_data=*(float *)(ReadAddress);
   FLASH_Lock();
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

uint8_t ReadFlashParameterTwo(uint16_t Label,float *ReadData1,float *ReadData2)
{
   uint16_t i=0;
   uint8_t buf[8];
   float temp_data1=0,temp_data2=0;
   uint32_t ReadAddress = (uint32_t)PARAMETER_TABLE_STARTADDR+Label*4;
   temp_data1=*(float *)(ReadAddress);
   ReadAddress+=4;
   temp_data2=*(float *)(ReadAddress);
   
   FLASH_Lock();
   for(i=0;i<8;i++)//单字节数据
   {
     *(buf+i)=*(__IO uint8_t*) ReadAddress++;
   }
   if((buf[0]==0xff&&buf[1]==0xff&&buf[2]==0xff&&buf[3]==0xff) &&(buf[4]==0xff&&buf[5]==0xff&&buf[6]==0xff&&buf[7]==0xff))
     return 0;
   else
   {
     *ReadData1=temp_data1;
     *ReadData2=temp_data2;
     return 1;
   }
}

uint8_t ReadFlashParameterThree(uint16_t Label,float *ReadData1,float *ReadData2,float *ReadData3)
{
   uint16_t i=0;
   uint8_t buf[12];
   float temp_data1=0,temp_data2=0,temp_data3=0;
   uint32_t ReadAddress = (uint32_t)PARAMETER_TABLE_STARTADDR+Label*4;
   temp_data1=*(float *)(ReadAddress);
   ReadAddress+=4;
   temp_data2=*(float *)(ReadAddress);
   ReadAddress+=4;
   temp_data3=*(float *)(ReadAddress);
   
   FLASH_Lock();
   for(i=0;i<12;i++)//单字节数据
   {
     *(buf+i)=*(__IO uint8_t*) ReadAddress++;
   }
   if((buf[0]==0xff&&buf[1]==0xff&&buf[2]==0xff&&buf[3]==0xff)
      &&(buf[4]==0xff&&buf[5]==0xff&&buf[6]==0xff&&buf[7]==0xff)
        &&(buf[8]==0xff&&buf[9]==0xff&&buf[10]==0xff&&buf[11]==0xff))
     return 0;
   else
   {
     *ReadData1=temp_data1;
     *ReadData2=temp_data2;
     *ReadData3=temp_data3;
     return 1;
   }
}


FLIGHT_PARAMETER Table_Parameter;
void WriteFlashParameter(uint16_t Label,
                         float WriteData,
                         FLIGHT_PARAMETER *Table)
{
   uint16_t i=0;
   ReadFlashParameterALL(Table);//先把片区内的所有数据都都出来
   Table->Parameter_Table[Label]=WriteData;//将需要更改的字段赋新值
   FLASH_Unlock();									//解锁 
   FLASH_DataCacheCmd(DISABLE);						//FLASH擦除期间,必须禁止数据缓存
   Parameter_Table_FLASHStatus = FLASH_EraseSector(STMFLASH_GetFlashSector(PARAMETER_TABLE_STARTADDR), VoltageRange_3);
   if(Parameter_Table_FLASHStatus == FLASH_COMPLETE)
   {
     for(i=0;i<FLIGHT_PARAMETER_TABLE_NUM;i++)
     {
       Parameter_Table_FLASHStatus = FLASH_ProgramWord(PARAMETER_TABLE_STARTADDR + 4*i,*(uint32_t *)(&Table->Parameter_Table[i]));
     }
   }
   Flash_Bug();
   FLASH_DataCacheCmd(ENABLE);	//FLASH擦除结束,开启数据缓存
   FLASH_Lock();//上锁
}


void WriteFlashParameter_Two(uint16_t Label,float WriteData1,float WriteData2, FLIGHT_PARAMETER *Table)
{
   uint16_t i=0;
   ReadFlashParameterALL(Table);//先把片区内的所有数据都都出来
   Table->Parameter_Table[Label]=WriteData1;//将需要更改的字段赋新值
   Table->Parameter_Table[Label+1]=WriteData2;//将需要更改的字段赋新值
   FLASH_Unlock();									//解锁 
   FLASH_DataCacheCmd(DISABLE);						//FLASH擦除期间,必须禁止数据缓存
   Parameter_Table_FLASHStatus = FLASH_EraseSector(STMFLASH_GetFlashSector(PARAMETER_TABLE_STARTADDR), VoltageRange_3);
 
   if(Parameter_Table_FLASHStatus == FLASH_COMPLETE)
   {
     for(i=0;i<FLIGHT_PARAMETER_TABLE_NUM;i++)
     {
       Parameter_Table_FLASHStatus = FLASH_ProgramWord(PARAMETER_TABLE_STARTADDR + 4*i,*(uint32_t *)(&Table->Parameter_Table[i]));
     }
   }
   Flash_Bug();
   FLASH_DataCacheCmd(ENABLE);	//FLASH擦除结束,开启数据缓存
   FLASH_Lock();//上锁
}

void WriteFlashParameter_Three(uint16_t Label,float WriteData1,float WriteData2,float WriteData3,FLIGHT_PARAMETER *Table)
{
   uint16_t i=0;
   ReadFlashParameterALL(Table);//先把片区内的所有数据都都出来
   Table->Parameter_Table[Label]=WriteData1;//将需要更改的字段赋新值
   Table->Parameter_Table[Label+1]=WriteData2;//将需要更改的字段赋新值
   Table->Parameter_Table[Label+2]=WriteData3;//将需要更改的字段赋新值
   FLASH_Unlock();									//解锁 
   FLASH_DataCacheCmd(DISABLE);						//FLASH擦除期间,必须禁止数据缓存
   Parameter_Table_FLASHStatus = FLASH_EraseSector(STMFLASH_GetFlashSector(PARAMETER_TABLE_STARTADDR), VoltageRange_3);
   if(Parameter_Table_FLASHStatus == FLASH_COMPLETE)
   {
     for(i=0;i<FLIGHT_PARAMETER_TABLE_NUM;i++)
     {
       Parameter_Table_FLASHStatus = FLASH_ProgramWord(PARAMETER_TABLE_STARTADDR + 4*i,*(uint32_t *)(&Table->Parameter_Table[i]));
     }
   }
   Flash_Bug();
   FLASH_DataCacheCmd(ENABLE);	//FLASH擦除结束,开启数据缓存
   FLASH_Lock();//上锁
}

void WriteFlashParameter_TOADDR_FLASH_SECTOR(FLIGHT_PARAMETER *Table)
{
	uint16_t i = 0;
	ReadFlashParameterALL(Table);//先把片区内的所有数据都都出来
	FLASH_Unlock();									//解锁 
	FLASH_DataCacheCmd(DISABLE);						//FLASH擦除期间,必须禁止数据缓存
	Parameter_Table_FLASHStatus = FLASH_EraseSector(STMFLASH_GetFlashSector(ADDR_FLASH_SECTOR_10), VoltageRange_3);
	if (Parameter_Table_FLASHStatus == FLASH_COMPLETE)
	{
		for (i = 0; i < FLIGHT_PARAMETER_TABLE_NUM; i++)
		{
			Parameter_Table_FLASHStatus = FLASH_ProgramWord(ADDR_FLASH_SECTOR_10 + 4 * i, *(uint32_t *)(&Table->Parameter_Table[i]));
		}
	}
	FLASH_DataCacheCmd(ENABLE);	//FLASH擦除结束,开启数据缓存
	FLASH_Lock();//上锁
}
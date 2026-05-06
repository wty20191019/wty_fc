#ifndef _FLIGHT_PARAMETER_TABLE_H_
#define _FLIGHT_PARAMETER_TABLE_H_


//stm32f103c8    FLASH到0x0800FFFF截止（64K）
//stm32f103cB    FLASH到0x0801FFFF截止（128K）
//FLASH 扇区的起始地址
#define ADDR_FLASH_SECTOR_0     ((u32)0x08000000) 	//扇区0起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_1     ((u32)0x08004000) 	//扇区1起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_2     ((u32)0x08008000) 	//扇区2起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_3     ((u32)0x0800C000) 	//扇区3起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_4     ((u32)0x08010000) 	//扇区4起始地址, 64 Kbytes  
#define ADDR_FLASH_SECTOR_5     ((u32)0x08020000) 	//扇区5起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_6     ((u32)0x08040000) 	//扇区6起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_7     ((u32)0x08060000) 	//扇区7起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_8     ((u32)0x08080000) 	//扇区8起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_9     ((u32)0x080A0000) 	//扇区9起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_10    ((u32)0x080C0000) 	//扇区10起始地址,128 Kbytes  
#define ADDR_FLASH_SECTOR_11    ((u32)0x080E0000) 	//扇区11起始地址,128 Kbytes  


#define PARAMETER_TABLE_STARTADDR   ADDR_FLASH_SECTOR_11 //0x0801FA00+190(0x(100*4))
#define FLIGHT_PARAMETER_TABLE_NUM  150

typedef struct
{
   float Parameter_Table[FLIGHT_PARAMETER_TABLE_NUM];
}FLIGHT_PARAMETER;


typedef enum
{
	DATA_TRUE_FLAG = 0,
	PITCH_OFFSET = 1,
	ROLL_OFFSET = 2,
	ACCEL_X_OFFSET = 3,
	ACCEL_Y_OFFSET = 4,
	ACCEL_Z_OFFSET = 5,
	ACCEL_X_SCALE = 6,
	ACCEL_Y_SCALE = 7,
	ACCEL_Z_SCALE = 8,
	MAG_X_OFFSET = 9,
	MAG_Y_OFFSET = 10,
	MAG_Z_OFFSET = 11,
	RC_CH1_MAX = 12,
	RC_CH1_MIN = 13,
	RC_CH2_MAX = 14,
	RC_CH2_MIN = 15,
	RC_CH3_MAX = 16,
	RC_CH3_MIN = 17,
	RC_CH4_MAX = 18,
	RC_CH4_MIN = 19,
	RC_CH5_MAX = 20,
	RC_CH5_MIN = 21,
	RC_CH6_MAX = 22,
	RC_CH6_MIN = 23,
	RC_CH7_MAX = 24,
	RC_CH7_MIN = 25,
	RC_CH8_MAX = 26,
	RC_CH8_MIN = 27,
	PID1_PARAMETER_KP = 28,
	PID1_PARAMETER_KI = 29,
	PID1_PARAMETER_KD = 30,
	PID2_PARAMETER_KP = 31,
	PID2_PARAMETER_KI = 32,
	PID2_PARAMETER_KD = 33,
	PID3_PARAMETER_KP = 34,
	PID3_PARAMETER_KI = 35,
	PID3_PARAMETER_KD = 36,
	PID4_PARAMETER_KP = 37,
	PID4_PARAMETER_KI = 38,
	PID4_PARAMETER_KD = 39,
	PID5_PARAMETER_KP = 40,
	PID5_PARAMETER_KI = 41,
	PID5_PARAMETER_KD = 42,
	PID6_PARAMETER_KP = 43,
	PID6_PARAMETER_KI = 44,
	PID6_PARAMETER_KD = 45,
	PID7_PARAMETER_KP = 46,
	PID7_PARAMETER_KI = 47,
	PID7_PARAMETER_KD = 48,
	PID8_PARAMETER_KP = 49,
	PID8_PARAMETER_KI = 50,
	PID8_PARAMETER_KD = 51,
	PID9_PARAMETER_KP = 52,
	PID9_PARAMETER_KI = 53,
	PID9_PARAMETER_KD = 54,
	PID10_PARAMETER_KP = 55,
	PID10_PARAMETER_KI = 56,
	PID10_PARAMETER_KD = 57,
	PID11_PARAMETER_KP = 58,
	PID11_PARAMETER_KI = 59,
	PID11_PARAMETER_KD = 60,
	PID12_PARAMETER_KP = 61,
	PID12_PARAMETER_KI = 62,
	PID12_PARAMETER_KD = 63,
	PID13_PARAMETER_KP = 64,
	PID13_PARAMETER_KI = 65,
	PID13_PARAMETER_KD = 66,
	PID14_PARAMETER_KP = 67,
	PID14_PARAMETER_KI = 68,
	PID14_PARAMETER_KD = 69,
	PID15_PARAMETER_KP = 70,
	PID15_PARAMETER_KI = 71,
	PID15_PARAMETER_KD = 72,
	PID16_PARAMETER_KP = 73,
	PID16_PARAMETER_KI = 74,
	PID16_PARAMETER_KD = 75,
	PID17_PARAMETER_KP = 76,
	PID17_PARAMETER_KI = 77,
	PID17_PARAMETER_KD = 78,
	PID18_PARAMETER_KP = 79,
	PID18_PARAMETER_KI = 80,
	PID18_PARAMETER_KD = 81,
	PID19_PARAMETER_KP = 82,
	PID19_PARAMETER_KI = 83,
	PID19_PARAMETER_KD = 84,
	ESC_CALIBRATION_FLAG = 85,
	HOR_CAL_ACCEL_X = 86,
	HOR_CAL_ACCEL_Y = 87,
	HOR_CAL_ACCEL_Z = 88,
	SDK_MODE_DEFAULT = 89,
	LEFT_1 = 90,
	AHEAD_1 = 91,
	LEFT_2 = 92,
	AHEAD_2 = 93,
	RIGHT_1 = 94,
	BACK_1 = 95,
	RIGHT_2 = 96,
	AHEAD_3 = 97,
	RIGHT_3 = 98,
	BACK_2 = 99,
	RIGHT_4 = 100,
	AHEAD_4 = 101,
	flyhight = 102,
	FLY_speed = 103,
	colour1 = 104,
	colour2 = 105,
	colour3 = 106,
	colour4 = 107,
	Danger_Place = 108,
	Read_Number = 109,
	Third_Number = 113,
	Del_Stage = 110,
	First_Number = 111,
	Second_Number = 112,
	tsk_hight = 114,
	FIRE_HEAD = 115,
	FIRE_BACK = 116,
	FIRE_LEFT_1 = 117,
	FIRE_LEFT_2 = 118,
	FIRE_HEAD_2 = 135,
	FIRE_HEAD_3 = 136,
	FIRE_BACK_2 = 137,
	FIRE_LEFT_3 = 138,
	FIRE_LEFT_4 = 139,
	OPENMV_CORRECT = 119,
}FLIGHT_PARAMETER_TABLE;




void ReadFlashParameterALL(FLIGHT_PARAMETER *WriteData);
uint8_t ReadFlashParameterOne(uint16_t Label,float *ReadData);
uint8_t ReadFlashParameterTwo(uint16_t Label,float *ReadData1,float *ReadData2);
uint8_t ReadFlashParameterThree(uint16_t Label,float *ReadData1,float *ReadData2,float *ReadData3);
   
   
void WriteFlashParameter(uint16_t Label,float WriteData,FLIGHT_PARAMETER *Table);
void WriteFlashParameter_Two(uint16_t Label,
                         float WriteData1,
                         float WriteData2,
                         FLIGHT_PARAMETER *Table);
void WriteFlashParameter_Three(uint16_t Label,
                         float WriteData1,
                         float WriteData2,
                         float WriteData3,
                         FLIGHT_PARAMETER *Table);
uint8_t ReadFlashParameterTwo(uint16_t Label,float *ReadData1,float *ReadData2);
void WriteFlashParameter_TOADDR_FLASH_SECTOR(FLIGHT_PARAMETER *Table);
extern FLIGHT_PARAMETER Table_Parameter;
extern volatile FLASH_Status Parameter_Table_FLASHStatus;      //Flash操作状态变量
uint16_t STMFLASH_GetFlashSector(u32 addr);

#endif


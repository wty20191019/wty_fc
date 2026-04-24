


#include "board.h"
#include "bsp_uart.h"
#include "i2c1.h"
#include "FFCY_NEW_ICM42688.h"
#include <stdio.h>
#include "oled.h"

int main(void)
{
	
	board_init();
	
	uart1_init(115200U);
	ImuSensor_Init();

	OLED_Init();     //≥ı ºªØOLED
	OLED_Clear();
	
	while(1) 
	{
		char line[22];

		ImuSensor_ReadReg_BuffAll();
		OLED_Clear();

		snprintf(line, sizeof(line), "ICM42688 RAW");
		OLED_ShowString(0, 0, (uint8_t *)line, 8, 1);

		snprintf(line, sizeof(line), "AX:%6d", MPU_Data.AccX);
		OLED_ShowString(0, 8, (uint8_t *)line, 8, 1);

		snprintf(line, sizeof(line), "AY:%6d", MPU_Data.AccY);
		OLED_ShowString(0, 16, (uint8_t *)line, 8, 1);

		snprintf(line, sizeof(line), "AZ:%6d", MPU_Data.AccZ);
		OLED_ShowString(0, 24, (uint8_t *)line, 8, 1);

		snprintf(line, sizeof(line), "GX:%6d", MPU_Data.GyroX);
		OLED_ShowString(0, 32, (uint8_t *)line, 8, 1);

		snprintf(line, sizeof(line), "GY:%6d", MPU_Data.GyroY);
		OLED_ShowString(0, 40, (uint8_t *)line, 8, 1);

		snprintf(line, sizeof(line), "GZ:%6d", MPU_Data.GyroZ);
		OLED_ShowString(0, 48, (uint8_t *)line, 8, 1);

		snprintf(line, sizeof(line), "T :%6d", MPU_Data.Temp);
		OLED_ShowString(0, 56, (uint8_t *)line, 8, 1);

		OLED_Refresh();
		delay_ms(100);
					
	}
	

}

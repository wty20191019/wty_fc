/*
 * 立创开发板软硬件资料与相关扩展板软硬件资料官网全部开源
 * 开发板官网：www.lckfb.com
 * 技术支持常驻论坛，任何技术问题欢迎随时交流学习
 * 立创论坛：https://oshwhub.com/forum
 * 关注bilibili账号：【立创开发板】，掌握我们的最新动态！
 * 不靠卖板赚钱，以培养中国工程师为己任
 * 

 Change Logs:
 * Date           Author       Notes
 * 2024-03-14     LCKFB-LP    first version
 */
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

	OLED_Init();     //初始化OLED
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

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
#include <stdio.h>
#include "oled.h"

int main(void)
{
	
	board_init();
	
	uart1_init(115200U);
	i2c1_init(100000U);

	OLED_Init();     //初始化OLED
	OLED_Clear();
	
	while(1) 
	{
		OLED_ShowString(0,0,(uint8_t *)"ABC",8,1);//6*8 “ABC”
		OLED_ShowString(0,8,(uint8_t *)"ABC",12,1);//6*12 “ABC”
		OLED_ShowString(0,20,(uint8_t *)"ABC",16,1);//8*16 “ABC”
		OLED_ShowString(0,36,(uint8_t *)"ABC",24,1);//12*24 “ABC”
		OLED_Refresh();
		delay_ms(100);
					
	}
	

}

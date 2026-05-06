#include "Headfile.h"
#include "SPI1.h"

char SPI_SendReceive(char data)     //SPI1µÄÊÕ·¢
  {		  
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);  //while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET); 
  SPI_I2S_SendData(SPI1, data);	
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
  return  SPI_I2S_ReceiveData(SPI1);
  } 

#ifndef __PMW3901_H
#define __PMW3901_H

#include "stm32f4xx.h"
#include "sys.h"

#define PMW3901_CS  PBout(12)

extern s16 DetX, DetY;


u8  PMW3901_Init(void);
u8  PMW3901_Getinit_OK(void);
void PMW3901_Read_deltaXY(s16 *deltaX, s16 *deltaY);
void flow_fusion();
void flow_altitude_compensation();
void PMW3901_Read_Images(void);

#endif
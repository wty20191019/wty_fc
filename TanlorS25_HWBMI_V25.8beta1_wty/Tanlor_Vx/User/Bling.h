#ifndef __BLING_H
#define __BLING_H


#define State_Led PCout(4)
#define  LED_R PCout(5)
#define  LED_B PCout(6)
#define  LED_G PCout(7)

#define LASER1 PCout(8)
#define BEEP PCout(9)
#define Anm_Led PBout(4)

void Bling_Init(void);
void Led_Statemachine(void);

extern  u8 LASER1_flag ; extern int LASER1_times ;
extern u8 BEEP_flag ; extern int BEEP_times ;
extern u8 Anm_Led_flag; extern int Anm_Led_times;

#endif

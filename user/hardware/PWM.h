#ifndef __PWM_H
#define __PWM_H

#include "stm32f4xx.h"
//#include "sys.h"
#include "main.h"

#define Fric_UP 1250
#define Fric_DOWN 1150


#define SET_Fric 1000
#define SET_Fric2 1325

#define SET_TIM8_Fric 1600

/*
Fric_OFF  1500 占空比0.075
Fric_OFF  1000 占空比0.05
1314
*/
extern void PWM_Init(void);
extern void set_fric2(void);
extern void set_fric(void);
extern void fric1_on(uint16_t cmd);
extern void fric2_on(uint16_t cmd);

extern void set_TIM8_fric1(void);
extern void set_TIM8_fric1_on(uint16_t cmd);

#endif


#ifndef __TIME_H
#define __TIME_H

#include "stm32f4xx.h"
//#include "sys.h"
#include "time.h"
#include "main.h"
extern unsigned long long sysTime;      //时间戳 单位ms

void Tick_TIM5_Init (u16 arr );
void TIM3_Int_Init(void);
void TIM4_Int_Init(void);
void TIM2_Int_Init(void);

#endif

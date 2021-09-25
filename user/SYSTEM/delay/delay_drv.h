#include "stm32f4xx.h"
#ifndef __DELAY_DRV_H__
#define __DELAY_DRV_H__

#ifdef __DELAY_DRV_C__
#define GLOBAL
#else
#define GLOBAL extern
#endif

GLOBAL uint32_t global_times;
GLOBAL volatile u32 delay_ms_const;

GLOBAL void Delay_us_Init(uint32_t c);
GLOBAL void delay_ms(u32 c);
GLOBAL void delay_us(u32 c);

u32 gt_get(void);
u32 gt_get_sub(u32 c);
	
#undef GLOBAL
#endif //__DELAY_DRV_H__


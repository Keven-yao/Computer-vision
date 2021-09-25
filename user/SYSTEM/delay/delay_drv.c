
#include "stm32f4xx.h"
//#include "timer.h"

#define __DELAY_DRV_C__
#include "delay_drv.h"

volatile static uint32_t Delay_us_Step;


void sleep(uint32_t c)
{
	volatile uint32_t t = c;
	while(t--);
}

void Delay_us_Init(uint32_t c)
{
	Delay_us_Step = c;
}

void delay_us(u32 nus)
{		
	nus *= Delay_us_Step;//72;
	sleep(nus);
}



void delay_ms(u32 c)
{
	delay_ms_const = c;
	while(delay_ms_const);
}

void delay_ms_set(u32 c)
{
	delay_ms_const = c;
}

u32 delay_ms_getlost(void)
{
	return delay_ms_const;
}

u8 delay_ms_endcheck(void)
{
	return delay_ms_const ? 0:1;
}

u32 gt_get(void)
{
	return global_times;
}

u32 gt_get_sub(u32 c)
{
	if(c > global_times)
		c -= global_times;
	else
		c = 0;
	return c;
}

//end file


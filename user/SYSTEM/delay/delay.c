#include "stm32f4xx.h"


uint8_t  src_nus = 0; // 微秒定时初值
uint16_t src_nms = 0; // 毫秒定时初值

/**********************************************************
函数名:delay_Init
功  能:延时初始化
参  数:无
返回值:无
**********************************************************/
void delay_Init()
{
	SysTick->CTRL = ~(1<<2); //AHB/8 分频时钟  即SystemcoreClock/8
	src_nus = SystemCoreClock/8000000; //得到微秒单位定时初值 
	src_nms = src_nus*1000;  //得到毫秒单位定时初值
}

/**********************************************************
函数名:delay_us
功  能:微秒单位延时
参  数:nus 延时时间  nus <= 1864135
返回值:无
**********************************************************/
void delay_us(uint32_t nus)
{
	uint32_t temp = 0;
	
	SysTick->LOAD = nus*src_nus;  //微秒单位定时预装初值
	SysTick->VAL = 0x000000;      //计数器清零
	SysTick->CTRL = 0x01;		  //定时器使能
	do
	{
		temp = SysTick->CTRL;	  //读定时器状态控制寄存器
	}
	while((temp&0x01) && !(temp&(1<<16))); //判断相关位
	SysTick->CTRL = 0x00;        //关闭定时器
}
/**********************************************************
函数名:delay_ms
功  能:毫秒单位延时
参  数:nms 延时时间   nms <= 1864
返回值:无
**********************************************************/
void delay_ms(uint16_t nms)
{
	uint32_t temp = 0;
	
	SysTick->LOAD = nms*src_nms;// 毫秒单位定时预装初值
	SysTick->VAL = 0x000000;	//计数器清零
	SysTick->CTRL = 0x01;		//使能定时器
	do
	{
		temp= SysTick->CTRL;	//读定时器状态控制寄存器
	}
	while((temp&0x01) && !(temp&(1<<16)));//判断相关位
	SysTick->CTRL = 0x00;
}

#include "stm32f4xx.h"
//#include "sys.h"
#include "start_task.h"
#include "feedback.h"
#include "start_task.h"
#include "control.h"
#include "init.h"
#include "imu.h"
#include "main.h"
unsigned long long sysTime = 0;      //时间戳 单位ms

void Tick_TIM5_Init (u16 arr )   //间隔为0.01ms * arr
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);  ///使能TIM7时钟
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=850 -1 ;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseInitStructure);//初始化TIM7
	
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE); //允许定时器7更新中断
	TIM_Cmd(TIM5,ENABLE); //使能定时器3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM5_IRQn; //定时器7中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x03; //抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

void TIM2_Int_Init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);//初始化定时器2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  	//TIM2时钟使能
	
	TIM_TimeBaseStructure.TIM_Prescaler=85-1;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=10000;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_Cmd(TIM2, ENABLE);  //使能TIM4
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
}

void TIM5_IRQHandler( void )
{
	if ( TIM_GetITStatus( TIM5, TIM_IT_Update ) == SET )
	{
		IMU_getYawPitchRoll( angle );
		//printf("angle0: %f\r\n",angle[0]);
	//	printf("angle0: %f  1:%f  2:%f\r\n",angle[0],angle[1],angle[2]);
		GetPitchYawGxGyGz();	
	}
	TIM_ClearITPendingBit( TIM5, TIM_IT_Update );
}



///**
//  ****************************(C) COPYRIGHT 2016 DJI****************************
//  * @file       main.c/h
//  * @brief      stm32初始化以及开始任务freeRTOS。h文件定义相关全局宏定义以及
//  *             typedef 一些常用数据类型
//  * @note       
//  * @history
//  *  Version    Date            Author          Modification
//  *  V1.0.0     Dec-26-2018     RM              1. 完成
//  *
//  @verbatim
//  ==============================================================================

//  ==============================================================================
//  @endverbatim
//  ****************************(C) COPYRIGHT 2016 DJI****************************
//  */
//#include "main.h"

//#include "stm32f4xx.h"

//#include "adc.h"
//#include "buzzer.h"
//#include "can.h"
//#include "delay.h"
//#include "flash.h"
//#include "fric.h"
//#include "laser.h"
//#include "led.h"
//#include "power_ctrl.h"
//#include "rc.h"
//#include "rng.h"
//#include "sys.h"
//#include "timer.h"

//#include "FreeRTOSConfig.h"
//#include "FreeRTOS.h"
//#include "task.h"

//#include "calibrate_task.h"
//#include "remote_control.h"
//#include "start_task.h"

//void BSP_init(void);

//int main(void)
//{
//    BSP_init();
//    delay_ms(100);
//    startTast();
//    vTaskStartScheduler();
//    while (1)
//    {
//        ;
//    }
//}

////四个24v 输出 依次开启 间隔 709us
//#define POWER_CTRL_ONE_BY_ONE_TIME 709

//void BSP_init(void)
//{
//    //中断组 4
//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
//    //初始化滴答时钟
//    delay_init(configTICK_RATE_HZ);
//    //流水灯，红绿灯初始化
//    led_configuration();
//    //stm32 板载温度传感器初始化
//    temperature_ADC_init();
//#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
//    //stm32 随机数发生器初始化
//    RNG_init();
//#endif
//    //24输出控制口 初始化
//    power_ctrl_configuration();
//    //摩擦轮电机PWM初始化
//    fric_PWM_configuration();
//    //蜂鸣器初始化
//    buzzer_init(30000, 90);
//    //激光IO初始化
//    laser_configuration();
//    //定时器6 初始化
//    TIM6_Init(60000, 90);
//    //CAN接口初始化
//    CAN1_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
//    CAN2_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);

//    //24v 输出 依次上电
//    for (uint8_t i = POWER1_CTRL_SWITCH; i < POWER4_CTRL_SWITCH + 1; i++)
//    {
//        power_ctrl_on(i);
//        delay_us(POWER_CTRL_ONE_BY_ONE_TIME);
//    }
//    //遥控器初始化
//    remote_control_init();
//    //flash读取函数，把校准值放回对应参数
//    cali_param_init();
//}
#include "main.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "gimbal.h"

#include "init.h"
#include "time.h"
#include "spi.h"
#include "imu.h"

#include "delay.h"


//板级初始化函数，初始化外设
void BSP_init(void);

int main(void)
{		
		//板级初始化函数，初始化外设
		BSP_init();
		//开始创建任务
		start_task();
		//开始任务调度
		vTaskStartScheduler();   
		//创建开始led_configuration任务
		set_fric();
		while (1)
    {
        ;
    }

}
void BSP_init()
{
	  //中断组4，抢断优先级0-15响应优先级0
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
		//初始化系统时钟
		delay_init(configTICK_RATE_HZ);
		//初始化流水灯
		led_configuration(); 
		//初始化串口1接收遥控器数据，并处理
    USART1_Configuration();
		//调试使用的串口6，不用关掉
    usart6_Init();
		//电源输出控制口 初始化
		powerInit();
    //CAN接口初始化
    CAN1_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
    CAN2_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
		//set_TIM8_fric1();
		
		set_fric();
	  SPI5_Init();
		SPI5_SetSpeed( SPI_BaudRatePrescaler_128 );
		MPU6500_Init();
	
		PWM_Init();     //pwm初始化
		//set_TIM8_fric1();
	
		Tick_TIM5_Init(100);
		TIM2_Int_Init();

}







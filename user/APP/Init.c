
#include "main.h" 
#include "time.h"
#include "spi.h"
#include "imu.h"
//#include "led.h"
void Init(void)
{
		delay_Init();   //延时初始化
	
		SPI5_Init();
		SPI5_SetSpeed( SPI_BaudRatePrescaler_128 );
		MPU6500_Init();
		
		powerInit();   //led初始化
	
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);  //16个抢占优先级   
	
    CAN1_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);//CAN1初始化
    CAN2_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);//CAN1初始化
    
	
		usart6_Init();  //遥控器
		PWM_Init();     //pwm初始化
	
		Tick_TIM5_Init(100);
		TIM2_Int_Init();

	

}

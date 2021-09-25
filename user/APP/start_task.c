
#include "main.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "gimbal.h"
//开始创建任务优先级
#define START_TASK_PRIO		1
//开始创建任务堆栈大小	
#define START_STK_SIZE 		512  
//开始创建任务句柄
TaskHandle_t StartTask_Handler;

//底盘任务优先级
#define Chassis_TASK_PRIO 15      
//底盘任务堆栈
#define Chassis_STK_SIZE 512
//底盘任务创建句柄
TaskHandle_t ChassisTask_Handler;

//云台任务优先级
#define GIMBAL_TASK_PRIO 19
//云台任务堆栈
#define GIMBAL_STK_SIZE 512
//云台任务句柄
TaskHandle_t GIMBALTask_Handler;

//发射任务优先级=
#define SHOOT_TASK_PRIO 12
//发射任务堆栈
#define SHOOT_STK_SIZE 512
//发射任务句柄
TaskHandle_t SHOOTTask_Handler;

void start_rob_task(void *pvParameters)
{
	//进入代码临界保护区，不得打断
	taskENTER_CRITICAL();
	//开启底盘任务
	xTaskCreate((TaskFunction_t)chassis_task, 
						(const char *)"ChassisTask",
						(uint16_t)Chassis_STK_SIZE,
						(void *)NULL,
						(UBaseType_t)Chassis_TASK_PRIO,
						(TaskHandle_t *)&ChassisTask_Handler);
						
	//开启云台任务
	xTaskCreate((TaskFunction_t)gimbal_Task,
						(const char *)"gimbal_Task",
						(uint16_t)GIMBAL_STK_SIZE,
						(void *)NULL,
						(UBaseType_t)GIMBAL_TASK_PRIO,
						(TaskHandle_t *)&GIMBALTask_Handler);
	//开启发射任务
	xTaskCreate((TaskFunction_t)shoot_task,
						(const char *)"shoot_task",
						(uint16_t)SHOOT_STK_SIZE,
						(void *)NULL,
						(UBaseType_t)SHOOT_TASK_PRIO,
						(TaskHandle_t *)&SHOOTTask_Handler);
	
	
	//删除开始任务
	vTaskDelete(StartTask_Handler);
	//退出临界区				
  taskEXIT_CRITICAL();            

}

void start_task()
{
	  
	 xTaskCreate((TaskFunction_t )start_rob_task,            //任务函数
                (const char*    )"start_rob_task",          //任务名称
                (uint16_t       )START_STK_SIZE,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )START_TASK_PRIO,       //任务优先级
                (TaskHandle_t*  )&StartTask_Handler);   //任务句柄              
     
}





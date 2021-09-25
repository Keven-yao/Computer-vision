#ifndef _GIMBAL_H_
#define _GIMBAL_H_
#include "stm32f4xx.h"
//#include "sys.h"
#include "main.h"
void gimbal_Task(void *pvParameters);

void gimbalTask(void);
void gimbalInit(void);
void gimbalSend(int16_t chassisMotor1, int16_t chassisMotor2, int16_t chassisMotor3);

extern volatile float angle_roll_pitch[3];
#define midYaw 4050    
#define midPitch 2700 

#define YAW_PID_KP 8.75555f
#define YAW_PID_KI 0.0f
#define YAW_PID_KD 7.50f
#define YAW_PID_MAX_OUT 20000.0f
#define YAW_PID_MAX_IOUT 3000.00f

//pitch 角度环 PID参数以及 PID最大输出，积分输出
#define PITCH_PID_KP 30.0f
#define PITCH_PID_KI 0.00f
#define PITCH_PID_KD 40.020f
#define PITCH_PID_MAX_OUT 30000.0f
#define PITCH_PID_MAX_IOUT 10000.0f

//roll 角度环 PID参数以及 PID最大输出，积分输出
#define ROLL_PID_KP 40.0f
#define ROLL_PID_KI 2.50f
#define ROLL_PID_KD 20.020f
#define ROLL_PID_MAX_OUT 20000.0f
#define ROLL_PID_MAX_IOUT 1000.0f

typedef struct
{
    float kp;
    float ki;
    float kd;

    float set;
    float get;
    float err;

    float max_out;
    float max_iout;

    float Pout;
    float Iout;
    float Dout;

    float out;
} Gimbal_PID_t;





#endif

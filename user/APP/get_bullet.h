#ifndef __GET_BULLET_H__
#define __GET_BULLET_H__

//yaw 角度环 PID参数以及 PID最大输出，积分输出
#define PID_KP 10.0f
#define PID_KI 0.05f
#define PID_KD 0.000f
#define PID_MAX_OUT 20000.0f
#define PID_MAX_IOUT 8000.0f

//pitch 角度环 PID参数以及 PID最大输出，积分输出
//#define PITCH_PID_KP 70.0f
//#define PITCH_PID_KI 0.50f
//#define PITCH_PID_KD 0.020f
//#define PITCH_PID_MAX_OUT 20000.0f
//#define PITCH_PID_MAX_IOUT 8000.0f

//M3508 速度环 PID参数以及 PID最大输出，积分输出
#define Speed_M3508_PID_KP 10.0f
#define Speed_M3508_PID_KI 0.02f
#define Speed_M3508_PID_KD 0.0f
#define Speed_M3508_PID_MAX_OUT 12000.0f
#define Speed_M3508_PID_MAX_IOUT 8000.0f

//M2006 速度环 PID参数以及 PID最大输出，积分输出
#define Speed_M2006_PID_KP 10.0f
#define Speed_M2006_PID_KI 0.01f
#define Speed_M2006_PID_KD 0.0f
#define Speed_M2006_PID_MAX_OUT 12000.0f
#define Speed_M2006_PID_MAX_IOUT 8000.0f


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
} PID_t;


void M3508Init(void);
void get_bullet_task(void);




#endif

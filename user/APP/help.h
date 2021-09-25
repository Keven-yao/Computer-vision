#ifndef __HELP_H__
#define __HELP_H__


//M2006 速度环 PID参数以及 PID最大输出，积分输出
#define Speed_M2006_2_PID_KP 10.0f
#define Speed_M2006_2_PID_KI 0.00f
#define Speed_M2006_2_PID_KD 0.0f
#define Speed_M2006_2_PID_MAX_OUT 12000.0f
#define Speed_M2006_2_PID_MAX_IOUT 8000.0f

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
} M2006_PID_t;

void M2006Init(void);
void help_task(void);


#endif


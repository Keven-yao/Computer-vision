

#include "main.h"
//
M2006_PID_t M2006_2_pid_Speed;
int32_t set2006_2_Angle;
int16_t set2006_2_Speed;
int32_t now2006_2_Angle;
int16_t now2006_2_Speed;
int32_t dErr2006_2;
int16_t last2006_2_Angle = 9000;
int16_t count2006_2=0;
int16_t set2006_2_output;
uint16_t t2=0;

void resurgence(void)
{
	if (RC_Ctl.rc.ch[4] < 0) set2006_2_Speed = 1500;
	else if (now2006_2_Angle > 0) set2006_2_Speed = -1500;
	else set2006_2_Speed = 0;
	
//	if (t2>200)
//		{
//			printf("%d \r\n",now2006_2_Angle);
//			t2=0;
//		}
//		else t2++;
	
}

void feedback_2(void)
{
	
	if (last2006_2_Angle == 9000) last2006_2_Angle = M2006_2.Mechanical_angle;
	
	if (M2006_2.Mechanical_angle - last2006_2_Angle > 4500) count2006_2--;
	else if(M2006_2.Mechanical_angle - last2006_2_Angle <-4500) count2006_2++;
	
	now2006_2_Angle = 8191*count2006_2 + M2006_2.Mechanical_angle;
	last2006_2_Angle = M2006_2.Mechanical_angle;
	
	now2006_2_Speed = M2006_2.Rotor_speed;

	
}

static void PidInit_2(M2006_PID_t *pid, float maxout, float max_iout, float kp, float ki, float kd)
{
    if (pid == NULL)
    {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;
}
static float PidControl_2(M2006_PID_t *pid, float get, float set, float dErr)
{
    float err;
		
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;

    err = set - get;
    pid->err = err;
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * (pid->err - dErr); //由于pid有周期性，所以微分直接为本次误差-上次误差
		if (pid->Iout > pid->max_iout)            pid->Iout = pid->max_iout;
    else if (pid->Iout < -pid->max_iout)      pid->Iout = -pid->max_iout;
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    if (pid->out > pid->max_out)            pid->out = pid->max_out;
    else if (pid->out < -pid->max_out)      pid->out = -pid->max_out;    
		dErr=pid->err;
		return pid->out;
}

void M2006_2_speedSet(void)
{
	//set3508_output = PidControl(&M3508_pid, now3508_Angle, set3508_Angle, dErr1);//pid调节yaw轴输出
	//set3508_output = PidControl(&M3508_pid_Speed, now3508_Speed, set3508_Speed, dErr2);
	set2006_2_output = PidControl_2(&M2006_2_pid_Speed, now2006_2_Speed, set2006_2_Speed, dErr2006_2);
	//pitchSpeed = gimbalPidContr0l(&gimbalPid_pitch, nowPitch, setPitch, dErr_pitch);//pid调节pitch轴输出
	//get_bullet_Send(yawSpeed,pitchSpeed); //max speed=30000 min speed=-30000
}


void M2006Init(void)
{
								// （Gimbal_PID_t *pid ， maxOut ，maxIout ，P ，I ，D）
	//PidInit(&M3508_pid, PID_MAX_OUT, PID_MAX_IOUT, PID_KP, PID_KI, PID_KD);
//	PidInit(&M3508_pid_Speed, Speed_M3508_PID_MAX_OUT, Speed_M3508_PID_MAX_IOUT, Speed_M3508_PID_KP, Speed_M3508_PID_KI, Speed_M3508_PID_KD);
	PidInit_2(&M2006_2_pid_Speed, Speed_M2006_2_PID_MAX_OUT, Speed_M2006_2_PID_MAX_IOUT, Speed_M2006_2_PID_KP, Speed_M2006_2_PID_KI, Speed_M2006_2_PID_KD);
	//gimbalPidInit(&gimbalPid_pitch, PITCH_PID_MAX_OUT, PITCH_PID_MAX_IOUT, PITCH_PID_KP, PITCH_PID_KI, PITCH_PID_KD);
}

void help_task(void)
{
		feedback_2();
		resurgence();
		M2006_2_speedSet();
		
		u8 MotorData[2];

		MotorData[0] = set2006_2_output >> 8;  
    MotorData[1] = set2006_2_output & 0xFF;   


		CAN1_Send_Msg(0x200, MotorData, 0x02); //6020 ID 1-4 : 0x1ff  ;  ID 5-7 : 0x2ff
}
	


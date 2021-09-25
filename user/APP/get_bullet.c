
#include "main.h"

//两个摩擦轮M3508 
PID_t M3508_pid_Speed;
int32_t set3508_Angle;
int16_t set3508_Speed;
int32_t now3508_Angle;
int16_t now3508_Speed;
int32_t dErr1,dErr2;
int16_t last3508_Angle = 9000;
int16_t count=0;
int16_t set3508_output,set3508_output_f;
u16 t=0;

PID_t M2006_pid_Speed;
int32_t set2006_Angle;
int16_t set2006_Speed;
int32_t now2006_Angle;
int16_t now2006_Speed;
int32_t dErr2006;
int16_t last2006_Angle = 9000;
int16_t count2006=0;
int16_t set2006_output;//,set2006_output_f;


void air_operated(void)
{
	if (RC_Ctl.rc.ch[4] != 0) GPIO_SetBits(GPIOH, GPIO_Pin_2);
	else GPIO_ResetBits(GPIOH, GPIO_Pin_2);
}

void motor(void)
{
	if (RC_Ctl.rc.s1 == 1  &&  (now3508_Angle  < 82000))   //限位80000
	{
			//set3508_Angle = 80000;
			set3508_Speed = 800;
	}
	else if (RC_Ctl.rc.s1 ==2 && (now3508_Angle > 12000))   //限位3000
		{
			//set3508_Angle = 3000;
			set3508_Speed = -800;

		}
		else set3508_Speed=0;
	
		
		if (RC_Ctl.rc.s1 ==3 )
		{
			//set3508_Angle = 3000;
			set3508_Speed = 0;
		}
		
//		if (t>200)
//		{
//			printf("%d \r\n",now2006_Angle);
//			t=0;
//		}
//		else t++;
		if (RC_Ctl.rc.s2 ==3  &&  (now2006_Angle  < 3800))   //限位80000
	{
			//set3508_Angle = 80000;
			set2006_Speed = 600;
	}
	else if (RC_Ctl.rc.s2 == 2 && (now2006_Angle > -75000))   //限位3000
	{
			//set3508_Angle = 3000;
		set2006_Speed = -600;
	
	}
	else set2006_Speed=0;
	
		
	if (RC_Ctl.rc.s1 ==1 )
	{
			//set3508_Angle = 3000;
		set2006_Speed = 0;
	}
		
		
		
}

void feedback(void)
{
	if (last3508_Angle == 9000) last3508_Angle = M3508_5.Mechanical_angle;
	
	if (M3508_5.Mechanical_angle - last3508_Angle > 4500) count--;
	else if(M3508_5.Mechanical_angle - last3508_Angle <-4500) count++;
	
	now3508_Angle = 8191*count + M3508_5.Mechanical_angle;
	last3508_Angle = M3508_5.Mechanical_angle;
	
	now3508_Speed = M3508_5.Rotor_speed;
	
	
	if (last2006_Angle == 9000) last2006_Angle = M2006_1.Mechanical_angle;
	
	if (M2006_1.Mechanical_angle - last2006_Angle > 4500) count2006--;
	else if(M2006_1.Mechanical_angle - last2006_Angle <-4500) count2006++;
	
	now2006_Angle = 8191*count2006 + M2006_1.Mechanical_angle;
	last2006_Angle = M2006_1.Mechanical_angle;
	
	now2006_Speed = M2006_1.Rotor_speed;
	
}

static void PidInit(PID_t *pid, float maxout, float max_iout, float kp, float ki, float kd)
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

static float PidControl(PID_t *pid, float get, float set, float dErr)
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

//设定每个电机旋转输出的速度
void M3508_speedSet(void)
{
	//set3508_output = PidControl(&M3508_pid, now3508_Angle, set3508_Angle, dErr1);//pid调节yaw轴输出
	set3508_output = PidControl(&M3508_pid_Speed, now3508_Speed, set3508_Speed, dErr2);
	set2006_output = PidControl(&M2006_pid_Speed, now2006_Speed, set2006_Speed, dErr2006);
	//pitchSpeed = gimbalPidContr0l(&gimbalPid_pitch, nowPitch, setPitch, dErr_pitch);//pid调节pitch轴输出
	//get_bullet_Send(yawSpeed,pitchSpeed); //max speed=30000 min speed=-30000
}

//初始化3508
void M3508Init(void)
{
								// （Gimbal_PID_t *pid ， maxOut ，maxIout ，P ，I ，D）
	//PidInit(&M3508_pid, PID_MAX_OUT, PID_MAX_IOUT, PID_KP, PID_KI, PID_KD);
	PidInit(&M3508_pid_Speed, Speed_M3508_PID_MAX_OUT, Speed_M3508_PID_MAX_IOUT, Speed_M3508_PID_KP, Speed_M3508_PID_KI, Speed_M3508_PID_KD);
	PidInit(&M2006_pid_Speed, Speed_M2006_PID_MAX_OUT, Speed_M2006_PID_MAX_IOUT, Speed_M2006_PID_KP, Speed_M2006_PID_KI, Speed_M2006_PID_KD);
	//gimbalPidInit(&gimbalPid_pitch, PITCH_PID_MAX_OUT, PITCH_PID_MAX_IOUT, PITCH_PID_KP, PITCH_PID_KI, PITCH_PID_KD);
}
//
void get_bullet_task(void)
{
	M3508Init();
	while(1)
	{
		delay_ms(3);
		feedback();
		air_operated();
		motor();
		M3508_speedSet();
		set3508_output_f=-set3508_output;
		u8 MotorData[6];
    MotorData[0] = set3508_output >> 8;  
    MotorData[1] = set3508_output & 0xFF;             
    MotorData[2] = set3508_output_f >> 8;
    MotorData[3] = set3508_output_f & 0xFF;
		MotorData[4] = set2006_output >> 8;  
    MotorData[5] = set2006_output & 0xFF;   


		CAN1_Send_Msg(0x1ff, MotorData, 0x06); //6020 ID 1-4 : 0x1ff  ;  ID 5-7 : 0x2ff
	}

	
}


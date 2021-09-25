#include "main.h"
#include "shoot.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
shoot_PID_t shootPid2006_1;                       //2006拨弹电机PID定义

shoot_PID_t shootPid;															//3508摩擦电机PID定义

shoot_PID_t shootPid6020;													//6020供弹电机PID定义

int16_t inputShootData;														//输入的发射相关数据

int16_t setShootSpeed;														//设置大弹丸摩擦轮速度

int16_t shootMotor2006_1,shootM4Speed;        		//拨弹轮速度和供弹轮速度

int16_t dErr_M3[3]={0};														//2个3508和1个6020 pid中的误差项

int16_t dErr_M2006; 															//2006电机中的pid的误差项

int16_t nowM5Speed,nowM6Speed;										//2个3508和1个6020 pid中的当前值项

int16_t nowGM4Speed;				

int16_t setM5Speed,setM6Speed;										//个6020 pid中的设定值项

int16_t setGM4Speed;				
//int16_t nowAngle2006_1;														//2006电机中的pid当前值项


//int16_t setAngle2006_1;														//2006电机中的pid设定值项
int16_t setSpeed2006_1;
int16_t nowSpeed2006_1;
int16_t setA2006_1;
int16_t setGM4Speed_out;
u8 inputFireData_2006_1;
u16 pressTime_l,pressTime_r;
u8 open = 0;
u8 en = 0;
u16 delay;
u8 fire = 0;
//static int16_t ecd_count1 = 0;
int16_t lastAngle2006_1=0;
u8 first = 1;
u8 fashe_flag=1;

void fashe()
{
  if(RC_Ctl.key.v&KEY_PRESSED_OFFSET_F)
{
	fashe_flag=2;
}
  else if(RC_Ctl.key.v&KEY_PRESSED_OFFSET_G)
{
	fashe_flag=3;
}
  if (RC_Ctl.rc.s1!=1)
{
	fashe_flag=1;
}
}

//static int16_t last_angle = 9000;
//static int16_t count=0;
//static int32_t num=0;
	
void shootSend(u16 shootMotor2006_1)
{
    u8 shootMotorData[2];
//    shootMotorData[0] = shootMotor1 >> 8;
//    shootMotorData[1] = shootMotor1 & 0xFF;
//    shootMotorData[2] = shootMotor2 >> 8;
//    shootMotorData[3] = shootMotor2 & 0xFF;				//控制电调id为 5，6  3508电机 
		shootMotorData[0] = shootMotor2006_1 >> 8;
    shootMotorData[1] = shootMotor2006_1 & 0xFF;
	  CAN2_Send_Msg(0x200, shootMotorData, 0x02);    //控制电调id为 1，2006 
		
//		u8 shootMotorData2[4];
//    shootMotorData2[0] = shootMotor6020>> 8;
//    shootMotorData2[1] = shootMotor6020 & 0xFF;
//			shootMotorData2[2] = shootMotor2006_1 >> 8;           
//    shootMotorData2[3] = shootMotor2006_1 & 0xFF;	
//	  CAN2_Send_Msg(0x1ff, shootMotorData2, 0x04);
	

}

void shootConversion(void)   //输入信号处理转换
{		
	
		fashe();
	
		if((RC_Ctl.rc.ch[4]>100)||(RC_Ctl.mouse.press_l==1)) 
		{
		setSpeed2006_1 = 4000;  
		} //2006 拨弹 电机 设置 速度 			//顺时 针为 电机 反转
		
		else if((RC_Ctl.rc.ch[4]<-100)||(RC_Ctl.mouse.press_r==1))
		{
			setSpeed2006_1 = -1500;
		}
		else setSpeed2006_1 = 0;
		
		
		
		if(RC_Ctl.rc.s1==3||fashe_flag==3)
		{
       set_fric();
		}
		else if(RC_Ctl.rc.s1==2||fashe_flag==2)
		{
		  set_fric2();
		}
//		else
//		  set_fric();
		
//		if(RC_Ctl.rc.s1==1)
//		{
//			 set_fric();
//		}
//		if(RC_Ctl.rc.s1==3)
//		{
//			 set_fric2();
//		}
}

void shootFeedback(void)  //存入的是速度，不是角度
{
	//nowGM4Speed=GM6020_give.Rotor_speed;
	//nowM5Speed=M3508_5.Rotor_speed;
	//nowM6Speed=M3508_6.Rotor_speed;
	//nowAngle2006_1=M2006_1.Mechanical_angle;
	nowSpeed2006_1=M2006_1.Rotor_speed;

}


static void shootPidInit(shoot_PID_t *pid, float maxout, float max_iout, float kp, float ki, float kd)
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
		
//		printf("42摩擦轮pid初始化完成\r\n");
}

static float shootPidControl(shoot_PID_t *pid, float get, float set, u8 i)
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
    pid->Dout = pid->kd * (pid->err - dErr_M3[i]); //由于pid有周期性，所以微分直接为本次误差-上次误差
		if (pid->Iout > pid->max_iout)            pid->Iout = pid->max_iout;
    else if (pid->Iout < -pid->max_iout)      pid->Iout = -pid->max_iout;
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    if (pid->out > pid->max_out)            pid->out = pid->max_out;
    else if (pid->out < -pid->max_out)      pid->out = -pid->max_out;    
		dErr_M3[i]=pid->err;
		return pid->out;
}
static float shootPidContr02(shoot_PID_t *pid, float get, float set, u8 i)//2006  这个i好像没啥用
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
    pid->Dout = pid->kd * (pid->err - dErr_M2006); //由于pid有周期性，所以微分直接为本次误差-上次误差
		if (pid->Iout > pid->max_iout)            pid->Iout = pid->max_iout;
    else if (pid->Iout < -pid->max_iout)      pid->Iout = -pid->max_iout;
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    if (pid->out > pid->max_out)            pid->out = pid->max_out;
    else if (pid->out < -pid->max_out)      pid->out = -pid->max_out;    
		dErr_M2006=pid->err;
		return pid->out;
}


void shootSpeedSet(void)
{
	
	setGM4Speed_out = shootPidControl(&shootPid6020, nowGM4Speed, setGM4Speed, 0);//PID调节M5电机输出  6020
	setM5Speed = shootPidControl(&shootPid, nowM5Speed, setM5Speed, 1);//PID调节M5电机输出 3508
	setM6Speed = shootPidControl(&shootPid, nowM6Speed, setM6Speed, 2);//PID调节M6电机输出 3508

}
void shootAngleSet(void)  //射击 角度 设置  // 
{
	setA2006_1 = shootPidContr02(&shootPid2006_1 , nowSpeed2006_1, setSpeed2006_1, 3);  //  2006

}


void shootInit(void)//电机初始化
{
	//shootPidInit(&shootPid, SHOOT_M3508_PID_MAX_OUT, SHOOT_M3508_PID_MAX_IOUT, SHOOT_M3508_PID_KP, SHOOT_M3508_PID_KI, SHOOT_M3508_PID_KD);
	shootPidInit(&shootPid2006_1, SHOOT_2006_PID_MAX_OUT, SHOOT_2006_PID_MAX_IOUT, SHOOT_2006_PID_KP, SHOOT_2006_PID_KI, SHOOT_2006_PID_KD);
	//shootPidInit(&shootPid6020, SHOOT_6020_PID_MAX_OUT, SHOOT_6020_PID_MAX_IOUT, SHOOT_6020_PID_KP, SHOOT_6020_PID_KI, SHOOT_6020_PID_KD);
}

void shootTask(void)//发射任务 调用所有函数
{
	shootFeedback();   //射击 反馈       //can1 获取两个摩擦轮3508速度 can2 获取2006
	shootConversion(); //2006 射击 转换       //s1控制 两个3502，一个6020供弹，一个2006拨弹 的转与停，以及转速
	
	//shootSpeedSet();   //6020 3508 射击 速度 设置  //  
	shootAngleSet();   //射击 角度 设置  //pid  
	shootSend(setA2006_1); //射击 发送
//	shootSend(setM5Speed,setM6Speed,setA2006_1,setGM4Speed_out); //射击 发送
//	else shootSend(setM5Speed,setM6Speed,0,setGM4Angle_out);
//	
//	if (RC_Ctl.rc.ch[4] > 600)
//	shootSend(setM5Speed,setM6Speed,0,500);
//	else shootSend(setM5Speed,setM6Speed,0,0);
	
//	shootSend(0,0,setA2006_1,0);
}
void shoot_task(void *pvParameters)
{
	shootInit();
	while(1)
	{
		
		shootTask();
		vTaskDelay(10);
		   
	}
	
	
}


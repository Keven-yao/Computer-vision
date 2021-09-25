/*****************************************************
name:gimbal.c
brief:云台控制文件
function:通过收到的遥控信号、反馈、以及其他信号来控制云台的转向
author:CadenLei
date:V1.0:2019.4.20
date:V2.0:2019.4.30 CadenLei Implementing PID algorithm.
date:V2.1:2020.1.12 Caden 为新版英雄机器人适配代码，主要修改：
										1）取消小弹丸发射机构
										2）增加一个6020云台pitch轴电机
										3）供弹机构为6020驱动，再带一个2006枪口限位拨弹机构

*******************************************************/
#include "stm32f4xx.h"
#include "gimbal.h"
#include "imu.h"
#include "main.h"
#include "start_task.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Chassis.h"
//#define midYaw 4100        //yaw轴中点位置

//#define midPitch 2700         //pitch轴中点位置

int16_t inputYawData;          //遥控输入 水平 角 数据
int16_t inputPitchData;        //遥控输入 俯仰 角 数据

int16_t ANflag=0; //按下键盘按键标识位
//int16_t remote_flag; //遥控器按键按键标识
float remote_gread=0.1;//小陀螺等级系数
int16_t remote_move_flag=0;//判断是否为小陀螺状态下移动系数

int16_t setYaw=midYaw,setPitch=midPitch,setRoll=6800;              //设定的位置初始为中点

int16_t addYaw,addPitch;															//yaw轴增添量/pitch轴增添量

int16_t leftLimit,rightLimit,upLimit,downLimit;       //上下左右限位位置变量

int16_t nowYaw,nowPitch,nowRoll,oldYaw;															//当前两个轴的位置   由can总线反馈的数据提供

int16_t yawSpeed,pitchSpeed,rollSpeed,addspeed=-40;//speed=-42											//两轴速度

volatile float yaw_direction;

int16_t yaw_direction_flag;

Gimbal_PID_t gimbalPid_yaw,gimbalPid_pitch,gimbalPid_roll;           //定义云台pid数据结构

static float dErr_yaw = 0;														//yaw轴误差
static float dErr_pitch = 0;													//pitch轴误差
static float dErr_roll = 0;													//roll轴误差
float multiple=0.17f;

void test_keybord()
{
	if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_Q)
	{
		ANflag=2;
		remote_gread=0.1;
	}
	else if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_E)
	{
		ANflag=3;
		remote_gread=0.08;
	}
	else if(RC_Ctl.rc.s2 !=1)
	{
		ANflag=1;
		remote_gread=0.05;
	}
		
}

void judje_mote_move()
{
	if(ANflag==2)
	{
		if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_W)
		{
			remote_move_flag=1;
		}
		else if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_S)
		{
			remote_move_flag=1;
		}
		else if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_A)
		{
			remote_move_flag=1;
		}
		else if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_D)
		{
			remote_move_flag=1;
		}
		else
			remote_move_flag=0;
	}
	else 
	{
		remote_move_flag=0;
	}
}

void top_speed() //小陀螺设置档位速度
{
	//judje_mote_move();
	if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_C )
	{
		addspeed=-40 ;
		//if(remote_move_flag==1)
//		{
//			addspeed+=10;
//		}
		
	}
	else if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_V)
	{
		addspeed=-44;
//		if(remote_move_flag==1)
//		{
//			addspeed+=20;
//		}
		
	}
	else if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_B)
	{
		addspeed=-48;
//		if(remote_move_flag==1)
//		{
//			addspeed+=30;
//		}
	}
}



/*gimbal's signal data send by can1
**now yawMotor ID = 5   pitchMotor1 ID = 6   pitchMotor2 ID = 7*/
void gimbalSend(int16_t chassisMotor1, int16_t chassisMotor2, int16_t chassisMotor3) //CAN1 发送云台数据   先存入，再发
{
    u8 gimebalMotorData[6];
    gimebalMotorData[0] = chassisMotor1 >> 8;      //存入 高八位  电机 数据    0 1 2 3 4 5 6 7 _ _ _ _ _ _ _ _
    gimebalMotorData[1] = chassisMotor1 & 0xFF;    //存入 低八位  电机 数据                    0 1 2 3 4 5 6 7  
		gimebalMotorData[2] = chassisMotor2 >> 8;			 //存入 高八位  电机 数据 	
    gimebalMotorData[3] = chassisMotor2 & 0xFF;    //存入 高八位  电机 数据 dd
		gimebalMotorData[4] = chassisMotor3 >> 8;			 //存入 高八位  电机 数据 	
    gimebalMotorData[5] = chassisMotor3 & 0xFF;    //存入 高八位  电机 数据 
		CAN1_Send_Msg(0x2FF, gimebalMotorData, 0x06); //6020 ID 1-4 : 0x1ff  ;  ID 5-7 : 0x2ff
}

//后期改成自动完成!!!
void limitSet(void)//限位     0 ~ 8191   8190/360=22.75
{
	leftLimit=  5100;             //  水平 左限 
	rightLimit=	3100;							//  水平 右限 (5100-3100)/22.75=2000/22.72=88°C
	//(5100+3100)/2=4100 水平中轴   硬件连接决定
	upLimit=		2200;							//  垂直 下限
	downLimit=	2850;			  				//  垂直 上限 (7500-6830)/22.75=670/22.75=30°C 
	//(7500-6830)/2=7165 垂直中轴   硬件连接决定	
}

/*************************************************
角度换算
获取信道数据，将其转换成角度增加值
判断是否超过限位值 超过的话返回限位值
**************************************************/
//extern volatile float yaw_angle,pitch_angle,roll_angle,gg_x,gg_y,gg_z,ga_x,ga_y,ga_z,ga_y1,kg,ga_y3,kg3,ga_y22,ga_y33;
void angleConversion(void)           //角度 转换    遥控 输入 云台 水平角 俯仰角 数据   
{
	test_keybord();
	top_speed();
	
	//printf("%c\r\n",RC_Ctl.key.v & KEY_PRESSED_OFFSET_Q);
	TIM_Cmd(TIM2, ENABLE);	
	inputYawData=(RC_Ctl.rc.ch[2]*0.1 + RC_Ctl.mouse.x*20*remote_gread);
	inputPitchData= RC_Ctl.rc.ch[3]*5.0 + RC_Ctl.mouse.y*7;

	addYaw=inputYawData*0.1;                                         
	addPitch=inputPitchData*0.1;  //遥控灵敏度  
//		if(inputPitchData>0&&inputPitchData<500)
//		{
//			addPitch=3;
//		}
//		else if(inputPitchData>-500&&inputPitchData<0)
//		{
//			addPitch=-3;
//		}
//				else if(inputPitchData>500)
//		{
//			addPitch=10;
//		}
//		else if(inputPitchData<-500)
//		{
//			addPitch=-10;
//		}
/********************************************************************/   //Pitch 6020
  if (inputPitchData != 0)               //垂直 上下限位
		{				
			if (setPitch+addPitch <= downLimit && setPitch+addPitch >= upLimit)  setPitch = setPitch + addPitch;
			else if (setPitch+addPitch > downLimit) setPitch = downLimit;
			else setPitch = upLimit;				
		}	
//		if (addPitch != 0)               //垂直 上下限位
//		{
//			
//			if (nowPitch+addPitch <= downLimit && nowPitch+addPitch >= upLimit)  setPitch = nowPitch + addPitch;
//			else if (nowPitch+addPitch > downLimit) setPitch = downLimit;
//			else setPitch = upLimit;
//			
//		}	
	
/********************************************************************/   //Yaw 6020		
	if ((RC_Ctl.rc.s2 == 3)||(ANflag==3))            //水平 复位
	{
		
		
		if(Yaw_flag==1)
		{
				setYaw+=addspeed+addYaw;
		}
		
		else if(Yaw_flag==0)
		{
			setYaw = midYaw;    //midYaw 4100   //yaw轴  水平轴 中点位置 
		}			
	}
	else if((RC_Ctl.rc.s2 == 2)||(ANflag==2))               //水平 陀螺仪
	{		
		Yaw_flag=1;	
		setYaw+=addspeed+addYaw;

	}
/********************************************************************/   //Roll 6020
//			setRoll = setRoll-(int16_t)(angle[1]*22.75f*0.05f);

	
	
}

//获取feedback中得到的电机角度值
void angleFeedback(void)   ////角度 反馈    从 can1 接收 现在 云台 水平俯仰 角 数据
{	
	nowYaw=GM6020_Yaw.Mechanical_angle;      //现在 水平 角 //////////（获取的是角度，不是速度）
	nowPitch=GM6020_Pitch.Mechanical_angle;  //现在 俯仰 角
	nowRoll=GM6020_Roll.Mechanical_angle;		 //现在 偏航 角
		
}


static void gimbalPidInit(Gimbal_PID_t *pid, float maxout, float max_iout, float kp, float ki, float kd)//云台 pid 初始化 
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
		
		//printf("云台pid初始化完成\r\n");
}
static float gimbalPidContr0l(Gimbal_PID_t *pid, float get, float set, float dErr)// 云台 pid 控制    pid算法  
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
		return pid->out;                          //将要设置 云台 水平角度 俯仰 角 数据
}

//设定每个电机旋转输出的速度
void speedSet(void)         //云台 角度 设置    将要设置 云台 水平角 俯仰角 数据  （通过pid算法得出) 
{
	if(nowYaw>6100&&oldYaw<2100)
			{
				setYaw=setYaw+8191;
			}
			
	else if(nowYaw<2100&&oldYaw>6100)
			{
				setYaw=setYaw-8191;
			}
	yawSpeed = gimbalPidContr0l(&gimbalPid_yaw, nowYaw, setYaw, dErr_yaw);          //pid调节yaw轴输出      水平角度
	pitchSpeed = gimbalPidContr0l(&gimbalPid_pitch, nowPitch, setPitch, dErr_pitch);//pid调节pitch轴输出    俯仰角度
	//pitchSpeed=pitchSpeed+20000;
	//USART6_TX_Byte(pitchSpeed);
			//printf(pitchSpeed);
	rollSpeed = gimbalPidContr0l(&gimbalPid_roll, nowRoll, setRoll, dErr_roll);
//  printf("%d =%d = %d\r\n",nowYaw,yawSpeed,setYaw);
//			if(Yaw_flag==1&&(nowYaw<=599&&nowYaw>595))
//			{
//				yawSpeed-=2000;
//			}
//			if(Yaw_flag==1&&(nowYaw<=595&&nowYaw>590))
//			{
//				yawSpeed-=4000;
//			}
//			if(Yaw_flag==1&&(nowYaw<=590&&nowYaw>585))
//			{
//				yawSpeed-=8000;
//			}
//			if(Yaw_flag==1&&(nowYaw<=5855&&nowYaw>580))
//			{
//				yawSpeed-=8000;
//			}
//			else if(Yaw_flag==1&&(nowYaw<=580&&nowYaw>570))
//			{
//				yawSpeed-=8000;
//			}
//			else	if(Yaw_flag==1&&(nowYaw<=570&&nowYaw>560))
//			{
//				yawSpeed-=8000;
//			}
//			else	if(Yaw_flag==1&&(nowYaw<=560&&nowYaw>500))
//			{
//				yawSpeed-=8000;
//			}
	gimbalSend(yawSpeed,pitchSpeed,rollSpeed); //max speed=30000 min speed=-30000  //can1 发送 云台 水平俯仰 偏转 数据
	oldYaw=nowYaw;
}


void gimbalInit(void) //云台 pid 初始化  gimbalPid_yaw 云台水平pid   gimbalPid_pitch 云台俯仰pid
{
								// （Gimbal_PID_t *pid ， maxOut ，maxIout ，P ，I ，D）
	gimbalPidInit(&gimbalPid_yaw, YAW_PID_MAX_OUT, YAW_PID_MAX_IOUT, YAW_PID_KP, YAW_PID_KI, YAW_PID_KD);
	gimbalPidInit(&gimbalPid_pitch, PITCH_PID_MAX_OUT, PITCH_PID_MAX_IOUT, PITCH_PID_KP, PITCH_PID_KI, PITCH_PID_KD);
	gimbalPidInit(&gimbalPid_roll, ROLL_PID_MAX_OUT, ROLL_PID_MAX_IOUT, ROLL_PID_KP, ROLL_PID_KI, ROLL_PID_KD);
}


void gimbalTask(void)//云台任务 调用所有函数
{
	limitSet();        //限制 设置     //设置     云台 水平左右限位 垂直上下限位    限位：当电机转过限位角度后 ，再发送装动命令时，电机角度不变
	angleConversion(); //角度 转换     //读取     遥控器 云台 水平俯仰  偏转 设置 数据   
	angleFeedback();   //角度 反馈     //读取     云台 水平俯仰 偏转 数据          can1 接收 现在 云台 水平俯仰 偏转 数据
	speedSet();        //角度 设置     //将要设置 云台 水平偏转 俯仰偏转 数据      can1 发送 云台 水平俯仰 偏转 数据
}



void gimbal_Task(void *pvParameters)
{
	gimbalInit();
  	
	while(1)
	{
		gimbalTask();
		vTaskDelay(10);
	}
	
}



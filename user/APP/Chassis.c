#include "stm32f4xx.h"
//#include "sys.h"


#include "main.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"


//小陀螺平移力增益和角度偏移修正
//#define chassisRotationProportion 0.75f
float chassisRotationProportion =0.75;
//#define chassisRotationProportion 1.0f
#define chassisAngleOffset -1.0f
#define PI 3.14159265358979f
int16_t chassisRotateSpeed = 0;

int16_t inputVxData; 
int16_t inputVyData;
int16_t inputWzData;
int16_t Yaw_flag=0;
float gears_flag=1;//速度档位变量

float Chassis_flag=1;//底盘超功率限制系数

int16_t setVx,setVy,setWz;

int16_t setW_S_A_Dspeed =180;//设置底盘速度系数

int16_t set_mouse_speed=10; //设置鼠标移动系数

int16_t addVx,addVy,addWz;

int16_t nowM1Speed,nowM2Speed,nowM3Speed,nowM4Speed;
int16_t setM1Speed,setM2Speed,setM3Speed,setM4Speed;

int16_t A_D_speed,W_S_speed;

float speedControl;

Chassis_PID_t chassisPid;
static float dErr_M[5] = {0};



//向can总线发送器传输电机速度的控制数据
void chassisSend(int16_t chassisMotor1, int16_t chassisMotor2, int16_t chassisMotor3, int16_t chassisMotor4) //底盘 发送
{
    u8 chassisMotorData[8];   // 底盘 电机 数据
    chassisMotorData[0] = chassisMotor1 >> 8;     //存入 高八位 底盘 电机 数据    0 1 2 3 4 5 6 7 _ _ _ _ _ _ _ _
    chassisMotorData[1] = chassisMotor1 & 0xFF;   //存入 低八位 底盘 电机 数据                    0 1 2 3 4 5 6 7
    chassisMotorData[2] = chassisMotor2 >> 8;
    chassisMotorData[3] = chassisMotor2 & 0xFF;
    chassisMotorData[4] = chassisMotor3 >> 8;
    chassisMotorData[5] = chassisMotor3 & 0xFF;
    chassisMotorData[6] = chassisMotor4 >> 8;
    chassisMotorData[7] = chassisMotor4 & 0xFF;
    CAN1_Send_Msg(0x200, chassisMotorData, 0x08);   //can2发送底盘 四个 电机 数据
}

void fix_speed()  //底盘设置初始速度
{
	if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_D)
	{
		A_D_speed=1;
	}
	else if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_A)
	{
		A_D_speed=-1;
	}
	else A_D_speed=0;
	if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_W)
	{
		W_S_speed=1;
	}
	else if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_S)
	{
		W_S_speed=-1;
	}
	else W_S_speed=0;
		
}

void gears_speed() //底盘设置档位速度
{
	if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_C )
	{
		gears_flag=1;
		Chassis_flag=1;
		chassisRotationProportion=0.75;
		//chassisRotationProportion=0.85;
	}
	else if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_V)
	{
		gears_flag=1.2;
		Chassis_flag=1.10;
		chassisRotationProportion=1.0;
		//chassisRotationProportion=1.1;
	}
	else if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_B)
	{
		gears_flag=1.3;
		Chassis_flag=1.2;
		chassisRotationProportion=1.2;
		//chassisRotationProportion=1.3;
	}
	
	
}
//////////////////////////////////////////////////////////////////////////////////
void chassisConversion(void)//底盘 转换  读取遥控器底盘设置数据
{
	fix_speed();
	gears_speed();
	if (Yaw_flag == 1)
	{
		if (((RC_Ctl.rc.s2 == 3)||(ANflag==3)) && ((GM6020_Yaw.Mechanical_angle < midYaw + 122) && (GM6020_Yaw.Mechanical_angle > midYaw - 122)))
		{
			Yaw_flag = 0;
		}
		
		
		chassisRotateSpeed = setM1Speed = setM2Speed = setM3Speed = setM4Speed = 2822*Chassis_flag; //默认的自转

		float angleYaw = GM6020_Yaw.Mechanical_angle / 8192.0f;
		angleYaw = 1.0f + angleYaw + chassisAngleOffset; //底盘坐标系下的前方向      底盘极坐标系以后为正方向，顺时针递增
		float dir_x = RC_Ctl.rc.ch[0]+A_D_speed*setW_S_A_Dspeed*gears_flag;  //输入方向向量的向右正方向分量
		float dir_z = RC_Ctl.rc.ch[1]+W_S_speed*setW_S_A_Dspeed*gears_flag;//输入方向向量的向前正方向分量
		float dir_scale = fmaxf(fabsf(dir_x), fabsf(dir_z)) / sqrtf(dir_x * dir_x + dir_z * dir_z); //标准化后的输入方向向量的长度
		if (isnan(dir_scale))
			dir_scale = 0.0f;
		float dir_angle = 0.25f - atan2f(dir_z, dir_x) / 2 / PI; //输入方向向量的方向角
		if (isnan(dir_angle))
			dir_angle = 0.0f;
		angleYaw += dir_angle; //直接把输入的方向叠加到底盘坐标系下的前方向上，得到底盘坐标系下要前进的方向

		int16_t i, j;
		wheelRotate_t wheels[4] = {{0.375f, &setM1Speed}, {0.625f, &setM2Speed}, {0.875f, &setM3Speed}, {0.125f, &setM4Speed}}; //ID分别为1、2、3、4的轮子
		for (i = 0; i < 4; i++) //计算底盘坐标系下每个轮子的施力方向
		{
			wheels[i].angle = wheels[i].angle - angleYaw + 1.5f;
			while (wheels[i].angle > 1.0f)
				wheels[i].angle -= 1.0f;
		}
		for (i = 0; i < 3; i++) //按施力方向升序排序
		{
			for (j = 0; j < 3 - i; j++)
			{
				wheelRotate_t tmp;
				if (wheels[j].angle > wheels[j + 1].angle)
				{
					tmp = wheels[j];
					wheels[j] = wheels[j + 1];
					wheels[j + 1] = tmp;
				}
			}
		}

		float angleMin = wheels[0].angle, angleMin2Pi = angleMin * PI * 2; //计算每个轮子要加/减的力
		float angleSin = sinf(angleMin2Pi), angleCos = cosf(angleMin2Pi);
		if (angleMin >= 0.125f)
		{
			float forceLarge = chassisRotationProportion * dir_scale / (angleSin + angleCos * angleCos / angleSin);
			float forceSmall = forceLarge / angleSin * angleCos;

			*(wheels[0].force) *= 1 + forceLarge; //叠加到自转力上得到自转向前的合力
			*(wheels[1].force) *= 1 + forceSmall;
			*(wheels[2].force) *= 1 - forceLarge;
			*(wheels[3].force) *= 1 - forceSmall;
		}
		else
		{
			float forceLarge = chassisRotationProportion * dir_scale / (angleCos + angleSin * angleSin / angleCos);
			float forceSmall = forceLarge * angleSin / angleCos;

			*(wheels[0].force) *= 1 + forceSmall;
			*(wheels[1].force) *= 1 + forceLarge;
			*(wheels[2].force) *= 1 - forceSmall;
			*(wheels[3].force) *= 1 - forceLarge;
		}
	}
	else
	{
//	inputVxData=RC_Ctl.rc.ch[1] + (RC_Ctl.key.v & KEY_PRESSED_OFFSET_W)*100 - (RC_Ctl.key.v & KEY_PRESSED_OFFSET_S)*100; //输入 x轴 数据
//	inputVyData=RC_Ctl.rc.ch[0] + (RC_Ctl.key.v & KEY_PRESSED_OFFSET_D)*80 - (RC_Ctl.key.v & KEY_PRESSED_OFFSET_A)*80;   //输入 y轴 数据
	inputWzData=RC_Ctl.rc.ch[2] + RC_Ctl.mouse.x*set_mouse_speed;  		//输入 z轴 数据
	inputVxData=RC_Ctl.rc.ch[1]+W_S_speed*setW_S_A_Dspeed*gears_flag;
	inputVyData=RC_Ctl.rc.ch[0]+A_D_speed*setW_S_A_Dspeed*gears_flag;
	
		
	addVx=inputVxData*15;
	addVy=inputVyData*15;
	addWz=inputWzData*4;
	
	speedControl = (1 + (RC_Ctl.key.v & KEY_PRESSED_OFFSET_SHIFT)*0.5) * (1 - (RC_Ctl.key.v & KEY_PRESSED_OFFSET_CTRL)*0.5);
	
	setVx = addVx * speedControl; 
	setVy = addVy * speedControl;
	setWz = addWz * speedControl;
	
	
	//假设左前方为ID 1 顺时针排序。
	
	setM1Speed = + setVx + setVy + setWz;  //设置 M1电机 速度 （遥控器发送底盘速度）
	setM2Speed = - setVx + setVy + setWz;  //设置 M2电机 速度
	setM3Speed = - setVx - setVy + setWz;  //设置 M3电机 速度
	setM4Speed = + setVx - setVy + setWz;  //设置 M4电机 速度	
	}
}
////////////////////////////////////////////////////////////////////////////////////
void chassisFeedback(void)          //底盘 反馈     读取can2接收底盘数据   读取底盘数据
{
	nowM1Speed=M3508_1.Rotor_speed;   //现在 M1电机速度   can2 总线接收M1电机速度 （底盘反馈的此时此刻速度）
	nowM2Speed=M3508_2.Rotor_speed;   //现在 M2电机速度   can2 总线接收M2电机速度 
	nowM3Speed=M3508_3.Rotor_speed;   //现在 M3电机速度   can2 总线接收M3电机速度 
	nowM4Speed=M3508_4.Rotor_speed;   //现在 M4电机速度   can2 总线接收M4电机速度 
}

//（chassisPid 存入pid各个参数）
static void chassisPidInit(Chassis_PID_t *pid, float maxout, float max_iout, float kp, float ki, float kd)  //底盘 pid 初始化
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
		
//		printf("底盘pid初始化完成\r\n");
}

static float chassisPidContr0l(Chassis_PID_t *pid, float get, float set, u8 i)    //底盘 pid 控制   （pid算法）
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
    pid->Dout = pid->kd * (pid->err - dErr_M[i]); //由于pid有周期性，所以微分直接为本次误差-上次误差
		if (pid->Iout > pid->max_iout)            pid->Iout = pid->max_iout;
    else if (pid->Iout < -pid->max_iout)      pid->Iout = -pid->max_iout;
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    if (pid->out > pid->max_out)            pid->out = pid->max_out;
    else if (pid->out < -pid->max_out)      pid->out = -pid->max_out;    
		dErr_M[i]=pid->err;
		return pid->out;
}

//设定每个电机旋转输出的速度
void chassisSpeedSet(void)    //   底盘速度设置    通过 CAN2 发送  （通过pid算法得出 底盘将要设置的速度）
{
	setM1Speed = chassisPidContr0l(&chassisPid, nowM1Speed, setM1Speed, 1);//底盘 pid 控制      设置 M1电机速度 速度 
	setM2Speed = chassisPidContr0l(&chassisPid, nowM2Speed, setM2Speed, 2);//底盘 pid 控制      设置 M2电机速度 速度
	setM3Speed = chassisPidContr0l(&chassisPid, nowM3Speed, setM3Speed, 3);//底盘 pid 控制      设置 M3电机速度 速度
	setM4Speed = chassisPidContr0l(&chassisPid, nowM4Speed, setM4Speed, 4);//底盘 pid 控制      设置 M4电机速度 速度
	
	
	chassisSend(setM1Speed,setM2Speed,setM3Speed,setM4Speed);                //底盘发送     通过 CAN2 发送   底盘 M1 M2 M3 M4电机 速度 数据
//	chassisSend(1000,-1000,-1000,1000);
}

void chassisInit(void) //底盘初始化   chassisPid 初始化
{
								// （Gimbal_PID_t *pid ， maxOut ，maxIout ，P ，I ，D）
	chassisPidInit(&chassisPid, M3508_PID_MAX_OUT, M3508_PID_MAX_IOUT, M3508_PID_KP, M3508_PID_KI, M3508_PID_KD);//底盘 pid 初始化
	
}


void chassisTask(void)//底盘 任务
{
	chassisConversion();  //底盘 转换             //读取遥控器底盘设置数据
	chassisFeedback();    //底盘 反馈             //读取底盘数据 通过  CAN2  接收
	chassisSpeedSet();    //底盘 速度 设置        //将要设置底盘数据 通过  CAN2  发送
}

void chassis_task(void *pvParameters)
{
	
	//底盘电机初始化
	//delay_ms(50);
   chassisInit();
	
	while(1)
	{
		chassisTask();  
		vTaskDelay(10);
		
	}

}




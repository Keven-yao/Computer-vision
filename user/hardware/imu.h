/**
  *@file test_imu.h
  *@date 2016-12-12
  *@author Albert.D
  *@brief 
  */
  
#ifndef _TEST__IMU_H
#define _TEST__IMU_H
#define M_PI  (float)3.1415926535

#include "stm32f4xx.h"

#include "spi.h"
#include "delay.h"
#include "usart.h"

#include "mpu6500_reg.h"
#include "IST8310_reg.h"

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdarg.h>

#define MPU6500     PFout(6)

/*

#define MPU6500_NSS_High()     GPIO_ResetBits(GPIOF,GPIO_Pin_6)
#define MPU6500_NSS_Low()    GPIO_SetBits(GPIOF,GPIO_Pin_6)
*/
//volatile float yaw_angle,pitch_angle,roll_angle,gg_x,gg_z,gg_y,ga_x,ga_y,ga_z,angle1, angle_dot,ga_y1,kg,angle2, angle_dot2,ga_y2,kg2,angle3, angle_dot3,ga_y3,kg3,ga_y22,ga_y33; //使用到的角度值

typedef struct
{
  int16_t ax;
  int16_t ay;
  int16_t az;
  
  int16_t temp;
  
  int16_t gx;
  int16_t gy;
  int16_t gz;
  
  int16_t mx;
  int16_t my;
  int16_t mz;
}IMUDataTypedef;


extern uint8_t MPU_id;
extern int16_t test;
uint8_t MPU6500_Init(void);
uint8_t MPU6500_Write_Reg(uint8_t const reg, uint8_t const data);
uint8_t MPU6500_Read_Reg(uint8_t const reg);
void IMU_getValues(volatile float * values);
uint8_t IST8310_Init(void);

extern volatile float angle[3];
extern volatile float yaw_temp,pitch_temp,roll_temp;
extern volatile float yaw_angle,pitch_angle,roll_angle,gg_x,gg_y,gg_z,ga_x,ga_y,ga_z,ga_y1,kg,ga_y3,kg3,ga_y22,ga_y33; //使用到的角度值

typedef struct __MPU6050_REAL_Data__
{
    float Accel_X;  //转换成实际的X轴加速度，
    float Accel_Y;  //转换成实际的Y轴加速度，
    float Accel_Z;  //转换成实际的Z轴加速度，
    float Temp;     //转换成实际的温度，单位为摄氏度
    float Gyro_X;   //转换成实际的X轴角加速度，
    float Gyro_Y;   //转换成实际的Y轴角加速度，
    float Gyro_Z;   //转换成实际的Z轴角加速度
		int16_t GyroRawData_Z;
		int16_t GyroRawData_Y;
}MPU6050_REAL_DATA;
extern MPU6050_REAL_DATA MPU6050_read_data;
void IMU_getYawPitchRoll(volatile float * ypr); //更新姿态
void GetPitchYawGxGyGz(void);
float Get_Yaw_Angle(int16_t Gyro_Z);

typedef struct mpu6500_data
{
  float olddata_yaw;//水平角
	float olddata_stich;//俯仰角
	float olddata_roll;//偏航角

} MPU6500_data;

void resultdata(MPU6500_data *fuck_data,float set_x,float set_y,float set_z,float new_x,float new_y,float new_z);
#endif


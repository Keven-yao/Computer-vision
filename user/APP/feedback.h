#ifndef _FEEDBACK_H_
#define _FEEDBACK_H_
#include "stm32f4xx.h"


typedef struct
{
		int16_t Mechanical_angle;								//data[0]-[1]
		int16_t Rotor_speed;										//data[2]-[3]
		int16_t	Torque_current;									//data[4]-[5]
		int8_t Temperature;											//data[6]
		
}feedback_data;


extern feedback_data M3508_1,M3508_2,M3508_3,M3508_4,M3508_5,M3508_6,GM6020_Roll,
											GM6020_Yaw,GM6020_Pitch,M2006_1,M2006_2,GM6020_give,
											GM6020_Pitch2;
extern int32_t num;


#endif

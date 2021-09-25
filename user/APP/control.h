#ifndef __CONTROL_H
#define __CONTROL_H

#include "stm32f4xx.h"
//#include "sys.h"

///* ----------------------- RC Channel Definition---------------------------- */ 
//#define RC_CH_VALUE_MIN              ((uint16_t)364 ) 
//#define RC_CH_VALUE_OFFSET           ((uint16_t)1024) 
//#define RC_CH_VALUE_MAX              ((uint16_t)1684) 
// 
///* ----------------------- RC Switch Definition----------------------------- */ 
//#define RC_SW_UP                     ((uint16_t)1) 
//#define RC_SW_MID                    ((uint16_t)3) 
//#define RC_SW_DOWN                   ((uint16_t)2) 
// 
///* ----------------------- PC Key Definition-------------------------------- */ 
//#define KEY_PRESSED_OFFSET_W         ((uint16_t)0x01<<0)
//#define KEY_PRESSED_OFFSET_S         ((uint16_t)0x01<<1) 
//#define KEY_PRESSED_OFFSET_A         ((uint16_t)0x01<<2) 
//#define KEY_PRESSED_OFFSET_D         ((uint16_t)0x01<<3) 
//#define KEY_PRESSED_OFFSET_Q         ((uint16_t)0x01<<4) 
//#define KEY_PRESSED_OFFSET_E         ((uint16_t)0x01<<5) 
//#define KEY_PRESSED_OFFSET_SHIFT     ((uint16_t)0x01<<6) 
//#define KEY_PRESSED_OFFSET_CTRL      ((uint16_t)0x01<<7) 
// 
//#define  RC_FRAME_LENGTH                            18u 
// 
///* ----------------------- Data Struct ------------------------------------- */ 
//typedef __packed struct 
//{
//     __packed struct
//     {
//         int ch[5];     //Í¨µÀ4Îª²à±ß¹öÂÖ ÆäÓà²éÒ£¿ØÊÖ²á
//         u8  s1;
//         u8  s2;
//     }rc; 
// 
//     __packed struct
//     {
//         int16_t x;
//         int16_t y;
//         int16_t z;
//         u8 press_l;   //×ó¼ü
//         u8 press_r;   //ÓÒ¼ü
//     }mouse; 
// 
//     __packed struct
//     {
//         u16 v;
//     }key; 
//}RC_Ctl_t; 

extern u8 rcTimer;
extern u8 sbusBuf[18];
extern u8 sbusNum;
//extern RC_Ctl_t rcCtrl;

void sbusToRc(void);
void control(void);

#endif

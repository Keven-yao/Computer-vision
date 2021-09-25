#ifndef __USART_H
#define __USART_H


#include "main.h"
#define RC_S1Up 1
#define RC_S1Down 2
#define RC_S1Middle 3

#define RC_S2Up 1
#define RC_S2Down 2
#define RC_S2Middle 3

#define USART_REC_LEN  			8  	//定义最大接收字节数 200
#define EN_USART6_RX 			1		//使能（1）/禁止（0）串口1接收  
#define UART6_Waiting 0
#define UART6_Receiving 1
#define UART6_Success 2 
#define UART6_Failed  3

/* ----------------------- RC Channel Definition---------------------------- */ 
#define RC_CH_VALUE_MIN              ((uint16_t)364 ) 
#define RC_CH_VALUE_OFFSET           ((uint16_t)1024) 
#define RC_CH_VALUE_MAX              ((uint16_t)1684) 
 
/* ----------------------- RC Switch Definition----------------------------- */ 
#define RC_SW_UP                     ((uint16_t)1) 
#define RC_SW_MID                    ((uint16_t)3) 
#define RC_SW_DOWN                   ((uint16_t)2) 
 
/* ----------------------- PC Key Definition-------------------------------- */ 
#define KEY_PRESSED_OFFSET_W         ((uint16_t)0x01<<0)
#define KEY_PRESSED_OFFSET_S         ((uint16_t)0x01<<1) 
#define KEY_PRESSED_OFFSET_A         ((uint16_t)0x01<<2) 
#define KEY_PRESSED_OFFSET_D         ((uint16_t)0x01<<3) 
#define KEY_PRESSED_OFFSET_SHIFT     ((uint16_t)0x01<<4) 
#define KEY_PRESSED_OFFSET_CTRL      ((uint16_t)0x01<<5) 
#define KEY_PRESSED_OFFSET_Q         ((uint16_t)0x01<<6) 
#define KEY_PRESSED_OFFSET_E         ((uint16_t)0x01<<7) 
#define KEY_PRESSED_OFFSET_R         ((uint16_t)0x01<<8) 
#define KEY_PRESSED_OFFSET_F         ((uint16_t)0x01<<9) 
#define KEY_PRESSED_OFFSET_G         ((uint16_t)0x01<<10) 
#define KEY_PRESSED_OFFSET_Z         ((uint16_t)0x01<<11) 
#define KEY_PRESSED_OFFSET_X         ((uint16_t)0x01<<12) 
#define KEY_PRESSED_OFFSET_C         ((uint16_t)0x01<<13) 
#define KEY_PRESSED_OFFSET_V         ((uint16_t)0x01<<14) 
#define KEY_PRESSED_OFFSET_B         ((uint16_t)0x01<<15) 

#define  RC_FRAME_LENGTH                            18u 
 
/* ----------------------- Data Struct ------------------------------------- */ 
typedef __packed struct 
{
     __packed struct
     {
         int ch[5];     //通道4为侧边滚轮 其余查遥控手册
         u8  s1;
         u8  s2;
     }rc; 
 
     __packed struct
     {
         int16_t x;
         int16_t y;
         int16_t z;
         u8 press_l;   //左键
         u8 press_r;   //右键
     }mouse; 
 
     __packed struct
     {
         u16 v;
     }key; 
}RC_Ctl_t; 

extern volatile  RC_Ctl_t RC_Ctl;

void usart6_Init(void);
void USART6_SendChar(u8 ch);
void USART6_TX_Byte(unsigned char data);
void USART1_IRQHandler(void);
void DMA2_Stream5_IRQHandler(void);
void USART1_Configuration(void);

#endif

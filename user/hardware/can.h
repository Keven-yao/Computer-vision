#ifndef __CAN_H
#define __CAN_H

#include "main.h"
uint8_t CAN1_mode_init(uint8_t tsjw, uint8_t tbs2, uint8_t tbs1, uint16_t brp, uint8_t mode);
uint8_t CAN2_mode_init(uint8_t tsjw, uint8_t tbs2, uint8_t tbs1, uint16_t brp, uint8_t mode);
u8 CAN1_Send_Msg(u16 stdId, u8* msg,u8 len);
u8 CAN2_Send_Msg(u16 stdId2, u8* msg2,u8 len2);

#endif

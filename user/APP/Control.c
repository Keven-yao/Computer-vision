#include <stdlib.h>
#include "stm32f4xx.h"
#include "sys.h"

#include "control.h"
#include "chassis.h"
#include "usart.h"
#include "stdio.h"

u8 rcTimer = 0; //遥控时间戳
u8 sbusBuf[18]; //遥控数据
u8 sbusNum = 0; //数据下标

u16 t_rc;
RC_Ctl_t rcCtrl;

void sbusToRc(void)
{
 //   sbusNum = 0;
    //在sbusNum != 18的情况下不进入解析
    if(*sbusBuf == NULL && sbusNum != 18)
    {
        return;
    }
    
    rcCtrl.rc.ch[0] = (sbusBuf[0] | (sbusBuf[1] << 8)) & 0x07ff;        //!< Channel 0
    rcCtrl.rc.ch[1] = ((sbusBuf[1] >> 3) | (sbusBuf[2] << 5)) & 0x07ff; //!< Channel 1
    rcCtrl.rc.ch[2] = ((sbusBuf[2] >> 6) | (sbusBuf[3] << 2) | (sbusBuf[4] << 10)) & 0x07ff;          //!< Channel 2
    rcCtrl.rc.ch[3] = ((sbusBuf[4] >> 1) | (sbusBuf[5] << 7)) & 0x07ff; //!< Channel 3
    rcCtrl.rc.ch[4] = sbusBuf[16] | (sbusBuf[17] << 8);                 //滚轮
    rcCtrl.rc.s1 = ((sbusBuf[5] >> 4) & 0x0003);                  //!< Switch right
    rcCtrl.rc.s2 = ((sbusBuf[5] >> 4) & 0x000C) >> 2;                       //!< Switch left
    rcCtrl.mouse.x = sbusBuf[6] | (sbusBuf[7] << 8);                    //!< Mouse X axis
    rcCtrl.mouse.y = sbusBuf[8] | (sbusBuf[9] << 8);                    //!< Mouse Y axis
    rcCtrl.mouse.z = sbusBuf[10] | (sbusBuf[11] << 8);                  //!< Mouse Z axis
    rcCtrl.mouse.press_l = sbusBuf[12];                                  //!< Mouse Left Is Pre
    rcCtrl.mouse.press_r = sbusBuf[13];                                  //!< Mouse Right Is Press ?
    rcCtrl.key.v = sbusBuf[14] | (sbusBuf[15] << 8);                    //!< KeyBoard value
    rcCtrl.rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rcCtrl.rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rcCtrl.rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rcCtrl.rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rcCtrl.rc.ch[4] -= RC_CH_VALUE_OFFSET;
		
//		if (t_rc>30)
//		{
//			printf("通道[2] : %d\r\n",rcCtrl.rc.ch[2]);
//			t_rc=0;
//		}
//		else t_rc++;
////    for(u8 i=0;i<4;i++){
//		     USART6_TX_Byte(rcCtrl.rc.ch[4]>>8);
//				 USART6_TX_Byte(rcCtrl.rc.ch[4]);
//		}
		

}

void control(void)
{
    //判断遥控是否在线
    if(rcTimer < 140)
    {
        //遥控器S1操作
        switch(rcCtrl.rc.s1)
        {
            case 1:
                break;
            case 2:
                break;
            case 3:
                break;
        }
        //遥控器S2操作
        switch(rcCtrl.rc.s2)
        {
            case 1:
                break;
            case 2:
                break;
            case 3:
                break;
        }
    }
    else
        rcTimer = 140;
    
}

/**************************************************************************************************
  Filename:       hal_timer.c
  Revised:        $Date: 2010-05-28 15:26:34 -0700 (Fri, 28 May 2010) $
  Revision:       $Revision: 22676 $

  Description:   This file contains the interface to the Timer Service.


  Copyright 2006-2010 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
 NOTE: Z-Stack and TIMAC no longer use CC2530 Timer 1, Timer 3, and 
       Timer 4. The supporting timer driver module is removed and left 
       for the users to implement their own application timer 
       functions.
*********************************************************************/
/*********************************************************************
 * INCLUDES
 */
#include  "hal_mcu.h"
#include  "hal_defs.h"
#include  "hal_types.h"
#include  "hal_timer.h"//-->"hal_board.h"-->"hal_board_cfg.h"-->应用宏定义 AC CA TH etc.

#include "hal_board.h"
#include "hal_drivers.h"
#include "osal.h"


#if defined (IR_GENERAL)  

#include "irr.h"
/**************************************************************************************************
 * @fn      halTimer3Isr
 *
 * @brief   Timer 3 ISR
 *
 * @param
 *
 * @return
 **************************************************************************************************/
HAL_ISR_FUNCTION( halTimer3Isr, T3_VECTOR )
{
//  halProcessTimer3 ();
  //HAL_ENTER_ISR();

	 //P1_1 = ~P1_1;
	 EdgeTimeSlot_HR[edgeCnt] = T4CNT;
	 EdgeTimeSlot_LRsl = us10Cnt;
	 //TIMIF &= 0xFB; //清除T3通道1（捕捉模式）的中断标志
	 us10Cnt = 0;
	 edgeCnt++;

  
//HAL_EXIT_ISR();

}

/**************************************************************************************************
 * @fn      halTimer4Isr
 *
 * @brief   Timer 4 ISR
 *
 * @param
 *
 * @return
 **************************************************************************************************/
HAL_ISR_FUNCTION( halTimer4Isr, T4_VECTOR )
{
//  halProcessTimer4 ();

	if(irCodeRxFlag)
	{
		//learn mode,
    	us10Cnt++;
	}else
	{
		//generation mode,
		if(2 == us10Cnt)
		{
			T4CC0 = EdgeTimeSlot_HR[edgeCnt];
			//T4CC0V1 = T4CC0;
			//T4CC0V1 = T4CC0;
		}
		if(1 == us10Cnt)
		{
			T4CC0 = 79;
			//T4CC0V2 = T4CC0;
			//T4CC0V2 = T4CC0;
		}
		if(0 == us10Cnt)
		{
                  //T4CC0V3 = T4CC0;
                  //T4CC0V3 = T4CC0;
                  singleEdgeEndFlag = 1;//单个边沿计时结束
                }
		else
		{
			us10Cnt--;
		}
	}

}
#endif
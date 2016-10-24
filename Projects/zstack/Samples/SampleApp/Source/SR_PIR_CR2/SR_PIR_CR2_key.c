/**************************************************************************************************
  Filename:       hal_key.c
  Revised:        $Date: 2010-09-15 19:02:45 -0700 (Wed, 15 Sep 2010) $
  Revision:       $Revision: 23815 $

  Description:    This file contains the interface to the HAL KEY Service.


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
 NOTE: If polling is used, the hal_driver task schedules the KeyRead()
       to occur every 100ms.  This should be long enough to naturally
       debounce the keys.  The KeyRead() function remembers the key
       state of the previous poll and will only return a non-zero
       value if the key state changes.

 NOTE: If interrupts are used, the KeyRead() function is scheduled
       25ms after the interrupt occurs by the ISR.  This delay is used
       for key debouncing.  The ISR disables any further Key interrupt
       until KeyRead() is executed.  KeyRead() will re-enable Key
       interrupts after executing.  Unlike polling, when interrupts
       are enabled, the previous key state is not remembered.  This
       means that KeyRead() will return the current state of the keys
       (not a change in state of the keys).

 NOTE: If interrupts are used, the KeyRead() fucntion is scheduled by
       the ISR.  Therefore, the joystick movements will only be detected
       during a pushbutton interrupt caused by S1 or the center joystick
       pushbutton.

 NOTE: When a switch like S1 is pushed, the S1 signal goes from a normally
       high state to a low state.  This transition is typically clean.  The
       duration of the low state is around 200ms.  When the signal returns
       to the high state, there is a high likelihood of signal bounce, which
       causes a unwanted interrupts.  Normally, we would set the interrupt
       edge to falling edge to generate an interrupt when S1 is pushed, but
       because of the signal bounce, it is better to set the edge to rising
       edge to generate an interrupt when S1 is released.  The debounce logic
       can then filter out the signal bounce.  The result is that we typically
       get only 1 interrupt per button push.  This mechanism is not totally
       foolproof because occasionally, signal bound occurs during the falling
       edge as well.  A similar mechanism is used to handle the joystick
       pushbutton on the DB.  For the EB, we do not have independent control
       of the interrupt edge for the S1 and center joystick pushbutton.  As
       a result, only one or the other pushbuttons work reasonably well with
       interrupts.  The default is the make the S1 switch on the EB work more
       reliably.

*********************************************************************/

/**************************************************************************************************
 *                                            INCLUDES
 **************************************************************************************************/
#include "hal_mcu.h"
#include "hal_defs.h"
#include "hal_types.h"
#include "hal_board.h"
#include "hal_drivers.h"
#include "hal_adc.h"
#include "hal_key.h"
#include "osal.h"
   #include "IO_config.h"
#if (defined HAL_KEY) && (HAL_KEY == TRUE)

/**************************************************************************************************
 *                                              MACROS
 **************************************************************************************************/

/**************************************************************************************************
 *                                            CONSTANTS
 **************************************************************************************************/


/**************************************************************************************************
 *                                            TYPEDEFS
 **************************************************************************************************/


/**************************************************************************************************
 *                                        GLOBAL VARIABLES
 **************************************************************************************************/
static uint8 halKeySavedKeys;     /* used to store previous key state in polling mode */
 halKeyCBack_t pHalKeyProcessFunction;
static uint8 HalKeyConfigured;
bool Hal_KeyIntEnable;            /* interrupt enable/disable flag */

/**************************************************************************************************
 *                                        FUNCTIONS - Local
 **************************************************************************************************/
void halProcessKeyInterrupt(void);
uint8 halGetJoyKeyInput(void);
uint8 kpush0=0;
uint8 kpush1=0;
uint8 key_ISR_FLAG=0;
uint8 KEY_NET_ISR_FLAG=0;//test p1.2口的单击 双击
uint8 key_double_flag = 0;//按键双击状态

uint8 key_holdtime_waiting_flag = 0;//等待长按时间（10s）标志 ，0无等待 1正在等待

uint8 key_hold_flag = 0;  //按键长按标志
extern uint8 FRONT_ALARM_FLAG;

/**************************************************************************************************
 *                                        FUNCTIONS - API
 **************************************************************************************************/


/**************************************************************************************************
 * @fn      HalKeyInit
 *
 * @brief   Initilize Key Service
 *
 * @param   none
 *
 * @return  None
 **************************************************************************************************/
void HalKeyInit( void )
{
  /* Initialize previous key to 0 */
  halKeySavedKeys = 0;

  
  HAL_KEY_SW_6_SEL &= ~(HAL_KEY_SW_6_BIT);    /* Set pin function to GPIO */
  HAL_KEY_SW_6_DIR &= ~(HAL_KEY_SW_6_BIT);    /* Set pin direction to Input */
  
  HAL_KEY_SW_8_SEL &= ~(HAL_KEY_SW_8_BIT);    /* Set pin function to GPIO */
  HAL_KEY_SW_8_DIR &= ~(HAL_KEY_SW_8_BIT);    /* Set pin direction to Input */
  
  HAL_KEY_SW_9_SEL &= ~(HAL_KEY_SW_9_BIT);    /* Set pin function to GPIO */
  HAL_KEY_SW_9_DIR &= ~(HAL_KEY_SW_9_BIT);    /* Set pin direction to Input */
  
  /* Initialize callback function */
  pHalKeyProcessFunction  = NULL;

  /* Start with key is not configured */
  HalKeyConfigured = FALSE;
}


/**************************************************************************************************
 * @fn      HalKeyConfig
 *
 * @brief   Configure the Key serivce
 *
 * @param   interruptEnable - TRUE/FALSE, enable/disable interrupt
 *          cback - pointer to the CallBack function
 *
 * @return  None
 **************************************************************************************************/
void HalKeyConfig (bool interruptEnable, halKeyCBack_t cback)
{
  /* Enable/Disable Interrupt or */
  Hal_KeyIntEnable = interruptEnable;

  /* Register the callback fucntion */
  pHalKeyProcessFunction = cback;

  /* Determine if interrupt is enable or not */
  if (Hal_KeyIntEnable)
  {
      /* Rising/Falling edge configuratinn */

   
    
    //P0.4  下降沿触发
    PICTL &= ~(HAL_KEY_SW_6_EDGEBIT);    /* Clear the edge bit */
    /* For falling edge, the bit must be set. */
  #if (HAL_KEY_SW_6_EDGE == HAL_KEY_FALLING_EDGE)//2530的上升 下降沿触发是按端口设置的，即P0要么是上升要么全是下降
    PICTL |= HAL_KEY_SW_6_EDGEBIT;
  #endif


    /* Interrupt configuration:
     * - Enable interrupt generation at the port
     * - Enable CPU interrupt
     * - Clear any pending interrupt
     */
    HAL_KEY_SW_6_ICTL |= HAL_KEY_SW_6_ICTLBIT;//P0IEN | 00010000  即P0.4使能中断
    HAL_KEY_SW_6_IEN |= HAL_KEY_SW_6_IENBIT;//使能整个CPU的P0口中断
    HAL_KEY_SW_6_PXIFG = ~(HAL_KEY_SW_6_BIT);//清中断标记位
  
    
    
   //p1.2 下升沿触发 
     PICTL &= ~(HAL_KEY_SW_8_EDGEBIT);    /* Clear the edge bit */
    /* For falling edge, the bit must be set. */
  #if (HAL_KEY_SW_8_EDGE == HAL_KEY_FALLING_EDGE)
    PICTL |= HAL_KEY_SW_8_EDGEBIT;			//P1.0~p1.3下降沿 。p1.2 按键下降沿
  #endif


    /* Interrupt configuration:
     * - Enable interrupt generation at the port
     * - Enable CPU interrupt
     * - Clear any pending interrupt
     */
    HAL_KEY_SW_8_ICTL |= HAL_KEY_SW_8_ICTLBIT;//P1IEN | 00000100  即P1.2使能中断
    HAL_KEY_SW_8_IEN |= HAL_KEY_SW_8_IENBIT;//使能整个CPU的P1口中断
    HAL_KEY_SW_8_PXIFG = ~(HAL_KEY_SW_8_BIT);//清中断标记位
   
    

//P1.5 上升沿触发
     PICTL &= ~(HAL_KEY_SW_9_EDGEBIT);    /* Clear the edge bit */
    /* For falling edge, the bit must be set. */
  #if (HAL_KEY_SW_9_EDGE == HAL_KEY_FALLING_EDGE)//2530的上升 下降沿触发是按端口设置的，即P0要么是上升要么全是下降
    PICTL |= HAL_KEY_SW_9_EDGEBIT;
  #endif

    HAL_KEY_SW_9_ICTL |= HAL_KEY_SW_9_ICTLBIT;//P1IEN | 00100000  即P1.5使能中断
    HAL_KEY_SW_9_IEN |= HAL_KEY_SW_9_IENBIT;//使能整个CPU的P1口中断
    HAL_KEY_SW_9_PXIFG = ~(HAL_KEY_SW_9_BIT);//清中断标记位
    
    



    /* Do this only after the hal_key is configured - to work with sleep stuff */
    if (HalKeyConfigured == TRUE)
    {
      osal_stop_timerEx(Hal_TaskID, HAL_KEY_EVENT);  /* Cancel polling if active */
    }
  }
  else    /* Interrupts NOT enabled */
  {
    HAL_KEY_SW_6_ICTL &= ~(HAL_KEY_SW_6_ICTLBIT); /* don't generate interrupt */
    HAL_KEY_SW_6_IEN &= ~(HAL_KEY_SW_6_IENBIT);   /* Clear interrupt enable bit */

    osal_set_event(Hal_TaskID, HAL_KEY_EVENT);
  }

  /* Key now is configured */
  HalKeyConfigured = TRUE;
}


/**************************************************************************************************
 * @fn      HalKeyRead
 *
 * @brief   Read the current value of a key
 *
 * @param   None
 *
 * @return  keys - current keys status
 **************************************************************************************************/
uint8 HalKeyRead ( void )
{
  uint8 keys = 0;

  return keys;
}


/**************************************************************************************************
 * @fn      HalKeyPoll
 *
 * @brief   Called by hal_driver to poll the keys
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalKeyPoll (void)
{
 uint8 keys = 0;

 if ((HAL_KEY_JOY_MOVE_PORT & HAL_KEY_JOY_MOVE_BIT))  /* Key is active HIGH */
  {
 
  }
  

  /* If interrupts are not enabled, previous key status and current key status
   * are compared to find out if a key has changed status.
   */
  if (!Hal_KeyIntEnable)
  {
    if (keys == halKeySavedKeys)
    {
       /* Exit - since no keys have changed */
      return;
    }
    /* Store the current keys for comparation next time */
    halKeySavedKeys = keys;
  }
  else
  {
    /* Key interrupt handled here */
       
   if ((key_ISR_FLAG==HAL_KEY_SW_9) && (HAL_PUSH_BUTTON5()))//初步理解，设置p1.5为上升沿触发，触发唤醒系统后，25ms到这，判断是否为1，是1则读取见键值
   {

    FRONT_ALARM_FLAG=1;// 确实是报警的
    keys |= HAL_KEY_SW_9; 
   }
  
   else if ((key_ISR_FLAG==HAL_KEY_SW_6) && (!HAL_PUSH_BUTTON4()))//初步理解，设置p0.4为下降沿触发，触发唤醒系统后，25ms到这，判断是否为0，是0则读取见键值
   {
     FRONT_ALARM_FLAG=0;
    keys |= HAL_KEY_SW_6;
   }
   
    else if ((key_ISR_FLAG==HAL_KEY_SW_8) && (!(HAL_PUSH_BUTTON3())))//初步理解，设置p1.2为上升沿触发，触发唤醒系统后，25ms到这，判断是否为1，是1则读取见键值
   {
     if(key_holdtime_waiting_flag)
	 {
	   key_holdtime_waiting_flag = 0;//在等待长按时间时再次按下按钮 此时需要清等待长按时间标志以及取消之前设置的等待长按时间满以后进入事件
	   osal_stop_timerEx (Hal_TaskID, HAL_KEY_HOLD_EVENT);//  停止之前进入的事件
	   KEY_NET_ISR_FLAG = 0;// 等待长按时间被打断后 重新计数按键次数
	 }
	 KEY_NET_ISR_FLAG++;//test 测试单击 双击等程序
     osal_stop_timerEx (Hal_TaskID, HAL_KEY_MULTI_CLICK_EVENT);//  停止之前进入的事件
     osal_start_timerEx (Hal_TaskID, HAL_KEY_MULTI_CLICK_EVENT, HAL_KEY_CLICKED_VALUE);//600mS后触发检测几击程序
     
   }
    
  }

  
 
  /* Invoke Callback if new keys were depressed */
  if (keys && (pHalKeyProcessFunction))
  {
    (pHalKeyProcessFunction) (keys, HAL_KEY_STATE_NORMAL);
    key_ISR_FLAG=0;
  }
}




/**************************************************************************************************
 * @fn      halGetJoyKeyInput
 *
 * @brief   Map the ADC value to its corresponding key.
 *
 * @param   None
 *
 * @return  keys - current joy key status
 **************************************************************************************************/
uint8 halGetJoyKeyInput(void)
{
  /* The joystick control is encoded as an analog voltage.
   * Read the JOY_LEVEL analog value and map it to joy movement.
   */
  uint8 adc;
  uint8 ksave0 = 0;
  uint8 ksave1;

  /* Keep on reading the ADC until two consecutive key decisions are the same. */
  do
  {
    ksave1 = ksave0;    /* save previouse key reading */

    adc = HalAdcRead (HAL_KEY_JOY_CHN, HAL_ADC_RESOLUTION_8);

    if ((adc >= 2) && (adc <= 38))
    {
    //   ksave0 |= HAL_KEY_UP;
    }
    else if ((adc >= 74) && (adc <= 88))
    {
   //   ksave0 |= HAL_KEY_RIGHT;
    }
    else if ((adc >= 60) && (adc <= 73))
    {
   //   ksave0 |= HAL_KEY_LEFT;
    }
    else if ((adc >= 39) && (adc <= 59))
    {
   //   ksave0 |= HAL_KEY_DOWN;
    }
    else if ((adc >= 89) && (adc <= 100))
    {
   //   ksave0 |= HAL_KEY_CENTER;
    }
  } while (ksave0 != ksave1);

  return ksave0;
}





/**************************************************************************************************
 * @fn      halProcessKeyInterrupt
 *
 * @brief   Checks to see if it's a valid key interrupt, saves interrupt driven key states for
 *          processing by HalKeyRead(), and debounces keys by scheduling HalKeyRead() 25ms later.
 *
 * @param
 *
 * @return
 **************************************************************************************************/
void halProcessKeyInterrupt (void)
{
  bool valid=FALSE;
  
  
  
     if (HAL_KEY_SW_6_PXIFG & HAL_KEY_SW_6_BIT)  /* Interrupt Flag has been set */
  {
    HAL_KEY_SW_6_PXIFG = ~(HAL_KEY_SW_6_BIT); /* Clear Interrupt Flag */
    valid = TRUE;
  }

  
    if (HAL_KEY_SW_8_PXIFG & HAL_KEY_SW_8_BIT)  /* Interrupt Flag has been set */
  {
    HAL_KEY_SW_8_PXIFG = ~(HAL_KEY_SW_8_BIT); /* Clear Interrupt Flag */
    valid = TRUE;
  }
  
    if (HAL_KEY_SW_9_PXIFG & HAL_KEY_SW_9_BIT)  /* Interrupt Flag has been set */
  {
    HAL_KEY_SW_9_PXIFG = ~(HAL_KEY_SW_9_BIT); /* Clear Interrupt Flag */
    valid = TRUE;
  }
  
  
  if (valid)
  {
    osal_start_timerEx (Hal_TaskID, HAL_KEY_EVENT, HAL_KEY_DEBOUNCE_VALUE);
  }
}

/**************************************************************************************************
 * @fn      HalKeyEnterSleep
 *
 * @brief  - Get called to enter sleep mode
 *
 * @param
 *
 * @return
 **************************************************************************************************/
void HalKeyEnterSleep ( void )
{
}

/**************************************************************************************************
 * @fn      HalKeyExitSleep
 *
 * @brief   - Get called when sleep is over
 *
 * @param
 *
 * @return  - return saved keys
 **************************************************************************************************/
uint8 HalKeyExitSleep ( void )
{
  /* Wake up and read keys */
  return ( HalKeyRead () );
}

/***************************************************************************************************
 *                                    INTERRUPT SERVICE ROUTINE
 ***************************************************************************************************/

/**************************************************************************************************
 * @fn      halKeyPort0Isr
 *
 * @brief   Port0 ISR
 *
 * @param
 *
 * @return
 **************************************************************************************************/

//消除警告
HAL_ISR_FUNCTION( halKeyPort0Isr, P0INT_VECTOR )
{
  uint8 P0IEN_FLAG_1=0;
  HAL_ENTER_ISR();
 if (HAL_KEY_SW_6_PXIFG & HAL_KEY_SW_6_BIT)
  {
    P0IEN_FLAG_1 = P0IEN & BV(4);//P0.4
    if(P0IEN_FLAG_1 == 0X10)
    {  
      halProcessKeyInterrupt();
      HAL_KEY_SW_6_PXIFG = 0;
      key_ISR_FLAG =HAL_KEY_SW_6;
    }
  }
   HAL_KEY_CPU_PORT_0_IF = 0;
   CLEAR_SLEEP_MODE();
  HAL_EXIT_ISR();
}


/**************************************************************************************************
 * @fn      halKeyPort1Isr
 *
 * @brief   Port1 ISR
 *
 * @param
 *
 * @return
 **************************************************************************************************/
HAL_ISR_FUNCTION( halKeyPort1Isr, P1INT_VECTOR )
{
uint8 P1IEN_FLAG_1=0;
  
 
       
  HAL_ENTER_ISR();
   if(HAL_KEY_SW_8_PXIFG & HAL_KEY_SW_8_BIT)
  {
     halProcessKeyInterrupt();
     HAL_KEY_SW_8_PXIFG &= ~(HAL_KEY_SW_8_BIT);
     key_ISR_FLAG =HAL_KEY_SW_8;
     
  }
 else if (HAL_KEY_SW_9_PXIFG & HAL_KEY_SW_9_BIT)
  {
    P1IEN_FLAG_1 = P1IEN & BV(5);//p1.3
    if(P1IEN_FLAG_1==0X20)
    {
      halProcessKeyInterrupt();
      HAL_KEY_SW_9_PXIFG = 0;
      key_ISR_FLAG =HAL_KEY_SW_9;
    }
  }
  HAL_KEY_CPU_PORT_1_IF = 0;
  
  CLEAR_SLEEP_MODE();
  HAL_EXIT_ISR();
  
}




/**************************************************************************************************
 * @fn      halKeyPort2Isr
 *
 * @brief   Port2 ISR
 *
 * @param
 *
 * @return
 **************************************************************************************************/
HAL_ISR_FUNCTION( halKeyPort2Isr, P2INT_VECTOR )
{
  HAL_ENTER_ISR();
  
  if (HAL_KEY_JOY_MOVE_PXIFG & HAL_KEY_JOY_MOVE_BIT)
  {
    halProcessKeyInterrupt();
  }


  HAL_KEY_JOY_MOVE_PXIFG = 0;
  HAL_KEY_CPU_PORT_2_IF = 0;

  CLEAR_SLEEP_MODE();
  HAL_EXIT_ISR();
}






#else

void Key_Check(void){}
void HalKeyInit(void){}
void HalKeyConfig(bool interruptEnable, halKeyCBack_t cback){}
uint8 HalKeyRead(void){ return 0;}
void HalKeyPoll(void){}
void HalKey_double_detect(void);
#endif /* HAL_KEY */





/**************************************************************************************************
**************************************************************************************************/




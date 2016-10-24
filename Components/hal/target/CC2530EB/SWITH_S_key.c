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
#include "my_time.h"
#include "device.h"


#if (defined HAL_KEY) && (HAL_KEY == TRUE)

/**************************************************************************************************
 *                                              MACROS
 **************************************************************************************************/

/**************************************************************************************************
 *                                            CONSTANTS
 **************************************************************************************************/
#define HAL_KEY_RISING_EDGE   0
#define HAL_KEY_FALLING_EDGE  1

#define HAL_KEY_DEBOUNCE_VALUE  25
#define HAL_KEY_CLICKED_VALUE  600  //600ms 内没按键中断则按键检测结束，发送检测到的按键次数到应用层
#define HAL_KEY_HOLD_VALUE  9400    //9.4s  按键长按10s（9.4s+600ms）以后判断为长按按钮操作
#define HAL_KEY_DOBULETICK_VALUE   1000


/* CPU port interrupt */
#define HAL_KEY_CPU_PORT_0_IF P0IF
#define HAL_KEY_CPU_PORT_1_IF P1IF
#define HAL_KEY_CPU_PORT_2_IF P2IF

/* SW_5 is at P0.5  专用红外放拆*/
#define HAL_KEY_SW_5_PORT   P0
#define HAL_KEY_SW_5_BIT    BV(5)
#define HAL_KEY_SW_5_SEL    P0SEL
#define HAL_KEY_SW_5_DIR    P0DIR

/* edge interrupt */
#define HAL_KEY_SW_5_EDGEBIT  BV(0)
#define HAL_KEY_SW_5_EDGE     HAL_KEY_FALLING_EDGE


/* SW_6 interrupts */
#define HAL_KEY_SW_5_IEN      IEN1  /* CPU interrupt mask register */
#define HAL_KEY_SW_5_IENBIT   BV(5) /* Mask bit for all of Port_0 */
#define HAL_KEY_SW_5_ICTL     P0IEN /* Port Interrupt Control register */
#define HAL_KEY_SW_5_ICTLBIT  BV(5) /* P0IEN - P0.1 enable/disable bit BV(1)*/
#define HAL_KEY_SW_5_PXIFG    P0IFG /* Interrupt flag at source */



/* SW_6 is at P0.4 */
#define HAL_KEY_SW_6_PORT   P0
#define HAL_KEY_SW_6_BIT    BV(4)//BV(1)
#define HAL_KEY_SW_6_SEL    P0SEL
#define HAL_KEY_SW_6_DIR    P0DIR

/* edge interrupt */
#define HAL_KEY_SW_6_EDGEBIT  BV(0)
#define HAL_KEY_SW_6_EDGE     HAL_KEY_FALLING_EDGE


/* SW_6 interrupts */
#define HAL_KEY_SW_6_IEN      IEN1  /* CPU interrupt mask register */
#define HAL_KEY_SW_6_IENBIT   BV(5) /* Mask bit for all of Port_0 */
#define HAL_KEY_SW_6_ICTL     P0IEN /* Port Interrupt Control register */
#define HAL_KEY_SW_6_ICTLBIT  BV(4) /* P0IEN - P0.1 enable/disable bit BV(1)*/
#define HAL_KEY_SW_6_PXIFG    P0IFG /* Interrupt flag at source */

/* SW_8 is at P1.2 */
#define HAL_KEY_SW_8_PORT   P1
#define HAL_KEY_SW_8_BIT    BV(2)
#define HAL_KEY_SW_8_SEL    P1SEL
#define HAL_KEY_SW_8_DIR    P1DIR
/* edge interrupt */
#define HAL_KEY_SW_8_EDGEBIT  BV(1)//p1端口的触发模式设置 在PICTL的低第二位（位1）
#define HAL_KEY_SW_8_EDGE     HAL_KEY_FALLING_EDGE  // 上升沿触发


/* SW_8 interrupts */
#define HAL_KEY_SW_8_IEN      IEN2  /* CPU interrupt mask register */
#define HAL_KEY_SW_8_IENBIT   BV(4) /* Mask bit for all of Port_1 */
#define HAL_KEY_SW_8_ICTL     P1IEN /* Port Interrupt Control register */
#define HAL_KEY_SW_8_ICTLBIT  BV(2) /* P1IEN - P1.2 enable/disable bit */ 
#define HAL_KEY_SW_8_PXIFG    P1IFG /* Interrupt flag at source */


/* SW_7 is at P1.3 */
#define HAL_KEY_SW_7_PORT   P1
#define HAL_KEY_SW_7_BIT    BV(3)
#define HAL_KEY_SW_7_SEL    P1SEL
#define HAL_KEY_SW_7_DIR    P1DIR

/* edge interrupt */
#define HAL_KEY_SW_7_EDGEBIT  BV(1)//p1端口的触发模式设置 在PICTL的低第二位（位1）
#define HAL_KEY_SW_7_EDGE     HAL_KEY_FALLING_EDGE  // 下升沿触发


/* SW_8 interrupts */
#define HAL_KEY_SW_7_IEN      IEN2  /* CPU interrupt mask register */
#define HAL_KEY_SW_7_IENBIT   BV(4) /* Mask bit for all of Port_1 */
#define HAL_KEY_SW_7_ICTL     P1IEN /* Port Interrupt Control register */
#define HAL_KEY_SW_7_ICTLBIT  BV(3) /* P1IEN - P1.3 enable/disable bit */ 
#define HAL_KEY_SW_7_PXIFG    P1IFG /* Interrupt flag at source */

/* SW_9 is at P1.5 */
#define HAL_KEY_SW_9_PORT   P1
#define HAL_KEY_SW_9_BIT    BV(5)
#define HAL_KEY_SW_9_SEL    P1SEL
#define HAL_KEY_SW_9_DIR    P1DIR

/* edge interrupt */
#define HAL_KEY_SW_9_EDGEBIT  BV(2)//p1端口的触发模式设置 在PICTL的低第三位（位2）P1.5为高4位地址
//#define HAL_KEY_SW_8_EDGE     HAL_KEY_FALLING_EDGE  // 上升沿触发


/* SW_8 interrupts */
#define HAL_KEY_SW_9_IEN      IEN2  /* CPU interrupt mask register */
#define HAL_KEY_SW_9_IENBIT   BV(4) /* Mask bit for all of Port_1 */
#define HAL_KEY_SW_9_ICTL     P1IEN /* Port Interrupt Control register */
#define HAL_KEY_SW_9_ICTLBIT  BV(5) /* P1IEN - P1.5 enable/disable bit */ 
#define HAL_KEY_SW_9_PXIFG    P1IFG /* Interrupt flag at source */

/* Joy stick move at P2.0 */
#define HAL_KEY_JOY_MOVE_PORT   P2
#define HAL_KEY_JOY_MOVE_BIT    BV(0)
#define HAL_KEY_JOY_MOVE_SEL    P2SEL
#define HAL_KEY_JOY_MOVE_DIR    P2DIR

/* edge interrupt */
#define HAL_KEY_JOY_MOVE_EDGEBIT  BV(3)
#define HAL_KEY_JOY_MOVE_EDGE     HAL_KEY_FALLING_EDGE

/* Joy move interrupts */
#define HAL_KEY_JOY_MOVE_IEN      IEN2  /* CPU interrupt mask register */
#define HAL_KEY_JOY_MOVE_IENBIT   BV(1) /* Mask bit for all of Port_2 */
#define HAL_KEY_JOY_MOVE_ICTL     P2IEN /* Port Interrupt Control register */
#define HAL_KEY_JOY_MOVE_ICTLBIT  BV(0) /* P2IENL - P2.0<->P2.3 enable/disable bit */
#define HAL_KEY_JOY_MOVE_PXIFG    P2IFG /* Interrupt flag at source */

#define HAL_KEY_JOY_CHN   HAL_ADC_CHANNEL_6




/**************************************************************************************************
 *                                            TYPEDEFS
 **************************************************************************************************/


/**************************************************************************************************
 *                                        GLOBAL VARIABLES
 **************************************************************************************************/
static uint8 halKeySavedKeys;     /* used to store previous key state in polling mode */
static halKeyCBack_t pHalKeyProcessFunction;
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
uint8 key_p1_2_ISR_FLAG=0;//test p1.2口的单击 双击
uint8 key_double_flag = 0;//按键双击状态

uint8 key_holdtime_waiting_flag = 0;//等待长按时间（10s）标志 ，0无等待 1正在等待

uint8 key_hold_flag = 0;  //按键长按标志

uint8 key_TGdouble_flag=0;//专门用于调光的按键双击标记，

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

  HAL_KEY_SW_5_SEL &= ~(HAL_KEY_SW_5_BIT);    /* Set pin function to GPIO */
  HAL_KEY_SW_5_DIR &= ~(HAL_KEY_SW_5_BIT);    /* Set pin direction to Input */
  
  HAL_KEY_SW_6_SEL &= ~(HAL_KEY_SW_6_BIT);    /* Set pin function to GPIO */
  HAL_KEY_SW_6_DIR &= ~(HAL_KEY_SW_6_BIT);    /* Set pin direction to Input */
  
  HAL_KEY_SW_7_SEL &= ~(HAL_KEY_SW_7_BIT);    /* Set pin function to GPIO */
  HAL_KEY_SW_7_DIR &= ~(HAL_KEY_SW_7_BIT);    /* Set pin direction to Input */
  
  HAL_KEY_SW_8_SEL &= ~(HAL_KEY_SW_8_BIT);    /* Set pin function to GPIO */
  HAL_KEY_SW_8_DIR &= ~(HAL_KEY_SW_8_BIT);    /* Set pin direction to Input */
  

  HAL_KEY_JOY_MOVE_SEL &= ~(HAL_KEY_JOY_MOVE_BIT); /* Set pin function to GPIO */
  HAL_KEY_JOY_MOVE_DIR &= ~(HAL_KEY_JOY_MOVE_BIT); /* Set pin direction to Input */


  /* Initialize callback function */
  pHalKeyProcessFunction  = NULL;

  /* Start with key is not configured */
  HalKeyConfigured = FALSE;
  
  Start_T4();
  Start_T3();
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
    
    //p1.3 下升沿触发 
   PICTL &= ~(HAL_KEY_SW_7_EDGEBIT);    /* Clear the edge bit */
    /* For falling edge, the bit must be set. */
  #if (HAL_KEY_SW_7_EDGE == HAL_KEY_FALLING_EDGE)//2530的上升 下降沿触发是按端口设置的，即P0要么是上升要么全是下降
    PICTL |= HAL_KEY_SW_7_EDGEBIT;
  #endif

    HAL_KEY_SW_7_ICTL |= HAL_KEY_SW_7_ICTLBIT;//P1IEN | 00001000  即P1.3使能中断
    HAL_KEY_SW_7_IEN |= HAL_KEY_SW_7_IENBIT;//使能整个CPU的P1口中断
    HAL_KEY_SW_7_PXIFG = ~(HAL_KEY_SW_7_BIT);//清中断标记位
       
    
    
    /* Rising/Falling edge configuratinn */

    HAL_KEY_JOY_MOVE_ICTL &= ~(HAL_KEY_JOY_MOVE_EDGEBIT);    /* Clear the edge bit */
    /* For falling edge, the bit must be set. */
  #if (HAL_KEY_JOY_MOVE_EDGE == HAL_KEY_FALLING_EDGE)
    HAL_KEY_JOY_MOVE_ICTL |= HAL_KEY_JOY_MOVE_EDGEBIT;
  #endif


    /* Interrupt configuration:
     * - Enable interrupt generation at the port
     * - Enable CPU interrupt
     * - Clear any pending interrupt
     */
   // HAL_KEY_JOY_MOVE_ICTL |= HAL_KEY_JOY_MOVE_ICTLBIT;
    HAL_KEY_JOY_MOVE_IEN |= HAL_KEY_JOY_MOVE_IENBIT;
    HAL_KEY_JOY_MOVE_PXIFG = ~(HAL_KEY_JOY_MOVE_BIT);


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

  if (HAL_PUSH_BUTTON1())
  {
    keys |= HAL_KEY_SW_6;
  }

  if ((HAL_KEY_JOY_MOVE_PORT & HAL_KEY_JOY_MOVE_BIT))  /* Key is active low */
  {
    keys |= halGetJoyKeyInput();
  }

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
 
 // #if defined( PIR )    //用老版本的板子
 //   keys = halGetJoyKeyInput();
 //  #endif
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
   if ((key_ISR_FLAG==HAL_KEY_SW_7) && (!HAL_PUSH_BUTTON6()))//
   {
       //下面为按键P1.3的处理，当按键P1.3按下后表示减少亮度到不亮，启动定时器3.将SET_TIME设置为最大
    SET_TIME=160;//1600;//目的亮度不亮
    //Start_T3();//开启定时器3
    if(SET_TIME != DY_TIME)
     TIMER34_START(3,1);
    //key_set_flag=1;
     
   }
  
   else if ((key_ISR_FLAG==HAL_KEY_SW_8) && (!(HAL_PUSH_BUTTON3())))//初步理解，设置p1.2为上升沿触发，触发唤醒系统后，25ms到这，判断是否为1，是1则读取见键值
   {
   //  keys |= HAL_KEY_SW_8;
     if(key_holdtime_waiting_flag)
	 {
	   key_holdtime_waiting_flag = 0;//在等待长按时间时再次按下按钮 此时需要清等待长按时间标志以及取消之前设置的等待长按时间满以后进入事件
	   osal_stop_timerEx (Hal_TaskID, HAL_KEY_HOLD_EVENT);//  停止之前进入的事件
	   key_p1_2_ISR_FLAG = 0;// 等待长按时间被打断后 重新计数按键次数
	 }
	 key_p1_2_ISR_FLAG++;//test 测试单击 双击等程序
     osal_stop_timerEx (Hal_TaskID, HAL_KEY_MULTI_CLICK_EVENT);//  停止之前进入的事件
     osal_start_timerEx (Hal_TaskID, HAL_KEY_MULTI_CLICK_EVENT, HAL_KEY_CLICKED_VALUE);//600mS后触发检测几击程序
     
/* 20110617 注释掉	 
	 if(key_p1_2_ISR_FLAG==1)
     {
       osal_start_timerEx (Hal_TaskID, HAL_KEY_P1_2_EVENT, HAL_KEY_DOBULETICK_VALUE);//1S后触发检测几击程序
     }
*/	 
    //下面为按键P1.2的处理，当按键P1.2按下后表示增加亮度到最亮，启动定时器3.将SET_TIME设置为0
    //当亮度还没到达最亮的时候，这时候按键双击标记key_TGdouble_flag设置为1，如果已经达到最亮则清为0，
    //如果还没到达最亮，又来一次P1.2的按键。这时标记还为1，这时就停止定时器3，让亮度停在这个瞬间
     
    SET_TIME=60;//600;
    //Start_T3();//开启定时器3，开始慢慢增加亮度
    if(SET_TIME != DY_TIME)
    TIMER34_START(3,1);
    //key_set_flag=1;  

  }
    
  }

  
  /*
  if (HAL_PUSH_BUTTON1())
  {
    keys |= HAL_KEY_SW_6;
  }
  */

 
  /* Invoke Callback if new keys were depressed */
  if (keys && (pHalKeyProcessFunction))
  {
    (pHalKeyProcessFunction) (keys, HAL_KEY_STATE_NORMAL);
    key_ISR_FLAG=0;
  }
}

/**************************************************************************************************
 * @fn      HalKey_double_detect
 *
 * @brief   Called by hal_driver to detect double tick    test
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalKey_double_detect()
 {
     uint8 keys = 0;
 
  //#if (defined HAL_KEY) && (HAL_KEY == TRUE)

    keys |= HAL_KEY_SW_8;
    key_double_flag=key_p1_2_ISR_FLAG;//将几次按键值赋给状态
    
   if (keys && (pHalKeyProcessFunction))
   {
    (pHalKeyProcessFunction) (keys, HAL_KEY_STATE_NORMAL);
    // key_ISR_FLAG=0;
    key_p1_2_ISR_FLAG=0;
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
   //    ksave0 |= HAL_KEY_UP;
    }
    else if ((adc >= 74) && (adc <= 88))
    {
  //    ksave0 |= HAL_KEY_RIGHT;
    }
    else if ((adc >= 60) && (adc <= 73))
    {
  //    ksave0 |= HAL_KEY_LEFT;
    }
    else if ((adc >= 39) && (adc <= 59))
    {
 //     ksave0 |= HAL_KEY_DOWN;
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
  
  if (HAL_KEY_SW_7_PXIFG & HAL_KEY_SW_7_BIT)  /* Interrupt Flag has been set */
  {
    HAL_KEY_SW_7_PXIFG &= ~(HAL_KEY_SW_7_BIT); /* Clear Interrupt Flag */
    valid = TRUE;
  }

  
    if (HAL_KEY_SW_8_PXIFG & HAL_KEY_SW_8_BIT)  /* Interrupt Flag has been set */
  {
    HAL_KEY_SW_8_PXIFG = ~(HAL_KEY_SW_8_BIT); /* Clear Interrupt Flag */
    valid = TRUE;
  }
  
   
  
  
  if (HAL_KEY_JOY_MOVE_PXIFG & HAL_KEY_JOY_MOVE_BIT)  /* Interrupt Flag has been set */
  {
    HAL_KEY_JOY_MOVE_PXIFG = ~(HAL_KEY_JOY_MOVE_BIT); /* Clear Interrupt Flag */
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
HAL_ISR_FUNCTION( halKeyPort0Isr, P0INT_VECTOR )
{
  HAL_ENTER_ISR();

  if (HAL_KEY_SW_6_PXIFG & HAL_KEY_SW_6_BIT)
  {  
    //检测火线和火线输出比较后的电压过零点的下降沿脉冲，过零后就触发TIME4，开始计数，计数值为全局变量DY_TIMER加上一个固定值.
    //这个固定值大概是3ms左右，是为保证单火线的正常工作。如果定时器设定为50US中断一次，10ms(100HZ)总共为计数200次
    //其中前60次为固定值，必须延迟的，DY_TIMER全局变量为0~120为通过按键设定或者无线信号过来的设置，用来设置可控硅导通角
    //最后的20次也就是1ms为保证在过零来到之前关断可控硅

   TIMER34_START(4,1);
    //   Start_T4();
   HAL_KEY_SW_6_PXIFG = 0;
  } 
  HAL_KEY_CPU_PORT_0_IF = 0;
  
 // CLEAR_SLEEP_MODE();
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
//    P1DIR &= ~0X20;//INput
 HAL_ENTER_ISR();
  
    if(HAL_KEY_SW_7_PXIFG & HAL_KEY_SW_7_BIT)
  {
    halProcessKeyInterrupt();

     HAL_KEY_SW_7_PXIFG &= ~(HAL_KEY_SW_7_BIT);;
     key_ISR_FLAG =HAL_KEY_SW_7;
  }

  else if(HAL_KEY_SW_8_PXIFG & HAL_KEY_SW_8_BIT)//p1.2中断标志
  {
     halProcessKeyInterrupt();
    // osal_start_timerEx (Hal_TaskID, HAL_KEY_EVENT, HAL_KEY_DEBOUNCE_VALUE);//25ms以后进入

     HAL_KEY_SW_8_PXIFG &= ~(HAL_KEY_SW_8_BIT);//清除p1.2标志
     key_ISR_FLAG =HAL_KEY_SW_8;
     
  }
  /*
    Clear the CPU interrupt flag for Port_1
    PxIFG has to be cleared before PxIF
  */
  
  
  HAL_KEY_CPU_PORT_1_IF = 0;
 // PCON = 0x00;//唤醒
  
//  CLEAR_SLEEP_MODE();
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

  /*
    Clear the CPU interrupt flag for Port_2
    PxIFG has to be cleared before PxIF
    Notes: P2_1 and P2_2 are debug lines.
  */
  HAL_KEY_JOY_MOVE_PXIFG = 0;
  HAL_KEY_CPU_PORT_2_IF = 0;

  CLEAR_SLEEP_MODE();
  HAL_EXIT_ISR();
}


void HalKeyMultiClickCheck(void)
{
  uint8 keys = 0;
  
  //#if (defined HAL_KEY) && (HAL_KEY == TRUE)
  
  keys |= HAL_KEY_SW_8;
  key_double_flag=key_p1_2_ISR_FLAG;//将几次按键值赋给状态
  if(1 == key_double_flag)
  {
	if((HAL_PUSH_BUTTON3()))
	{
	  key_hold_flag = 0;//长按无效。  key_double_flag 表示按键按下的次数
	  if (keys && (pHalKeyProcessFunction))
	  {
		(pHalKeyProcessFunction) (keys, HAL_KEY_STATE_NORMAL);
		// key_ISR_FLAG=0;
		key_p1_2_ISR_FLAG=0;
	  }
	  
	}
	else
	{
	(pHalKeyProcessFunction) (keys, HAL_KEY_STATE_NORMAL);//将1次击也送到应用层去
	 	osal_start_timerEx (Hal_TaskID, HAL_KEY_HOLD_EVENT, HAL_KEY_HOLD_VALUE);//长按时间（10s）达到以后进入相应的处理事件
		key_holdtime_waiting_flag = 1;
	}
  }
  else
  {
	
	key_hold_flag = 0;//长按无效。  key_double_flag 表示按键按下的次数
	if (keys && (pHalKeyProcessFunction))
	{
	  (pHalKeyProcessFunction) (keys, HAL_KEY_STATE_NORMAL);
	  // key_ISR_FLAG=0;
	  key_p1_2_ISR_FLAG=0;
	}
  }
  
}


void HalKeyHoldCheck(void)
{
  uint8 keys = 0;
  keys |= HAL_KEY_SW_8;
  if(!(HAL_PUSH_BUTTON3()))
  {
	key_hold_flag = 1;//长按有效。  key_double_flag 表示按键按下的次数
	if (keys && (pHalKeyProcessFunction))
	{
	  (pHalKeyProcessFunction) (keys, HAL_KEY_STATE_NORMAL);
	  // key_ISR_FLAG=0;
	  key_p1_2_ISR_FLAG=0;
	}
  }
  key_holdtime_waiting_flag = 0;	//清除等待长按时间标志
  key_p1_2_ISR_FLAG=0;
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




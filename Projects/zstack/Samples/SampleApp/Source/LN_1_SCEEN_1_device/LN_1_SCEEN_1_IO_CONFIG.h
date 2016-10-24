
/******************** (C) COPYRIGHT 2011 Jami***********************************
* File Name		: SRMO_IO_CONFIG.h
* Author		: JamiLiang At Gmail.com
* Date			: 2013/07/17
* Description	: This file provides all the xxx Module functions.
* Version		: V0.1
* ChangeLog	:
* Version		Name       		Date			Description
  0.1			JamiLiang		2013/07/17		Initial Version
   	
*******************************************************************************/
#ifndef CS5463_SOKET_IO_CONFIG_H
#define CS5463_SOKET_IO_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "ioCC2530.h"
/* External variables --------------------------------------------------------*/
/* External functions --------------------------------------------------------*/

/* Public typedef ------------------------------------------------------------*/
/* Public define -------------------------------------------------------------*/
//General LED

//添加的输出
// 老版本
#define LED2_BV           BV(4)
#define LED2_SBIT         P1_4
#define LED2_DDR          P1DIR
#define LED2_POLARITY     ACTIVE_HIGH	//20110721 ACTIVE_HIGH


#define LED3_BV           BV(0)
#define LED3_SBIT         P1_0
#define LED3_DDR          P1DIR
#define LED3_POLARITY     ACTIVE_HIGH

//老版本
#define LED4_BV           BV(5)
#define LED4_SBIT         P1_5
#define LED4_DDR          P1DIR
#define LED4_POLARITY     ACTIVE_HIGH


#define LED5_BV           BV(4)
#define LED5_SBIT         P0_4
#define LED5_DDR          P0DIR
#define LED5_POLARITY     ACTIVE_HIGH

#define LED1_BV           BV(0)
#define LED1_SBIT         P2_0
#define LED1_DDR          P2DIR
#define LED1_POLARITY     ACTIVE_LOW


//老版本
#define OUT1_BV           BV(1)            //火线切换
#define OUT1_SBIT         P0_1             
#define OUT1_DDR          P0DIR            
#define OUT1_POLARITY     ACTIVE_HIGH   


#define OUT2_BV           BV(0)            //火线控制端
#define OUT2_SBIT         P0_0            
#define OUT2_DDR          P0DIR            
#define OUT2_POLARITY     ACTIVE_HIGH 


//老版本
#define OUT3_BV           BV(6)            
#define OUT3_SBIT         P0_6            
#define OUT3_DDR          P0DIR            
#define OUT3_POLARITY     ACTIVE_HIGH 


#define OUT4_BV           BV(2)           
#define OUT4_SBIT         P0_2            
#define OUT4_DDR          P0DIR            
#define OUT4_POLARITY     ACTIVE_HIGH 

#define OUT5_BV           BV(4)            
#define OUT5_SBIT         P0_4            
#define OUT5_DDR          P0DIR            
#define OUT5_POLARITY     ACTIVE_HIGH 

#define OUT6_BV           BV(5)           
#define OUT6_SBIT         P0_5           
#define OUT6_DDR          P0DIR            
#define OUT6_POLARITY     ACTIVE_HIGH 



#define PUSH1_BV          BV(3)
#define PUSH1_SBIT        P1_3
#define PUSH1_POLARITY    ACTIVE_HIGH

#define PUSH2_BV          BV(4)
#define PUSH2_SBIT        P0_4
#define PUSH2_POLARITY    ACTIVE_HIGH

#define PUSH3_BV          BV(2)
#define PUSH3_SBIT        P1_2
#define PUSH3_POLARITY    ACTIVE_HIGH


#define PUSH4_BV          BV(7)
#define PUSH4_SBIT        P1_7
#define PUSH4_POLARITY    ACTIVE_HIGH

#define PUSH5_BV          BV(6)
#define PUSH5_SBIT        P0_6
#define PUSH5_POLARITY    ACTIVE_HIGH

#define HAL_PUSH_BUTTON1()        (PUSH1_POLARITY (PUSH1_SBIT))
#define HAL_PUSH_BUTTON2()        (PUSH2_POLARITY (PUSH2_SBIT))
#define HAL_PUSH_BUTTON3()        (PUSH3_POLARITY (PUSH3_SBIT))
#define HAL_PUSH_BUTTON4()        (PUSH4_POLARITY (PUSH4_SBIT))
#define HAL_PUSH_BUTTON5()        (PUSH5_POLARITY (PUSH5_SBIT))
#define HAL_PUSH_BUTTON6()        (PUSH6_POLARITY (PUSH6_SBIT)




/* Public macro --------------------------------------------------------------*/
#define HAL_IO_CONFIG_INIT()			\
	{									\
          LED1_DDR |= LED1_BV;                                           \
          LED2_DDR |= LED2_BV;                                           \
          LED3_DDR |= LED3_BV;                                           \
          LED4_DDR |= LED4_BV;                                          \
          HAL_TURN_OFF_LED1();                                           \
          HAL_TURN_OFF_LED2();                                           \
          HAL_TURN_OFF_LED3();                                           \
          HAL_TURN_OFF_LED4();                                           \
          OUT1_DDR |= OUT1_BV;                                           \
          OUT2_DDR |= OUT2_BV;                                           \
          OUT1_SBIT = 0;\
          OUT2_SBIT = 0;\
          OUT3_DDR |= OUT3_BV;           \
          OUT4_DDR |= OUT4_BV;              \
          OUT3_SBIT = 0;\
          OUT4_SBIT = 0;\
          OUT5_DDR |= OUT5_BV;           \
          OUT6_DDR |= OUT6_BV;              \
          OUT5_SBIT = 0;\
          OUT6_SBIT = 0;\
	}
/* Public variables ----------------------------------------------------------*/
/* Public function prototypes ------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/


#endif//SRMO_IO_CONFIG_H

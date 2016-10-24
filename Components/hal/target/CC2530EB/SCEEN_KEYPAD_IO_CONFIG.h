#ifndef SCEEN_KEYPAD_IO_CONFIG_H
#define SCEEN_KEYPAD_IO_CONFIG_H


//添加的输出
#define LED1_BV           BV(0)
#define LED1_SBIT         P1_0
#define LED1_DDR          P1DIR
#define LED1_POLARITY     ACTIVE_LOW	//20110721 ACTIVE_HIGH


#define LED2_BV           BV(6)
#define LED2_SBIT         P0_6
#define LED2_DDR          P0DIR
#define LED2_POLARITY     ACTIVE_LOW

#define LED3_BV           BV(6)
#define LED3_SBIT         P0_6
#define LED3_DDR          P0DIR
#define LED3_POLARITY     ACTIVE_LOW


#define LED4_BV           BV(6)
#define LED4_SBIT         P0_6
#define LED4_DDR          P0DIR
#define LED4_POLARITY     ACTIVE_LOW

#define LED5_BV           BV(6)
#define LED5_SBIT         P0_6
#define LED5_DDR          P0DIR
#define LED5_POLARITY     ACTIVE_LOW


#define PUSH1_BV          BV(2)
#define PUSH1_SBIT        P1_2
#define PUSH1_POLARITY    ACTIVE_HIGH

#define PUSH2_BV          BV(3)
#define PUSH2_SBIT        P1_3
#define PUSH2_POLARITY    ACTIVE_HIGH


#define PUSH3_BV          BV(5)
#define PUSH3_SBIT        P1_5
#define PUSH3_POLARITY    ACTIVE_HIGH

#define PUSH4_BV          BV(6)
#define PUSH4_SBIT        P1_6
#define PUSH4_POLARITY    ACTIVE_HIGH

#define PUSH5_BV          BV(7)
#define PUSH5_SBIT        P1_7
#define PUSH5_POLARITY    ACTIVE_HIGH

#define PUSH6_BV          BV(4)
#define PUSH6_SBIT        P0_4
#define PUSH6_POLARITY    ACTIVE_HIGH

#define HAL_PUSH_BUTTON1()        (PUSH1_POLARITY (PUSH1_SBIT))
#define HAL_PUSH_BUTTON2()        (PUSH2_POLARITY (PUSH2_SBIT))
#define HAL_PUSH_BUTTON3()        (PUSH3_POLARITY (PUSH3_SBIT))
#define HAL_PUSH_BUTTON4()        (PUSH4_POLARITY (PUSH4_SBIT))
#define HAL_PUSH_BUTTON5()        (PUSH5_POLARITY (PUSH5_SBIT))
#define HAL_PUSH_BUTTON6()        (PUSH5_POLARITY (PUSH6_SBIT))

//在下面修改IO口配置
#define HAL_IO_CONFIG_INIT()                                     \
{                                                                \
  LED1_DDR |= LED1_BV;                                           \
  LED2_DDR |= LED2_BV;                                           \
  HAL_TURN_OFF_LED1();                                           \
  HAL_TURN_OFF_LED2();                                           \
}                                                                
/*
PUSH4_DIR |= PUSH4_BV;										 \
  PUSH4_SBIT = 0;												 \
*/



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


//p1.2
#define HAL_KEY_SW_1_PORT   P1
#define HAL_KEY_SW_1_BIT    BV(2)
#define HAL_KEY_SW_1_SEL    P1SEL
#define HAL_KEY_SW_1_DIR    P1DIR
#define HAL_KEY_SW_1_EDGEBIT  BV(1)
#define HAL_KEY_SW_1_EDGE     HAL_KEY_FALLING_EDGE
#define HAL_KEY_SW_1_IEN      IEN2  /* CPU interrupt mask register */
#define HAL_KEY_SW_1_IENBIT   BV(4) /* Mask bit for all of Port_0 */
#define HAL_KEY_SW_1_ICTL     P1IEN /* Port Interrupt Control register */
#define HAL_KEY_SW_1_ICTLBIT  BV(2) /* P0IEN - P0.1 enable/disable bit BV(1)*/
#define HAL_KEY_SW_1_PXIFG    P1IFG /* Interrupt flag at source */

//p1.3
#define HAL_KEY_SW_2_PORT   P1
#define HAL_KEY_SW_2_BIT    BV(3)
#define HAL_KEY_SW_2_SEL    P1SEL
#define HAL_KEY_SW_2_DIR    P1DIR
#define HAL_KEY_SW_2_EDGEBIT  BV(1)
#define HAL_KEY_SW_2_EDGE     HAL_KEY_FALLING_EDGE
#define HAL_KEY_SW_2_IEN      IEN2  /* CPU interrupt mask register */
#define HAL_KEY_SW_2_IENBIT   BV(4) /* Mask bit for all of Port_0 */
#define HAL_KEY_SW_2_ICTL     P1IEN /* Port Interrupt Control register */
#define HAL_KEY_SW_2_ICTLBIT  BV(3) /* P0IEN - P0.1 enable/disable bit BV(1)*/
#define HAL_KEY_SW_2_PXIFG    P1IFG /* Interrupt flag at source */

//p1.5
#define HAL_KEY_SW_3_PORT   P1
#define HAL_KEY_SW_3_BIT    BV(5)
#define HAL_KEY_SW_3_SEL    P1SEL
#define HAL_KEY_SW_3_DIR    P1DIR
#define HAL_KEY_SW_3_EDGEBIT  BV(2)//p1端口的触发模式设置 在PICTL的低第三位（位2）P1.5为高4位地址
#define HAL_KEY_SW_3_EDGE     HAL_KEY_FALLING_EDGE  // 上升沿触发
#define HAL_KEY_SW_3_IEN      IEN2  /* CPU interrupt mask register */
#define HAL_KEY_SW_3_IENBIT   BV(4) /* Mask bit for all of Port_1 */
#define HAL_KEY_SW_3_ICTL     P1IEN /* Port Interrupt Control register */
#define HAL_KEY_SW_3_ICTLBIT  BV(5) /* P1IEN - P1.5 enable/disable bit */ 
#define HAL_KEY_SW_3_PXIFG    P1IFG /* Interrupt flag at source */

//p1.6
#define HAL_KEY_SW_4_PORT   P1
#define HAL_KEY_SW_4_BIT    BV(6)
#define HAL_KEY_SW_4_SEL    P1SEL
#define HAL_KEY_SW_4_DIR    P1DIR
#define HAL_KEY_SW_4_EDGEBIT  BV(2)
#define HAL_KEY_SW_4_EDGE     HAL_KEY_FALLING_EDGE
#define HAL_KEY_SW_4_IEN      IEN2  /* CPU interrupt mask register */
#define HAL_KEY_SW_4_IENBIT   BV(4) /* Mask bit for all of Port_0 */
#define HAL_KEY_SW_4_ICTL     P1IEN /* Port Interrupt Control register */
#define HAL_KEY_SW_4_ICTLBIT  BV(6) /* P0IEN - P0.1 enable/disable bit BV(1)*/
#define HAL_KEY_SW_4_PXIFG    P1IFG /* Interrupt flag at source */

//p1.7
#define HAL_KEY_SW_5_PORT   P1
#define HAL_KEY_SW_5_BIT    BV(7)
#define HAL_KEY_SW_5_SEL    P1SEL
#define HAL_KEY_SW_5_DIR    P1DIR
#define HAL_KEY_SW_5_EDGEBIT  BV(2)
#define HAL_KEY_SW_5_EDGE     HAL_KEY_FALLING_EDGE
#define HAL_KEY_SW_5_IEN      IEN2  /* CPU interrupt mask register */
#define HAL_KEY_SW_5_IENBIT   BV(4) /* Mask bit for all of Port_0 */
#define HAL_KEY_SW_5_ICTL     P1IEN /* Port Interrupt Control register */
#define HAL_KEY_SW_5_ICTLBIT  BV(7) /* P0IEN - P0.1 enable/disable bit BV(1)*/
#define HAL_KEY_SW_5_PXIFG    P1IFG /* Interrupt flag at source */

//p0.4
#define HAL_KEY_SW_6_PORT   P0
#define HAL_KEY_SW_6_BIT    BV(4)
#define HAL_KEY_SW_6_SEL    P0SEL
#define HAL_KEY_SW_6_DIR    P0DIR
#define HAL_KEY_SW_6_EDGEBIT  BV(0)
#define HAL_KEY_SW_6_EDGE     HAL_KEY_FALLING_EDGE
#define HAL_KEY_SW_6_IEN      IEN1  /* CPU interrupt mask register */
#define HAL_KEY_SW_6_IENBIT   BV(5) /* Mask bit for all of Port_0 */
#define HAL_KEY_SW_6_ICTL     P0IEN /* Port Interrupt Control register */
#define HAL_KEY_SW_6_ICTLBIT  BV(4) /* P0IEN - P0.1 enable/disable bit BV(1)*/
#define HAL_KEY_SW_6_PXIFG    P0IFG /* Interrupt flag at source */


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








#endif
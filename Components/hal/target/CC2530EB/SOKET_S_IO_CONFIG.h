#ifndef SOCKET_S_IO_CONFIG_H
#define SOCKET_S_IO_CONFIG_H

//添加的输出
#define LED1_BV           BV(1)
#define LED1_SBIT         P2_1
#define LED1_DDR          P2DIR
#define LED1_POLARITY     ACTIVE_LOW	//20110721 ACTIVE_HIGH

  /* 2 - Red */
  #define TG_OUT_BV           BV(3)  //配置OUT2 的输出，控制继电器的开关
  #define TG_OUT_SBIT         P1_3
  #define TG_OUT_DDR          P1DIR
  #define TG_OUT_POLARITY     ACTIVE_HIGH

  /* 3 - Yellow */
  #define LED3_BV           BV(7)
  #define LED3_SBIT         P0_7
  #define LED3_DDR          P0DIR
  #define LED3_POLARITY     ACTIVE_HIGH

  #define LED2_BV           BV(5)
  #define LED2_SBIT         P0_5
  #define LED2_DDR          P0DIR
  #define LED2_POLARITY     ACTIVE_LOW

  #define LED4_BV           BV(7)
  #define LED4_SBIT         P0_7
  #define LED4_DDR          P0DIR
  #define LED4_POLARITY     ACTIVE_HIGH

  #define LED5_BV           BV(7)
  #define LED5_SBIT         P0_7
  #define LED5_DDR          P0DIR
  #define LED5_POLARITY     ACTIVE_HIGH


#define PUSH1_BV          BV(1)
#define PUSH1_SBIT        P0_1
#define PUSH1_POLARITY    ACTIVE_HIGH

/* Joystick Center Press */
#define PUSH2_BV          BV(0)
#define PUSH2_SBIT        P2_0
#define PUSH2_POLARITY    ACTIVE_HIGH


/* 添加的输入 */
#define PUSH3_BV          BV(2)
#define PUSH3_SBIT        P1_2
#define PUSH3_POLARITY    ACTIVE_HIGH

#define PUSH4_BV          BV(4)
#define PUSH4_SBIT        P0_4
#define PUSH4_POLARITY    ACTIVE_HIGH

#define PUSH5_BV          BV(6)
#define PUSH5_SBIT        P1_6
#define PUSH5_POLARITY    ACTIVE_HIGH

#define PUSH6_BV          BV(7)
#define PUSH6_SBIT        P1_7
#define PUSH6_POLARITY    ACTIVE_HIGH

//在下面修改IO口配置
#define HAL_IO_CONFIG_INIT()                                     \
{                                                                \
  LED1_DDR |= LED1_BV;                                           \
  HAL_TURN_OFF_LED1();                                           \
  P1DIR |= TG_OUT_BV;                                        \
  P1_3=0;                                                 \
}                                                                
/*
    P0INP |= PUSH4_BV;
    P1INP |= PUSH5_BV;


*/

#endif
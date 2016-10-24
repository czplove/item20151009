#ifndef TOUCH_DIMMER_IO_CONFIG_2_H
#define TOUCH_DIMMER_IO_CONFIG_2_H

                                            
//添加的输出
#define LED1_BV           BV(0)
#define LED1_SBIT         P1_0
#define LED1_DDR          P1DIR
#define LED1_POLARITY     ACTIVE_HIGH	//20110721 ACTIVE_HIGH

#define LED2_BV           BV(6)
#define LED2_SBIT         P0_6
#define LED2_DDR          P0DIR
#define LED2_POLARITY     ACTIVE_HIGH

#define LED3_BV           BV(5)
#define LED3_SBIT         P0_5
#define LED3_DDR          P0DIR
#define LED3_POLARITY     ACTIVE_HIGH

#define LED4_BV           BV(4)
#define LED4_SBIT         P0_4
#define LED4_DDR          P0DIR
#define LED4_POLARITY     ACTIVE_HIGH


#define LED5_BV           BV(0)
#define LED5_SBIT         P2_0
#define LED5_DDR          P2DIR
#define LED5_POLARITY     ACTIVE_LOW


#define TG_OUT_1_BV           BV(0) //PUSLE OUT
#define TG_OUT_1_SBIT         P0_0
#define TG_OUT_1_DDR          P0DIR
#define TG_OUT_1_POLARITY     ACTIVE_HIGH


#define TG_OUT_2_BV           BV(2) //PUSLE OUT
#define TG_OUT_2_SBIT         P0_2
#define TG_OUT_2_DDR          P0DIR
#define TG_OUT_2_POLARITY     ACTIVE_HIGH

#define PUSH1_BV          BV(1)
#define PUSH1_SBIT        P0_1
#define PUSH1_POLARITY    ACTIVE_HIGH


#define PUSH2_BV          BV(0)
#define PUSH2_SBIT        P2_0
#define PUSH2_POLARITY    ACTIVE_HIGH

/* 添加的输入 */
#define PUSH3_BV          BV(2)   //KEY1
#define PUSH3_SBIT        P1_2
#define PUSH3_POLARITY    ACTIVE_HIGH

#define PUSH4_BV          BV(1)  //ZERO TEST
#define PUSH4_SBIT        P0_1
#define PUSH4_POLARITY    ACTIVE_HIGH

#define PUSH5_BV          BV(5)    //KEY2
#define PUSH5_SBIT        P1_5
#define PUSH5_POLARITY    ACTIVE_HIGH

#define PUSH6_BV          BV(6)    //KEY3
#define PUSH6_SBIT        P1_6
#define PUSH6_POLARITY    ACTIVE_HIGH

#define PUSH7_BV          BV(7)    //KEY4
#define PUSH7_SBIT        P1_7
#define PUSH7_POLARITY    ACTIVE_HIGH


//在下面修改IO口配置
#define HAL_IO_CONFIG_INIT()                                     \
{                                                                \
  LED1_DDR |= LED1_BV;                                           \
  LED2_DDR |= LED2_BV;                                           \
  LED3_DDR |= LED3_BV;                                           \
  LED4_DDR |= LED4_BV;                                           \
  LED5_DDR |= LED5_BV;                                           \
  HAL_TURN_OFF_LED1();                                           \
  HAL_TURN_OFF_LED2();                                           \
  HAL_TURN_OFF_LED3();                                           \
  HAL_TURN_OFF_LED4();                                           \
  HAL_TURN_OFF_LED5();                                           \
  TG_OUT_1_DDR |= TG_OUT_1_BV; \
  TG_OUT_2_DDR |= TG_OUT_2_BV; \
  TG_OUT_1_SBIT=0;\
  TG_OUT_2_SBIT=0;\
  P0INP |= PUSH1_BV;\
} 

//P1INP |= PUSH5_BV;
/*


*/

#endif
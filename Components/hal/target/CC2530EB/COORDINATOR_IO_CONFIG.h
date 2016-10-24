#ifndef COORDINATOR_IO_CONFIG_H
#define COORDINATOR_IO_CONFIG_H

//添加的输出
#define LED1_BV           BV(0)
#define LED1_SBIT         P1_0
#define LED1_DDR          P1DIR
#define LED1_POLARITY     ACTIVE_LOW	//20110721 ACTIVE_HIGH

#define LED2_BV           BV(3)
#define LED2_SBIT         P1_3
#define LED2_DDR          P1DIR
#define LED2_POLARITY     ACTIVE_LOW


#define LED3_BV           BV(7)
#define LED3_SBIT         P1_7
#define LED3_DDR          P1DIR
#define LED3_POLARITY     ACTIVE_HIGH

#define LED4_BV           BV(7)
#define LED4_SBIT         P1_7
#define LED4_DDR          P1DIR
#define LED4_POLARITY     ACTIVE_HIGH

#define LED5_BV           BV(7)
#define LED5_SBIT         P1_7
#define LED5_DDR          P1DIR
#define LED5_POLARITY     ACTIVE_HIGH

#define PUSH1_BV          BV(1)
#define PUSH1_SBIT        P0_1
#define PUSH1_POLARITY    ACTIVE_HIGH


//添加的输入 
#define PUSH2_BV          BV(0)
#define PUSH2_SBIT        P2_0
#define PUSH2_POLARITY    ACTIVE_HIGH

#define PUSH3_BV          BV(2)
#define PUSH3_SBIT        P1_2
#define PUSH3_POLARITY    ACTIVE_HIGH





//在下面修改IO口配置
#define HAL_IO_CONFIG_INIT()                                     \
{                                                                \
  LED1_DDR |= LED1_BV;                                           \
  LED2_DDR |= LED2_BV;                                           \
  HAL_TURN_OFF_LED1();                                           \
  HAL_TURN_OFF_LED2();                                           \
}                                                                


#endif
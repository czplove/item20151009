#ifndef AUTODOOR_IO_CONFIG_H
#define AUTODOOR_IO_CONFIG_H

//添加的输出
#define LED1_BV           BV(6)
#define LED1_SBIT         P0_6
#define LED1_DDR          P0DIR
#define LED1_POLARITY    ACTIVE_LOW	// ACTIVE_HIGH////20110721 ACTIVE_HIGH

#define LED2_BV           BV(5)
#define LED2_SBIT         P0_5
#define LED2_DDR          P0DIR
#define LED2_POLARITY     ACTIVE_LOW    //ACTIVE_HIGH //

#define LED3_BV           BV(0)   // as  beep
#define LED3_SBIT         P0_0
#define LED3_DDR          P0DIR
#define LED3_POLARITY     ACTIVE_HIGH

#define LED4_BV           BV(0)   // as  beep
#define LED4_SBIT         P2_0
#define LED4_DDR          P2DIR
#define LED4_POLARITY     ACTIVE_HIGH

#define LED5_BV           BV(0)   // as  beep
#define LED5_SBIT         P2_0
#define LED5_DDR          P2DIR
#define LED5_POLARITY     ACTIVE_HIGH


#define OUT1_BV           BV(1)            //复位
#define OUT1_SBIT         P0_1             
#define OUT1_DDR          P0DIR            
#define OUT1_POLARITY     ACTIVE_HIGH   

#define OUT2_BV           BV(2)            //常闭
#define OUT2_SBIT         P0_2             
#define OUT2_DDR          P0DIR            
#define OUT2_POLARITY     ACTIVE_HIGH 

#define OUT3_BV           BV(3)            //开门
#define OUT3_SBIT         P0_3             
#define OUT3_DDR          P0DIR            
#define OUT3_POLARITY     ACTIVE_HIGH   

#define OUT4_BV           BV(4)            //常开
#define OUT4_SBIT         P0_4             
#define OUT4_DDR          P0DIR            
#define OUT4_POLARITY     ACTIVE_HIGH 



#define PUSH1_BV          BV(1)
#define PUSH1_SBIT        P0_1
#define PUSH1_POLARITY    ACTIVE_HIGH

//添加的输入 
#define PUSH0_BV          BV(0)
#define PUSH0_SBIT        P1_0
#define PUSH0_POLARITY    ACTIVE_HIGH

#define PUSH3_BV          BV(3)
#define PUSH3_SBIT        P1_3
#define PUSH3_POLARITY    ACTIVE_HIGH

#define PUSH4_BV          BV(2)
#define PUSH4_SBIT        P1_2
#define PUSH4_POLARITY    ACTIVE_HIGH

#define PUSH5_BV          BV(5)
#define PUSH5_SBIT        P1_5
#define PUSH5_POLARITY    ACTIVE_HIGH

#define PUSH6_BV          BV(6)
#define PUSH6_SBIT        P1_6
#define PUSH6_POLARITY    ACTIVE_HIGH

#define PUSH7_BV          BV(7)
#define PUSH7_SBIT        P1_7
#define PUSH7_POLARITY    ACTIVE_HIGH

//在下面修改IO口配置
#define HAL_IO_CONFIG_INIT()                                     \
{                                                                \
  LED1_DDR |= LED1_BV;                                           \
  LED2_DDR |= LED2_BV;                                           \
  LED3_DDR |= LED3_BV;                                           \
  HAL_TURN_OFF_LED1();                                           \
  HAL_TURN_OFF_LED2();                                           \
  HAL_TURN_OFF_LED3();                                           \
  OUT1_DDR |= OUT1_BV;                                           \
  OUT2_DDR |= OUT2_BV;                                           \
  OUT3_DDR |= OUT3_BV;                                           \
  OUT4_DDR |= OUT4_BV;                                           \
  OUT1_SBIT = 0;                                                 \
  OUT2_SBIT = 0;                                                 \
  OUT3_SBIT = 0;                                                 \
  OUT4_SBIT = 0;                                                 \
}                                                                


#endif
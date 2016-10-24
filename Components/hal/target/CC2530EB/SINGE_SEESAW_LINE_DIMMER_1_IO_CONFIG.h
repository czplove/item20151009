#ifndef SINGE_SEESAW_LINE_DIMMER_1_IO_CONFIG_H
#define SINGE_SEESAW_LINE_DIMMER_1_IO_CONFIG_H

//添加的输出
// 老版本
#define LED1_BV           BV(0)
#define LED1_SBIT         P1_0
#define LED1_DDR          P1DIR
#define LED1_POLARITY     ACTIVE_HIGH	//20110721 ACTIVE_HIGH


#define LED2_BV           BV(6)
#define LED2_SBIT         P0_6
#define LED2_DDR          P0DIR
#define LED2_POLARITY     ACTIVE_HIGH

//老版本
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

#define TG_OUT_BV           BV(0)            //脉冲控制端
#define TG_OUT_SBIT         P0_0             
#define TG_OUT_DDR          P0DIR            
#define TG_OUT_POLARITY     ACTIVE_HIGH   


#define OUT2_BV           BV(1)            //火线控制端
#define OUT2_SBIT         P0_1            
#define OUT2_DDR          P0DIR            
#define OUT2_POLARITY     ACTIVE_HIGH 



#define OUT3_BV           BV(2)            //火线控制端
#define OUT3_SBIT         P0_2            
#define OUT3_DDR          P0DIR            
#define OUT3_POLARITY     ACTIVE_HIGH 


#define OUT4_BV           BV(3)            //火线控制端
#define OUT4_SBIT         P0_3            
#define OUT4_DDR          P0DIR            
#define OUT4_POLARITY     ACTIVE_HIGH 

// new
#define PUSH1_BV          BV(6)
#define PUSH1_SBIT        P1_6
#define PUSH1_POLARITY    ACTIVE_HIGH

#define PUSH2_BV          BV(5)
#define PUSH2_SBIT        P1_5
#define PUSH2_POLARITY    ACTIVE_HIGH

#define PUSH3_BV          BV(2)
#define PUSH3_SBIT        P1_2
#define PUSH3_POLARITY    ACTIVE_HIGH


#define PUSH4_BV          BV(7)
#define PUSH4_SBIT        P1_7
#define PUSH4_POLARITY    ACTIVE_HIGH

#define PUSH5_BV          BV(7)
#define PUSH5_SBIT        P1_7
#define PUSH5_POLARITY    ACTIVE_HIGH


#define ZERO_BV          BV(1)    //过零
#define ZERO_SBIT        P0_1
#define ZERO_POLARITY    ACTIVE_HIGH

#define MONZ_2             P0_7
#define MONZ_1             P1_1
#define MONZ_3             P1_4

//在下面修改IO口配置
#define HAL_IO_CONFIG_INIT()                                     \
{                                                                \
  LED1_DDR |= LED1_BV;                                           \
  LED2_DDR |= LED2_BV;                                           \
  LED5_DDR |= LED5_BV;                                          \
  HAL_TURN_OFF_LED1();                                           \
  HAL_TURN_OFF_LED2();                                           \
  HAL_TURN_OFF_LED5();                                           \
  TG_OUT_DDR |= TG_OUT_BV;                                           \
  TG_OUT_SBIT = 0;\
}     

 //LED5_DDR |= LED5_BV;                                          


#endif
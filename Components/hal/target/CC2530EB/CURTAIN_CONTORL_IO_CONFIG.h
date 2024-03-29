#ifndef CURTAIN_CONTORL_IO_CONFIG_H
#define CURTAIN_CONTORL_IO_CONFIG_H

//添加的输出
#define LED1_BV           BV(6)
#define LED1_SBIT         P0_6
#define LED1_DDR          P0DIR
#define LED1_POLARITY     ACTIVE_LOW	//20110721 ACTIVE_HIGH

#define LED2_BV           BV(5)
#define LED2_SBIT         P0_5
#define LED2_DDR          P0DIR
#define LED2_POLARITY     ACTIVE_LOW


#define LED3_BV           BV(7)
#define LED3_SBIT         P1_7
#define LED3_DDR          P1DIR
#define LED3_POLARITY     ACTIVE_HIGH

#define LED4_BV           BV(7)
#define LED4_SBIT         P1_7
#define LED4_DDR          P1DIR
#define LED4_POLARITY     ACTIVE_HIGH

#define LED5_BV           BV(0)
#define LED5_SBIT         P2_0
#define LED5_DDR          P2DIR
#define LED5_POLARITY     ACTIVE_HIGH

#define OUT1_BV           BV(1)            //火线切换
#define OUT1_SBIT         P1_1             
#define OUT1_DDR          P1DIR            
#define OUT1_POLARITY     ACTIVE_HIGH   

#define OUT2_BV           BV(0)            //零线控制端
#define OUT2_SBIT         P1_0             
#define OUT2_DDR          P1DIR            
#define OUT2_POLARITY     ACTIVE_HIGH 



#define PUSH1_BV          BV(1)
#define PUSH1_SBIT        P0_1
#define PUSH1_POLARITY    ACTIVE_HIGH


//添加的输入 
#define PUSH3_BV          BV(3)
#define PUSH3_SBIT        P1_3
#define PUSH3_POLARITY    ACTIVE_HIGH

#define PUSH4_BV          BV(2)
#define PUSH4_SBIT        P1_2
#define PUSH4_POLARITY    ACTIVE_HIGH

#define PUSH5_BV          BV(4)
#define PUSH5_SBIT        P1_4
#define PUSH5_POLARITY    ACTIVE_HIGH



//在下面修改IO口配置
#define HAL_IO_CONFIG_INIT()                                     \
{                                                                \
  LED1_DDR |= LED1_BV;                                           \
  LED2_DDR |= LED2_BV;                                           \
  HAL_TURN_OFF_LED1();                                           \
  HAL_TURN_OFF_LED2();                                           \
  OUT1_DDR |= OUT1_BV;                                           \
  OUT2_DDR |= OUT2_BV;                                           \
}                                                                


#endif
#ifndef BATTERY_CURTAIN_CONTORL_CONTORL_IO_CONFIG_H
#define BATTERY_CURTAIN_CONTORL_CONTORL_IO_CONFIG_H

//添加的输出
#define LED1_BV           BV(0)
#define LED1_SBIT         P1_0
#define LED1_DDR          P1DIR
#define LED1_POLARITY    ACTIVE_HIGH	// ACTIVE_HIGH////20110721 ACTIVE_HIGH

#define LED2_BV           BV(7)
#define LED2_SBIT         P0_7
#define LED2_DDR          P0DIR
#define LED2_POLARITY     ACTIVE_HIGH    //ACTIVE_HIGH //

#define LED3_BV           BV(7)   // as  beep
#define LED3_SBIT         P0_7
#define LED3_DDR          P0DIR
#define LED3_POLARITY     ACTIVE_HIGH

#define LED4_BV           BV(0)   // as  beep
#define LED4_SBIT         P0_0
#define LED4_DDR          P0DIR
#define LED4_POLARITY     ACTIVE_HIGH

#define LED5_BV           BV(0)   // as  beep
#define LED5_SBIT         P0_0
#define LED5_DDR          P0DIR
#define LED5_POLARITY     ACTIVE_HIGH

#define OUT1_BV           BV(5)            
#define OUT1_SBIT         P1_5             
#define OUT1_DDR          P1DIR            
#define OUT1_POLARITY     ACTIVE_HIGH   

#define OUT2_BV           BV(1)            
#define OUT2_SBIT         P1_1             
#define OUT2_DDR          P1DIR            
#define OUT2_POLARITY     ACTIVE_HIGH 

#define OUT3_BV           BV(7)          
#define OUT3_SBIT         P1_7             
#define OUT3_DDR          P1DIR            
#define OUT3_POLARITY     ACTIVE_LOW   

#define OUT4_BV           BV(7)            
#define OUT4_SBIT         P1_7             
#define OUT4_DDR          P1DIR            
#define OUT4_POLARITY     ACTIVE_HIGH 



#define PUSH1_BV          BV(1)
#define PUSH1_SBIT        P0_1
#define PUSH1_POLARITY    ACTIVE_HIGH

//添加的输入 
#define PUSH0_BV          BV(7)
#define PUSH0_SBIT        P1_7
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

//在下面修改IO口配置   LED2作为输出IO口用 用以唤醒主板

#define HAL_IO_CONFIG_INIT()                                     \
{                                                                \
  LED1_DDR |= LED1_BV;                                           \
  HAL_TURN_OFF_LED1();                                           \
  OUT1_DDR |= OUT1_BV;                                           \
  OUT1_SBIT = 0;                                                 \
  OUT2_DDR |= OUT2_BV;                                           \
  OUT2_SBIT = 1;                                                 \
} 
#endif

//OUT2_SBIT 在初始化拉低后约2ms 再次跟串口初始化一起拉高  
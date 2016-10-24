#ifndef ELOCK_IO_CONFIG_H
#define ELOCK_IO_CONFIG_H

//添加的输出

//添加的输出

#define LED1_BV           BV(6) //green
#define LED1_SBIT         P0_6
#define LED1_DDR          P0DIR
#define LED1_POLARITY     ACTIVE_LOW//ACTIVE_LOW	//20110721 ACTIVE_HIGH

#define LED2_BV           BV(5) //red
#define LED2_SBIT         P0_5
#define LED2_DDR          P0DIR
#define LED2_POLARITY     ACTIVE_HIGH//ACTIVE_HIGH//ACTIVE_LOW


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
#define LED5_POLARITY     ACTIVE_LOW//ACTIVE_HIGH

#define OUT1_BV           BV(1)            //在初始化时用来移位设置方向的  
#define OUT1_SBIT         P1_1             //继电器输出 P1.3 终端板
#define OUT1_DDR          P1DIR            //方向寄存器
#define OUT1_POLARITY     ACTIVE_HIGH   

#define OUT2_BV           BV(0)            //在初始化时用来移位设置方向的  
#define OUT2_SBIT         P1_0             //继电器输出 P1.3 终端板
#define OUT2_DDR          P1DIR            //方向寄存器
#define OUT2_POLARITY     ACTIVE_HIGH   
/*
#define OUT3_BV           BV(4)            //在初始化时用来移位设置方向的  
#define OUT3_SBIT         P1_4             //继电器输出 P1.3 终端板
#define OUT3_DDR          P1DIR            //方向寄存器
#define OUT3_POLARITY     ACTIVE_HIGH   
*/

#define PUSH1_BV          BV(1)
#define PUSH1_SBIT        P0_1
#define PUSH1_POLARITY    ACTIVE_HIGH
/*
#define PUSH4_BV          BV(4)
#define PUSH4_SBIT        P0_4
#define PUSH4_DIR         P0DIR
#define PUSH4_POLARITY    ACTIVE_HIGH

*/
//添加的输入 
#define PUSH3_BV          BV(2)
#define PUSH3_SBIT        P1_2			//钥匙
#define PUSH3_POLARITY    ACTIVE_HIGH

#define PUSH4_BV          BV(4)			//大舌头，保险
#define PUSH4_SBIT        P1_4
#define PUSH4_POLARITY    ACTIVE_HIGH

#define PUSH5_BV          BV(3)			//小舌头，锁头
#define PUSH5_SBIT        P1_3
#define PUSH5_POLARITY    ACTIVE_HIGH

#define PUSH6_BV          BV(5)
#define PUSH6_SBIT        P0_5
#define PUSH6_POLARITY    ACTIVE_HIGH

//在下面修改IO口配置
#define HAL_IO_CONFIG_INIT()                                     \
{                                                                \
  LED1_DDR |= LED1_BV;                                           \
  LED2_DDR |= LED2_BV;                                           \
  HAL_TURN_OFF_LED1();                                           \
  HAL_TURN_OFF_LED2();                                           \
  OUT1_DDR |= OUT1_BV;                                           \
  OUT2_DDR |= OUT2_BV;                                           \
  OUT1_SBIT = 1;\
  OUT2_SBIT = 1;\
  P1DIR  &= ~0x20;\
  P1DIR  &= ~0x10;\
  P1DIR  &= ~0x08;\
  P1DIR  &= ~0x04;\
  P1INP |= 0x20;/*P1_5 蜂鸣器不用，设为3太输入 */                                             \
/*  将P0.4 P1.5 P0.5设为3态状态，外接上啦和下拉 P0.5给红外的放拆用*/  \
}                                                                
/*
  OUT3_DDR |= OUT3_BV;                                           \
     OUT3_SBIT = 0;\
P0INP |= PUSH4_BV;                                             \
  P1INP |= PUSH5_BV;                                             \
  P0INP |= PUSH5_BV;                                             \
*/

#endif
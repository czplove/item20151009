#ifndef CPT_V2_IO_CONFIG_H
#define CPT_V2_IO_CONFIG_H

//添加的输出
#define LED1_BV           BV(0)
#define LED1_SBIT         P1_0
#define LED1_DDR          P1DIR
#define LED1_POLARITY     ACTIVE_LOW	//20110721 ACTIVE_HIGH

#define LED2_BV           BV(7)
#define LED2_SBIT         P1_7
#define LED2_DDR          P0DIR
#define LED2_POLARITY     ACTIVE_LOW


#define LED3_BV           BV(7)
#define LED3_SBIT         P1_7
#define LED3_DDR          P1DIR
#define LED3_POLARITY     ACTIVE_HIGH


#define PUSH1_BV          BV(6)
#define PUSH1_SBIT        P0_6
#define PUSH1_DIR         P0DIR
#define PUSH1_POLARITY    ACTIVE_HIGH

#define PUSH4_BV          BV(5)
#define PUSH4_SBIT        P0_5
#define PUSH4_DIR         P0DIR
#define PUSH4_POLARITY    ACTIVE_HIGH


//添加的输入 
#define PUSH3_BV          BV(2)
#define PUSH3_SBIT        P1_2
#define PUSH3_POLARITY    ACTIVE_HIGH


#define PUSH5_BV          BV(5)
#define PUSH5_SBIT        P1_5
#define PUSH5_POLARITY    ACTIVE_HIGH

#define PUSH6_BV          BV(3)
#define PUSH6_SBIT        P1_3
#define PUSH6_POLARITY    ACTIVE_HIGH

//在下面修改IO口配置
/*P0用作外设接口*/
/*优先作为TIMER1()1st priority: Timer 1 channels 0-1*/
#define HAL_IO_CONFIG_INIT()                                     \
{                                                                \
  LED1_DDR |= LED1_BV;                                           \
  HAL_TURN_OFF_LED1();                                           \
  P1SEL |= 0x08;/**//*p1.3 红外发射 peripheral*/\
/*  P1DIR |= 0x08;*//*p1.3 OUTPUT*/\
  P0DIR |= 0x01;/*p0.0 OUTPUT*/\
  P0DIR |= 0x02;/*p0.1 OUTPUT*/\
  P0SEL |= 0x08;                                                 \
  P2DIR = 0X80;                                                 \
  /*  将P0.4 P1.5 P0.5设为3态状态，外接上啦和下拉 P0.5给红外的放拆用*/  \
}                                                                


/*
PUSH4_DIR |= PUSH4_BV;										 \
  PUSH4_SBIT = 0;												 \
*/

#endif
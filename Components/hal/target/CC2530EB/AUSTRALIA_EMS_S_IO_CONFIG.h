#ifndef AUSTRALIA_EMS_S_IO_CONFIG_H
#define AUSTRALIA_EMS_S_IO_CONFIG_H

//添加的输出
#define LED1_BV           BV(0)
#define LED1_SBIT         P0_0//2.1
#define LED1_DDR          P0DIR
#define LED1_POLARITY     ACTIVE_LOW	//20110721 ACTIVE_HIGH

#define LED2_BV           BV(3)//BV(3)
#define LED2_SBIT         P1_3//P1_3//P0_1//P2_1
#define LED2_DDR          P1DIR//P0DIR//P2DIR
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

#define OUT1_BV           BV(1)            //在初始化时用来移位设置方向的  
#define OUT1_SBIT         P0_1             //继电器输出 P1.3 终端板
#define OUT1_DDR          P0DIR            //方向寄存器
#define OUT1_POLARITY     ACTIVE_HIGH   


#define PUSH1_BV          BV(1)
#define PUSH1_SBIT        P0_1
#define PUSH1_POLARITY    ACTIVE_HIGH


#define PUSH2_BV          BV(1)
#define PUSH2_SBIT        P0_1
#define PUSH2_POLARITY    ACTIVE_HIGH
//添加的输入 



#define PUSH3_BV          BV(2)
#define PUSH3_SBIT        P1_2
#define PUSH3_POLARITY    ACTIVE_HIGH

#define PUSH4_BV          BV(6)
#define PUSH4_SBIT        P1_6
#define PUSH4_POLARITY    ACTIVE_HIGH

#define PUSH5_BV          BV(5)
#define PUSH5_SBIT        P1_5
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
    OUT1_DDR |=  OUT1_BV;\
    OUT1_SBIT = 0;\
}                                                                
/*
	//POWER_DOWN_SEL &= ~(POWER_DOWN_BIT);     Set pin function to GPIO 
    //POWER_DOWN_DIR &= ~(POWER_DOWN_BIT);     Set pin direction to Input 
    //POWER_DOWN_ICTL &= ~(POWER_DOWN_ICTLBIT);
    //POWER_DOWN_PXIFG &= ~(POWER_DOWN_BIT);清中断标记位
    //POWER_DOWN_IEN |= POWER_DOWN_IENBIT;使能整个CPU的P0口中断
    //PICTL |= POWER_DOWN_EDGEBIT;下降沿
    //POWER_DOWN_ICTL |= POWER_DOWN_ICTLBIT;
*/

#define HAL_POWER_DOWN_INIT()                                     \
{                                                                \
    POWER_DOWN_SEL &= ~(POWER_DOWN_BIT);    /* Set pin function to GPIO */\
    POWER_DOWN_DIR &= ~(POWER_DOWN_BIT);    /* Set pin direction to Input */\
    POWER_DOWN_PXIFG &= ~(POWER_DOWN_BIT);/*清中断标记位*/\
    PICTL |= POWER_DOWN_EDGEBIT;/*下降沿*/\
    POWER_DOWN_IEN |= POWER_DOWN_IENBIT;/*使能整个CPU的P0口中断*/\
    POWER_DOWN_ICTL |= POWER_DOWN_ICTLBIT;/*P0IEN | 00010000  即P0.4使能中断*/\
}                                                                


/*
  P0INP |= PUSH4_BV;                                             \
  P1INP |= PUSH5_BV;                                             \
  P0INP |= PUSH5_BV;                                             \
*/



#endif
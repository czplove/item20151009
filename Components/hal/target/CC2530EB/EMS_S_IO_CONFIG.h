#ifndef EMS_S_IO_CONFIG_H
#define EMS_S_IO_CONFIG_H

//添加的输出
#define LED1_BV           BV(1)//BV(0)//BV(1)
#define LED1_SBIT         P2_1//P0_0//P2_1//2.1
#define LED1_DDR          P2DIR//P0DIR//P2DIR
#define LED1_POLARITY     ACTIVE_LOW//ACTIVE_LOW	//20110721 ACTIVE_HIGH

#define LED2_BV           BV(1)//BV(0)//BV(1)
#define LED2_SBIT         P2_1//P0_0//P2_1//P2_1
#define LED2_DDR          P2DIR//P0DIR//P2DIR
#define LED2_POLARITY     ACTIVE_LOW//ACTIVE_LOW


#define LED3_BV           BV(4)
#define LED3_SBIT         P1_4
#define LED3_DDR          P1DIR
#define LED3_POLARITY     ACTIVE_HIGH

#define LED4_BV           BV(4)
#define LED4_SBIT         P1_4
#define LED4_DDR          P1DIR
#define LED4_POLARITY     ACTIVE_HIGH

#define LED5_BV           BV(4)
#define LED5_SBIT         P1_4
#define LED5_DDR          P1DIR
#define LED5_POLARITY     ACTIVE_HIGH

#define OUT1_BV           BV(3)//BV(1) //BV(3)            //在初始化时用来移位设置方向的  
#define OUT1_SBIT         P1_3//P0_1//P1_3             //继电器输出 P1.3 终端板
#define OUT1_DDR          P1DIR//P0DIR//P1DIR            //方向寄存器
#define OUT1_POLARITY     ACTIVE_HIGH   


#define PUSH1_BV          BV(1)
#define PUSH1_SBIT        P0_1
#define PUSH1_POLARITY    ACTIVE_HIGH

//添加的输入 

/* POWER_DOWN is at P1.0 */
#define POWER_DOWN_PORT   P1
#define POWER_DOWN_BIT    BV(0)//BV(1)
#define POWER_DOWN_SEL    P1SEL
#define POWER_DOWN_DIR    P1DIR

/* edge interrupt */
#define POWER_DOWN_EDGEBIT  BV(1)//p1端口(0~3)的触发模式设置 在PICTL_bit1
#define POWER_DOWN_EDGE     HAL_KEY_FALLING_EDGE


/* POWER_DOWN interrupts */
#define POWER_DOWN_IEN      IEN2  /* CPU interrupt mask register */
#define POWER_DOWN_IENBIT   BV(4) /* Mask bit for all of Port_1 */
#define POWER_DOWN_ICTL     P1IEN /* Port Interrupt Control register */
#define POWER_DOWN_ICTLBIT  BV(0) /* P0IEN - P0.4 enable/disable bit BV(4)*/
#define POWER_DOWN_PXIFG    P1IFG /* Interrupt flag at source */




#define PUSH3_BV          BV(2)
#define PUSH3_SBIT        P1_2
#define PUSH3_POLARITY    ACTIVE_HIGH

#define PUSH4_BV          BV(4)
#define PUSH4_SBIT        P1_4
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
    OUT1_DDR  |= OUT1_BV;\
    OUT1_SBIT = 1;  \
    HAL_TURN_OFF_LED1();                                           \
    HAL_TURN_OFF_LED2();                                           \
    POWER_DOWN_ICTL &= ~(POWER_DOWN_ICTLBIT);\
	POWER_DOWN_SEL &= ~(POWER_DOWN_BIT);    /* Set pin function to GPIO */\
    POWER_DOWN_DIR &= ~(POWER_DOWN_BIT);    /* Set pin direction to Input */\
    POWER_DOWN_PXIFG &= ~(POWER_DOWN_BIT);/*清中断标记位*/\
    POWER_DOWN_IEN |= POWER_DOWN_IENBIT;/*使能整个CPU的P0口中断*/\
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
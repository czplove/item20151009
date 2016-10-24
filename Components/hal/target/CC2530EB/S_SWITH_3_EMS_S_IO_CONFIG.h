#ifndef S_SWITH_3_EMS_S_IO_CONFIG_H
#define S_SWITH_3_EMS_S_IO_CONFIG_H

//添加的输出
#define LED1_BV           BV(0)//BV(2)//BV(0)//BV(2)
#define LED1_SBIT         P1_0//P2_2//P1_0//P2_2
#define LED1_DDR          P1DIR//P2DIR//P1DIR//P2DIR
#define LED1_POLARITY     ACTIVE_LOW	//20110721 ACTIVE_HIGH

#define LED2_BV           BV(0)//BV(1)//BV(0)//BV(1)
#define LED2_SBIT         P1_0//P2_1//P1_0//P2_1
#define LED2_DDR         P1DIR// P2DIR//P1DIR// P2DIR//P0DIR//P2DIR
#define LED2_POLARITY     ACTIVE_LOW


#define LED3_BV           BV(0)
#define LED3_SBIT         P1_0
#define LED3_DDR          P1DIR
#define LED3_POLARITY     ACTIVE_HIGH

#define LED4_BV           BV(0)
#define LED4_SBIT         P1_0
#define LED4_DDR          P1DIR
#define LED4_POLARITY     ACTIVE_HIGH

#define LED5_BV         BV(0)//BV(1)//BV(0)//BV(1)// BV(0)// BV(1)
#define LED5_SBIT       P1_0//P2_1//P1_0//   P2_1//P1_0//P2_1
#define LED5_DDR        P1DIR//P2DIR//P1DIR// P2DIR//P1DIR// P2DIR
#define LED5_POLARITY   ACTIVE_LOW//  ACTIVE_LOW

#define OUT1_BV           BV(0)            //火线切换
#define OUT1_SBIT         P0_0             
#define OUT1_DDR          P0DIR            
#define OUT1_POLARITY     ACTIVE_HIGH   


#define OUT2_BV           BV(1)            //火线控制端
#define OUT2_SBIT         P0_1            
#define OUT2_DDR          P0DIR            
#define OUT2_POLARITY     ACTIVE_HIGH 



#define OUT3_BV           BV(3)            //火线控制端
#define OUT3_SBIT         P1_3            
#define OUT3_DDR          P1DIR            
#define OUT3_POLARITY     ACTIVE_HIGH   



#define IO_LATC_BV           BV(0)            //CLK
#define IO_LATC_SBIT         P1_0            
#define IO_LATC_DDR          P1DIR            
#define IO_LATC_POLARITY     ACTIVE_HIGH


#define IO_SCK_BV           BV(0)            //SCL
#define IO_SCK_SBIT         P0_0            
#define IO_SCK_DDR          P0DIR            
#define IO_SCK_POLARITY     ACTIVE_HIGH

#define IO_SDA_BV           BV(1)            //SDA
#define IO_SDA_SBIT         P0_1            
#define IO_SDA_DDR          P0DIR            
#define IO_SDA_POLARITY     ACTIVE_HIGH


#define PUSH2_BV          BV(5)
#define PUSH2_SBIT        P1_5
#define PUSH2_POLARITY    ACTIVE_HIGH



#define PUSH1_BV          BV(6)
#define PUSH1_SBIT        P1_6
#define PUSH1_POLARITY    ACTIVE_HIGH

//new
#define PUSH3_BV          BV(2)
#define PUSH3_SBIT        P1_2
#define PUSH3_POLARITY    ACTIVE_HIGH



#define PUSH4_BV          BV(7)
#define PUSH4_SBIT        P1_7
#define PUSH4_POLARITY    ACTIVE_HIGH

#define PUSH5_BV          BV(7)
#define PUSH5_SBIT        P1_7
#define PUSH5_POLARITY    ACTIVE_HIGH


//添加的输入 




//在下面修改IO口配置
#define HAL_IO_CONFIG_INIT()                                     \
{                                                                \
    IO_LATC_DDR |= IO_LATC_BV;                                           \
    IO_SCK_DDR |= IO_SCK_BV;                                           \
    IO_SDA_DDR |= IO_SDA_BV;                                           \
    IO_LATC_SBIT = 0;\
    IO_SCK_SBIT = 0;\
    IO_SDA_SBIT =0;\
    LED5_DDR |= LED5_BV;\
    HAL_TURN_OFF_LED5(); \
}                                                             
                                          
      //POWER_DOWN_ICTL &= ~(POWER_DOWN_ICTLBIT);
    //POWER_DOWN_SEL &= ~(POWER_DOWN_BIT);    /* Set pin function to GPIO */
    //POWER_DOWN_DIR &= ~(POWER_DOWN_BIT);    /* Set pin direction to Input */
    //POWER_DOWN_PXIFG &= ~(POWER_DOWN_BIT);/*清中断标记位*/
    //POWER_DOWN_IEN |= POWER_DOWN_IENBIT;/*使能整个CPU的P0口中断*/


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
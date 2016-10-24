#ifndef ELE_SCALE_IO_CONFIG_H
#define ELE_SCALE_IO_CONFIG_H

//添加的输出
#define LED1_BV           BV(0)
#define LED1_SBIT         P1_0
#define LED1_DDR          P1DIR
#define LED1_POLARITY     ACTIVE_LOW	//20110721 ACTIVE_HIGH

#define LED2_BV           BV(1)
#define LED2_SBIT         P1_1
#define LED2_DDR          P1DIR
#define LED2_POLARITY     ACTIVE_LOW


#define LED3_BV           BV(7)
#define LED3_SBIT         P1_7
#define LED3_DDR          P1DIR
#define LED3_POLARITY     ACTIVE_HIGH

#define SCL_BV           BV(5)
#define SCL_SBIT         P1_5
#define SCL_DDR          P1DIR
#define SCL_POLARITY     ACTIVE_HIGH


#define AVDD_EN_BV              BV(4)
#define AVDD_EN_SBIT            P0_4
#define AVDD_EN_DDR             P0DIR
#define AVDD_EN_POLARITY        ACTIVE_HIGH



#define PUSH1_BV          BV(1)
#define PUSH1_SBIT        P0_1
#define PUSH1_POLARITY    ACTIVE_HIGH

//添加的输入 
#define PUSH3_BV          BV(2)
#define PUSH3_SBIT        P1_2
#define PUSH3_POLARITY    ACTIVE_HIGH

#define PUSH4_BV          BV(3)
#define PUSH4_SBIT        P1_3
#define PUSH4_POLARITY    ACTIVE_HIGH

#define PUSH5_BV          BV(7)
#define PUSH5_SBIT        P1_7
#define PUSH5_POLARITY    ACTIVE_HIGH


#define SDA_BV           BV(4)
#define SDA_SBIT         P1_4
#define SDA_DDR          P1DIR
#define SDA_POLARITY     ACTIVE_HIGH

#define PUSH6_BV          BV(5)
#define PUSH6_SBIT        P0_5
#define PUSH6_POLARITY    ACTIVE_HIGH

//在下面修改IO口配置
#define HAL_IO_CONFIG_INIT()                                     \
{                                                                \
  LED1_DDR |= LED1_BV;                                           \
  LED2_DDR |= LED2_BV;                                           \
  SCL_DDR |=  SCL_BV;   \
  HAL_TURN_OFF_LED1();                                           \
  HAL_TURN_OFF_LED2();                                           \
  SCL_SBIT = 0;\
  P1INP |=  BV(3);\
  P1INP |=  BV(4);\
}                                                                
/*

  LED1_DDR |= LED1_BV;                                           \
  LED2_DDR |= LED2_BV;                                           \
  SDA_DDR |=SDA_BV;   \
  HAL_TURN_OFF_LED1();                                           \
  HAL_TURN_OFF_LED2();                                           \
  AVDD_EN_DDR |= AVDD_EN_BV;                                        \
  AVDD_EN_SBIT =0;                                               \
  P0INP |=  BV(6);\
  P1INP |=  BV(3);\
  P1INP |=  BV(4);\
*/

#endif
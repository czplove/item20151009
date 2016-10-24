#ifndef WEIGHT_SCACLE_IO_CONFIG_H
#define WEIGHT_SCACLE_IO_CONFIG_H

//添加的输出
#define LED1_BV           BV(1)
#define LED1_SBIT         P1_1
#define LED1_DDR          P1DIR
#define LED1_POLARITY     ACTIVE_LOW	//20110721 ACTIVE_HIGH

#define LED2_BV           BV(6)
#define LED2_SBIT         P0_6
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


#define LED5_BV           BV(7)
#define LED5_SBIT         P1_7
#define LED5_DDR          P1DIR
#define LED5_POLARITY     ACTIVE_HIGH

#define SCL_BV           BV(6)
#define SCL_SBIT         P1_6
#define SCL_DDR          P1DIR
#define SCL_POLARITY     ACTIVE_HIGH


#define AVDD_EN_BV              BV(4)
#define AVDD_EN_SBIT            P0_4
#define AVDD_EN_DDR             P0DIR
#define AVDD_EN_POLARITY        ACTIVE_HIGH


#define HT1621_CS_BV              BV(1)
#define HT1621_CS_SBIT            P0_1
#define HT1621_CS_DDR             P0DIR
#define HT1621_CS_POLARITY        ACTIVE_HIGH


#define HT1621_DAT_BV              BV(2)
#define HT1621_DAT_SBIT            P0_2
#define HT1621_DAT_DDR             P0DIR
#define HT1621_DAT_POLARITY        ACTIVE_HIGH

#define HT1621_WR_BV              BV(3)
#define HT1621_WR_SBIT            P0_3
#define HT1621_WR_DDR             P0DIR
#define HT1621_WR_POLARITY        ACTIVE_HIGH

#define HT1621_RD_BV              BV(0)
#define HT1621_RD_SBIT            P0_0
#define HT1621_RD_DDR             P0DIR
#define HT1621_RD_POLARITY        ACTIVE_HIGH



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


#define SDA_BV           BV(5)
#define SDA_SBIT         P1_5
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
  HT1621_CS_DDR |=  HT1621_CS_BV;   \
  HT1621_DAT_DDR |=  HT1621_DAT_BV;   \
  HT1621_WR_DDR |=  HT1621_WR_BV;   \
  HT1621_RD_DDR |=  HT1621_RD_BV;   \
  HAL_TURN_OFF_LED1();                                           \
  HAL_TURN_OFF_LED2();                                           \
  SCL_SBIT = 0;\
  HT1621_CS_SBIT = 0;\
  HT1621_DAT_SBIT = 0;\
  HT1621_WR_SBIT = 0;\
  HT1621_RD_SBIT = 0;\
}                                                                
                                                             


#endif

#ifndef TOUCH_1_ID_IO_CONFIG_H
#define TOUCH_1_ID_IO_CONFIG_H

//��ӵ����
/*
#define LED1_BV           BV(0)
#define LED1_SBIT         P1_0
#define LED1_DDR          P1DIR
#define LED1_POLARITY     ACTIVE_HIGH	//20110721 ACTIVE_HIGH
*/

#define LED1_BV           BV(6)
#define LED1_SBIT         P0_6
#define LED1_DDR          P0DIR
#define LED1_POLARITY     ACTIVE_HIGH	//20110721 ACTIVE_HIGH

#define LED2_BV           BV(4)
#define LED2_SBIT         P0_4
#define LED2_DDR          P0DIR
#define LED2_POLARITY     ACTIVE_HIGH	//20110721 ACTIVE_HIGH


#define LED3_BV           BV(4)
#define LED3_SBIT         P0_4
#define LED3_DDR          P0DIR
#define LED3_POLARITY     ACTIVE_HIGH	//20110721 ACTIVE_HIGH


#define LED4_BV           BV(4)
#define LED4_SBIT         P0_4
#define LED4_DDR          P0DIR
#define LED4_POLARITY     ACTIVE_HIGH	//20110721 ACTIVE_HIGH


#define LED5_BV           BV(0)
#define LED5_SBIT         P2_0
#define LED5_DDR          P2DIR
#define LED5_POLARITY     ACTIVE_LOW

#define OUT1_BV           BV(0)            //�����л�
#define OUT1_SBIT         P0_0             
#define OUT1_DDR          P0DIR            
#define OUT1_POLARITY     ACTIVE_HIGH   

/*
#define PUSH1_BV          BV(2)
#define PUSH1_SBIT        P1_2
#define PUSH1_POLARITY    ACTIVE_HIGH
*/

// new
#define PUSH1_BV          BV(5)
#define PUSH1_SBIT        P1_5
#define PUSH1_POLARITY    ACTIVE_HIGH

/*
//old
#define PUSH1_BV          BV(2)
#define PUSH1_SBIT        P1_2
#define PUSH1_POLARITY    ACTIVE_HIGH
*/

#define PUSH2_BV          BV(6)
#define PUSH2_SBIT        P1_6
#define PUSH2_POLARITY    ACTIVE_HIGH

//new
#define PUSH3_BV          BV(2)
#define PUSH3_SBIT        P1_2
#define PUSH3_POLARITY    ACTIVE_HIGH

/*
//old
#define PUSH3_BV          BV(6)
#define PUSH3_SBIT        P1_6
#define PUSH3_POLARITY    ACTIVE_HIGH
*/

#define PUSH4_BV          BV(7)
#define PUSH4_SBIT        P1_7
#define PUSH4_POLARITY    ACTIVE_HIGH

#define PUSH5_BV          BV(7)
#define PUSH5_SBIT        P1_7
#define PUSH5_POLARITY    ACTIVE_HIGH

//�������޸�IO������
#define HAL_IO_CONFIG_INIT()                                     \
{                                                                \
  LED1_DDR |= LED1_BV;                                           \
  LED5_DDR |= LED5_BV;                                           \
  HAL_TURN_OFF_LED1();                                           \
  HAL_TURN_OFF_LED5();                                           \
  OUT1_DDR |= OUT1_BV;                                           \
  OUT1_SBIT = 0;\
}                                                                


#endif
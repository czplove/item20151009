#ifndef INSET_CURTAIN_CONTORL_IO_CONFIG_H
#define INSET_CURTAIN_CONTORL_IO_CONFIG_H

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


#define OUT1_BV           BV(0)            //����߿��ƶ�1
#define OUT1_SBIT         P0_0             
#define OUT1_DDR          P0DIR            
#define OUT1_POLARITY     ACTIVE_HIGH   

#define OUT2_BV           BV(1)            //���߿��ƶ�2
#define OUT2_SBIT         P0_1             
#define OUT2_DDR          P0DIR            
#define OUT2_POLARITY     ACTIVE_HIGH 



#define PUSH1_BV          BV(1)
#define PUSH1_SBIT        P0_1
#define PUSH1_POLARITY    ACTIVE_HIGH

//��ӵ����� 
#define PUSH2_BV          BV(5)
#define PUSH2_SBIT        P1_5
#define PUSH2_POLARITY    ACTIVE_HIGH

#define PUSH3_BV          BV(6)
#define PUSH3_SBIT        P1_6
#define PUSH3_POLARITY    ACTIVE_HIGH

#define PUSH4_BV          BV(2)
#define PUSH4_SBIT        P1_2
#define PUSH4_POLARITY    ACTIVE_HIGH

#define PUSH5_BV          BV(0)
#define PUSH5_SBIT        P1_0
#define PUSH5_POLARITY    ACTIVE_HIGH

#define PUSH6_BV          BV(3)
#define PUSH6_SBIT        P1_3
#define PUSH6_POLARITY    ACTIVE_HIGH

#define PUSH7_BV          BV(7)
#define PUSH7_SBIT        P1_7
#define PUSH7_POLARITY    ACTIVE_HIGH

//�������޸�IO������
#define HAL_IO_CONFIG_INIT()                                     \
{                                                                \
  LED1_DDR |= LED1_BV;                                           \
  LED2_DDR |= LED2_BV;                                           \
  HAL_TURN_OFF_LED1();                                           \
  HAL_TURN_OFF_LED2();                                           \
  OUT1_DDR |= OUT1_BV;                                           \
  OUT2_DDR |= OUT2_BV;                                           \
  OUT1_SBIT = 0;\
  OUT2_SBIT = 0;\
}                                                                


#endif
#ifndef SWITH_1_IO_CONFIG_H
#define SWITH_1_IO_CONFIG_H

//��ӵ����
#define LED1_BV           BV(0)
#define LED1_SBIT         P1_0
#define LED1_DDR          P1DIR
#define LED1_POLARITY     ACTIVE_LOW	//20110721 ACTIVE_HIGH

#define LED2_BV           BV(4)
#define LED2_SBIT         P1_4
#define LED2_DDR          P1DIR
#define LED2_POLARITY     ACTIVE_LOW


#define LED3_BV           BV(6)
#define LED3_SBIT         P0_6
#define LED3_DDR          P0DIR
#define LED3_POLARITY     ACTIVE_LOW

#define LED4_BV           BV(5)
#define LED4_SBIT         P1_5
#define LED4_DDR          P1DIR
#define LED4_POLARITY     ACTIVE_LOW

#define LED5_BV           BV(0)
#define LED5_SBIT         P2_0
#define LED5_DDR          P2DIR
#define LED5_POLARITY     ACTIVE_LOW

#define OUT1_BV           BV(1)            //�ڳ�ʼ��ʱ������λ���÷����  
#define OUT1_SBIT         P1_1             //�̵������ P1.3 �ն˰�
#define OUT1_DDR          P1DIR            //����Ĵ���
#define OUT1_POLARITY     ACTIVE_HIGH   


#define PUSH1_BV          BV(1)
#define PUSH1_SBIT        P0_1
#define PUSH1_POLARITY    ACTIVE_HIGH

//��ӵ����� 
#define PUSH3_BV          BV(2)
#define PUSH3_SBIT        P1_2
#define PUSH3_POLARITY    ACTIVE_HIGH

#define PUSH4_BV          BV(3)
#define PUSH4_SBIT        P1_3
#define PUSH4_POLARITY    ACTIVE_HIGH



//�������޸�IO������
#define HAL_IO_CONFIG_INIT()                                \
{                                                           \
  LED1_DDR |= LED1_BV;                                      \
  LED2_DDR |= LED2_BV;                                      \
  LED3_DDR |= LED3_BV;                                      \
  OUT1_DDR |= OUT1_BV;                                      \
  OUT1_SBIT=0;                                              \
  HAL_TURN_OFF_LED1();\
  HAL_TURN_OFF_LED2();\
  HAL_TURN_OFF_LED3();\
}                                                                


#endif
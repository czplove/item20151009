#ifndef ALARM_IO_CONFIG_H
#define ALARM_IO_CONFIG_H

//��ӵ����
#define LED1_BV           BV(0)
#define LED1_SBIT         P1_0
#define LED1_DDR          P1DIR
#define LED1_POLARITY     ACTIVE_LOW	//20110721 ACTIVE_HIGH

#define LED2_BV           BV(1)
#define LED2_SBIT         P1_1
#define LED2_DDR          P1DIR
#define LED2_POLARITY     ACTIVE_LOW


#define LED3_BV           BV(4)
#define LED3_SBIT         P1_4
#define LED3_DDR          P1DIR
#define LED3_POLARITY     ACTIVE_HIGH

#define OUT1_BV           BV(3)            //�ڳ�ʼ��ʱ������λ���÷����  
#define OUT1_SBIT         P1_3             //�̵������ P1.3 �ն˰�
#define OUT1_DDR          P1DIR            //����Ĵ���
#define OUT1_POLARITY     ACTIVE_HIGH   


#define PUSH1_BV          BV(1)
#define PUSH1_SBIT        P0_1


//��ӵ����� 
#define PUSH3_BV          BV(2)
#define PUSH3_SBIT        P1_2
#define PUSH3_POLARITY    ACTIVE_HIGH

#define PUSH4_BV          BV(4)
#define PUSH4_SBIT        P0_4
#define PUSH4_POLARITY    ACTIVE_HIGH

#define PUSH5_BV          BV(5)
#define PUSH5_SBIT        P1_5
#define PUSH5_POLARITY    ACTIVE_HIGH

#define PUSH6_BV          BV(5)
#define PUSH6_SBIT        P0_5
#define PUSH6_POLARITY    ACTIVE_HIGH

//�������޸�IO������
#define HAL_IO_CONFIG_INIT()                                     \
{                                                                \
  LED1_DDR |= LED1_BV;                                           \
  LED2_DDR |= LED2_BV;                                           \
  HAL_TURN_OFF_LED1();                                           \
  HAL_TURN_OFF_LED2();                                           \
/*  ��P0.4 P1.5 P0.5��Ϊ3̬״̬��������������� P0.5������ķŲ���*/  \
  P0INP |= PUSH4_BV;                                             \
  P1INP |= PUSH5_BV;                                             \
  P0INP |= PUSH5_BV;                                             \
}                                                                


#endif
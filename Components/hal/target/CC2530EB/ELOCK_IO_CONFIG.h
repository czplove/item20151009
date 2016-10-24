#ifndef ELOCK_IO_CONFIG_H
#define ELOCK_IO_CONFIG_H

//��ӵ����

//��ӵ����

#define LED1_BV           BV(6) //green
#define LED1_SBIT         P0_6
#define LED1_DDR          P0DIR
#define LED1_POLARITY     ACTIVE_LOW//ACTIVE_LOW	//20110721 ACTIVE_HIGH

#define LED2_BV           BV(5) //red
#define LED2_SBIT         P0_5
#define LED2_DDR          P0DIR
#define LED2_POLARITY     ACTIVE_HIGH//ACTIVE_HIGH//ACTIVE_LOW


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
#define LED5_POLARITY     ACTIVE_LOW//ACTIVE_HIGH

#define OUT1_BV           BV(1)            //�ڳ�ʼ��ʱ������λ���÷����  
#define OUT1_SBIT         P1_1             //�̵������ P1.3 �ն˰�
#define OUT1_DDR          P1DIR            //����Ĵ���
#define OUT1_POLARITY     ACTIVE_HIGH   

#define OUT2_BV           BV(0)            //�ڳ�ʼ��ʱ������λ���÷����  
#define OUT2_SBIT         P1_0             //�̵������ P1.3 �ն˰�
#define OUT2_DDR          P1DIR            //����Ĵ���
#define OUT2_POLARITY     ACTIVE_HIGH   
/*
#define OUT3_BV           BV(4)            //�ڳ�ʼ��ʱ������λ���÷����  
#define OUT3_SBIT         P1_4             //�̵������ P1.3 �ն˰�
#define OUT3_DDR          P1DIR            //����Ĵ���
#define OUT3_POLARITY     ACTIVE_HIGH   
*/

#define PUSH1_BV          BV(1)
#define PUSH1_SBIT        P0_1
#define PUSH1_POLARITY    ACTIVE_HIGH
/*
#define PUSH4_BV          BV(4)
#define PUSH4_SBIT        P0_4
#define PUSH4_DIR         P0DIR
#define PUSH4_POLARITY    ACTIVE_HIGH

*/
//��ӵ����� 
#define PUSH3_BV          BV(2)
#define PUSH3_SBIT        P1_2			//Կ��
#define PUSH3_POLARITY    ACTIVE_HIGH

#define PUSH4_BV          BV(4)			//����ͷ������
#define PUSH4_SBIT        P1_4
#define PUSH4_POLARITY    ACTIVE_HIGH

#define PUSH5_BV          BV(3)			//С��ͷ����ͷ
#define PUSH5_SBIT        P1_3
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
  OUT1_DDR |= OUT1_BV;                                           \
  OUT2_DDR |= OUT2_BV;                                           \
  OUT1_SBIT = 1;\
  OUT2_SBIT = 1;\
  P1DIR  &= ~0x20;\
  P1DIR  &= ~0x10;\
  P1DIR  &= ~0x08;\
  P1DIR  &= ~0x04;\
  P1INP |= 0x20;/*P1_5 ���������ã���Ϊ3̫���� */                                             \
/*  ��P0.4 P1.5 P0.5��Ϊ3̬״̬��������������� P0.5������ķŲ���*/  \
}                                                                
/*
  OUT3_DDR |= OUT3_BV;                                           \
     OUT3_SBIT = 0;\
P0INP |= PUSH4_BV;                                             \
  P1INP |= PUSH5_BV;                                             \
  P0INP |= PUSH5_BV;                                             \
*/

#endif
#ifndef WALL_SOKET_2_IO_CONFIG_H
#define WALL_SOKET_2_IO_CONFIG_H

//��ӵ����
#define LED1_BV           BV(3)
#define LED1_SBIT         P1_3
#define LED1_DDR          P1DIR
#define LED1_POLARITY     ACTIVE_LOW	//20110721 ACTIVE_HIGH

#define LED2_BV           BV(0)
#define LED2_SBIT         P1_0
#define LED2_DDR          P1DIR
#define LED2_POLARITY     ACTIVE_LOW


#define LED3_BV           BV(5)
#define LED3_SBIT         P1_5
#define LED3_DDR          P1DIR
#define LED3_POLARITY     ACTIVE_LOW

#define LED4_BV           BV(6)
#define LED4_SBIT         P0_6
#define LED4_DDR          P0DIR
#define LED4_POLARITY     ACTIVE_LOW

#define LED5_BV           BV(7)
#define LED5_SBIT         P1_7
#define LED5_DDR          P1DIR
#define LED5_POLARITY     ACTIVE_LOW

#define OUT1_BV           BV(6)            //�ڳ�ʼ��ʱ������λ���÷����  
#define OUT1_SBIT         P1_6             //�̵������ P1.3 �ն˰�
#define OUT1_DDR          P1DIR            //����Ĵ���
#define OUT1_POLARITY     ACTIVE_HIGH   

#define OUT2_BV           BV(5)            //�ڳ�ʼ��ʱ������λ���÷����  
#define OUT2_SBIT         P0_5             //�̵������ P1.3 �ն˰�
#define OUT2_DDR          P0DIR            //����Ĵ���
#define OUT2_POLARITY     ACTIVE_HIGH  

#define PUSH1_BV          BV(1)
#define PUSH1_SBIT        P0_1
#define PUSH1_POLARITY    ACTIVE_HIGH

//��ӵ����� 
#define PUSH3_BV          BV(2)
#define PUSH3_SBIT        P1_2
#define PUSH3_POLARITY    ACTIVE_HIGH

#define PUSH4_BV          BV(4)
#define PUSH4_SBIT        P0_4
#define PUSH4_POLARITY    ACTIVE_HIGH



//�������޸�IO������
#define HAL_IO_CONFIG_INIT()                                \
{                                                           \
  LED1_DDR |= LED1_BV;                                      \
  LED2_DDR |= LED2_BV;                                      \
  LED3_DDR |= LED3_BV;                                      \
  LED4_DDR |= LED4_BV;                                      \
  OUT1_DDR |= OUT1_BV;                                      \
  OUT2_DDR |= OUT2_BV;                                      \
  OUT1_SBIT=0;                                              \
  OUT2_SBIT=0;                                              \
}                                                                


#endif
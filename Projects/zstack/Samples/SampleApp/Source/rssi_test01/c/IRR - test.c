/*******************************************************************************
*
*����������TIMER3�ж�200��LED��˸һ��
*
*���ߣ�Rfstorm studio QQȺ��51109148
*
*ʱ�䣺2010-12-7
*
********************************************************************************/

//#include "T3config.h"
//#include "T4config.h"
#include "hal_mcu.h"
#include "hal_board_cfg.h"
#include "ZComDef.h"
#include "OSAL_Nv.h"
#include "hal_led.h"

#include "irr.h"
#include "m25p16.h"


#define uint unsigned int
#define uchar unsigned char




//void Initial(void);

/*****************************************
//����ȫ�ֱ���
*****************************************/
//uchar counter = 0;
//uint16 pluseWide[127];
//uint16 pluseWide[127];
//uint16 rEdgeTimeSlot[100];  //������ʱ����
//uint16 fEdgeTimeSlot[100];   //�½���ʱ����

//uint16 EdgeTimeSlot;   
//uint16 EdgeTimeSlot_LR[500];   //����ʱ����= EdgeTimeSlot_LRsl * us10Cnt + EdgeTimeSlot_HRsl
//uint16 EdgeTimeSlot_LRsl = 0;   //����ʱ���� �ͷֱ���  10us

//uint8  EdgeTimeSlot_HR[500];   //����ʱ���� �߷ֱ���  0.125us
//uint8  EdgeTimeSlot_HRsl = 0;   //����ʱ���� �߷ֱ���  0.125us

/*
volatile uint16 ledBlinkCnt = 0;        //

volatile uint16 us10Cnt = 0;        //t4 ��ʱ��ʱ϶
volatile uint16  edgeCnt = 0;       //���ؼ���
volatile uint16  edgeCnt_temp = 0;  //���ؼ���

uint16  edgeNum = 0;
uint8 singleEdgeEndFlag = 0;

uint8  m0 = 0;
uint8  NV_ACType = '0'; //������NV�еĿյ�����ʼ��ʱΪ'0','1'��A�࣬'2'��B��
uint8  NV_CTType = '0'; //������NV�еĿյ�����ʼ��ʱΪ'0',

//uint8  edgeCntPre = 255;       //���ؼ���
uint8  callIrRxFunFlag = 0; //���ú��������պ�����־
uint8  callIrTxFunFlag = 0; //���ú�����뷢�ͺ�����־
//uint8 i;
volatile uint8 irCodeTxFlag = 0;
volatile uint8 irCodeRxFlag = 0;
*/

void irCodeRx4Test(void);
void irCodeTx4Test(void);
void flashTest(void);
//void keyCheck();
//void irCodeRx();
//void irCodeTx();
/****************************
//��ʼ������
*************************/
/*void delay(uint16 i)
{
 uint16 x,y;
for(x = 0;x < i;x++)
  for(y = 0; y<0xffff;y++);
  
}
*/



void irCodeRx4Test(void)
{
  uint8 i;
        uint16 ledBlink100msCnt = 0;
	uint8  ms100Cnt=0;//ѧϰ��ʱ��ʱ��
	
	P1_5 = 1;//open the power for receive pin 
	Delay_1u(65000);//ѧϰʱ�ܿ����չ��ϵ�˲�����ŵ�ƽ��ת��Ӱ��
	
	irCodeRxFlag = 1;
	edgeCnt = 0;
    T3CCTL1 = 0x43;     //T3 cha1 config as capture mode;both edges 
    TIMER3_START(1);       //����T3,
    TIMER34_START(4,1);    //����T4,
    while(1)
	{
		if(!edgeCnt)
		{
			//�ޱ��أ���û���롣��10s�ȴ�����
			if(10000 == us10Cnt) //100ms
			{
				//ms100Cnt++;
				us10Cnt = 0;
                                ledBlink100msCnt++;
			}
                        /*
			if(100 == ms100Cnt)//10s
			{
				TIMER34_START(4,0);    //�ر�T4,
				TIMER3_START(0);       //�ر�T3,
				//T3CCTL0 = 0;     	 
				T3CCTL1 = 0;
				irCodeRxFlag = 0;
				ms100Cnt = 0;
				P1_5 = 0;//close  the power for low power model
				//m25p16_deepPowerDown();
				//powerDown();
                                P1_1 = 1;//off led
                                
				return;
			}*/
                        if(ledBlink100msCnt == 5)//500ms
                        {
                          P1_1 = ~P1_1;
                          ledBlink100msCnt = 0;
                        }
		}
		else
		{
                        P1_1 = 0;//on led
			//�Ѿ���׽������
			if(60000 < us10Cnt) //600ms �ղ������ݣ������������ݣ�ѧϰ����
			{
				TIMER34_START(4,0);    //�ر�T4,
				TIMER3_START(0);       //�ر�T3,
				//T3CCTL0 = 0;     	 
				T3CCTL1 = 0;
				us10Cnt = 0;
				edgeNum = edgeCnt;
				edgeCnt_temp = 0;
				edgeCnt = 0;

                                                            
                                for(i= 1 ;i<10;i++)
                                {
                                  if((180< EdgeTimeSlot_LR[i]) && (EdgeTimeSlot_LR[i]<220))
                                  {
                                    
                                  }
                                  else
                                  {
                                    //ir fail
                                    P1_1 = 0;//off led			

                                    while(1);
                                  }
                                
                                }
                    
                                
				irCodeRxFlag = 0;
				P1_5 = 0;//close  the power for low power model
                                //HalLedBlink ( HAL_LED_2, 3, 50, 200 );//��BLINKģ����������Ϩ��
                                P1_1 = 1;//off led			
                                return;
			}
			else
			{
				//��������б��ؼ����ʱ��ֵ��NV
				if(edgeCnt_temp != edgeCnt)
				{
					//���� EdgeTimeSlot �� EdgeTimeSlot_L ��NV
					EdgeTimeSlot_LR[edgeCnt_temp] = EdgeTimeSlot_LRsl;
					edgeCnt_temp = edgeCnt;
					
				}
			}
		}
	}
}

//�����ݷ���ȥ
void irCodeTx4Test(void)
{
    uint16 i;

    for(i= 0 ;i<10;i++)
    {
      EdgeTimeSlot_LR[i] = 200;
      EdgeTimeSlot_HR[i] = 5;
    }
    edgeNum = 10;
    T3CCTL0 = 0x04;     //0x14--�Ƚ������0->104 ʱ��ת�Ƚ�������� ����38K�ز�
						//0x04 �Ƚ�������� ����ߵ�ƽ
    TIMER3_START(1);       //����T3,
    TIMER34_START(4,1);    //����T4,
	for(edgeCnt = 0; edgeCnt < edgeNum; edgeCnt++)
	{
		us10Cnt = EdgeTimeSlot_LR[edgeCnt];
		if(edgeCnt % 2)
		{
			T3CCTL0 = 0x14;		//0x14--�Ƚ������0->105->0 ʱ��ת�Ƚ�������� 
			//0x04 �Ƚ�������� ����ߵ�ƽ
		}
		else
		{
			T3CCTL0 = 0x04;		//0x14--�Ƚ������0->105->0 ʱ��ת�Ƚ�������� 
			//0x04 �Ƚ�������� ����ߵ�ƽ
		}
		//T4CTL |= 0x04;    //����counter
		//read us10Cnt from NV
		
		//while(us10Cnt);
		//T4CTL |= 0x04;    //����counter
		//T4CC0 = 100 - EdgeTimeSlot_L[i];
		//EdgeTimeSlot_HRsl = EdgeTimeSlot_HR[i];
		singleEdgeEndFlag = 0;
                P1_1 = ~P1_1;
		while(!singleEdgeEndFlag);
		
	}
	TIMER34_START(4,0);    //�ر�T4,
	TIMER3_START(0);       //�ر�T3,
	T3CCTL0 = 0;		//0x14--�Ƚ������0->105 ʱ��ת�Ƚ��������
	irCodeTxFlag = 0;
        P1_1 = 1;//off led
}

//����flash��д
void flashTest(void)
{
  uint8 m25p16_wrBuff[20] = {1,2,3,4,5,6,7,8,9,10};
  uint8 i;
  //�ж�ID�Ƿ���ȷ
  Hal_SPI_Init();
  m25p16_wakeUP(); //����m25p16��
  m25p16_beWIP();
  m25p16_readID();
  if((m25p16_ID[0] == 0)&&
    (m25p16_ID[1] == 0x20)&&
      (m25p16_ID[2] == 0x20)&&
        (m25p16_ID[3] == 0x15))
  {

  }
  else
  {
    //ID����
    P1_1 = 0;//�����
    while(1);
  }
  //��д�����Ƿ�OK
  m25p16_beWIP();
  m25p16_writeEnable();
  m25p16_sectorErase(0,0,0);//������0����
  
  m25p16_beWIP();
  m25p16_writeEnable();
  m25p16_pageProgram(0,0,0x00,(uint8 *)&(m25p16_wrBuff),10);//д0���� 0ҳ ��0�ֽ� ��д10��
  
  //clear buff
  for(i = 0;i < 10;i++)
    m25p16_wrBuff[i] = 0;
  
  //����д�������
  m25p16_beWIP();
  m25p16_readData(0,0,0x00,(uint8 *)&(m25p16_wrBuff),10);
  
  //�Ա�����
  for(i = 0;i < 10;i++)
  {
    if(m25p16_wrBuff[i] == i + 1)
    {
      
    }
    else
    {
      //flash fail
      P1_1 = 0;//�����
      while(1);
    }
  }
  
//��дOK����flash  
  //�����ܸ�д��ֻ�ܲ�����������0����
  m25p16_beWIP();
  m25p16_writeEnable();
  m25p16_sectorErase(0,0,0);//������0����
  
  m25p16_beWIP();
  m25p16_readData(0,0,0x00,(uint8 *)&(m25p16_wrBuff),20);//���10�����жϣ����Ƿ񱻲���
  
  for(i = 0;i < 20;i++)
  {
    if(m25p16_wrBuff[i] == 0xFF)
    {
      
    }
    else
    {
      //flash fail
      P1_1 = 0;//�����
      while(1);
    }
  }
  //pass
  m25p16_beWIP();
  m25p16_deepPowerDown();//m25p16˯��
  powerDown();//2530 ����ر�
  
}

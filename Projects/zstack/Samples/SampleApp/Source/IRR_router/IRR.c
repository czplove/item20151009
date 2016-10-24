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
uchar counter = 0;
//uint16 pluseWide[127];
//uint16 pluseWide[127];
//uint16 rEdgeTimeSlot[100];  //������ʱ����
//uint16 fEdgeTimeSlot[100];   //�½���ʱ����

uint16 EdgeTimeSlot;   
uint16 EdgeTimeSlot_LR[500];   //����ʱ����= EdgeTimeSlot_LRsl * us10Cnt + EdgeTimeSlot_HRsl
uint16 EdgeTimeSlot_LRsl = 0;   //����ʱ���� �ͷֱ���  10us

uint8  EdgeTimeSlot_HR[500];   //����ʱ���� �߷ֱ���  0.125us
uint8  EdgeTimeSlot_HRsl = 0;   //����ʱ���� �߷ֱ���  0.125us

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


void irrIOInit(void)
{
	
    P1SEL |= 0x10;//p1.4 ���Ⲷ׽ peripheral
    P1SEL |= 0x08;//p1.3 ���ⷢ�� peripheral

	//P1SEL |= 0x01;//p1.0 ���� peripheral T4 ch0

    P1DIR |= 0X09;//p1.0��p1.3 OUTPUT
    P1DIR |= 0X02;//p1.1 OUTPUT

    P1DIR |= 0X20;//p1.5 ����POWERctrl OUTPUT

    P1INP |= 0x10;//P1.4 ������գ���Ϊ������̬

	LED1_SBIT = 1;
	LED2_SBIT = 1;
	P1_5 = 0;
}

void irrInit(void)
{
  irrIOInit();
  Hal_SPI_Init();
  m25p16_deepPowerDown(); //m25p16˯��
  powerDown();//2530 ����ر�
  Init_T3();
  Init_T4();
}


void irCodeRx(uint16 IRCodeIndex)
{
        uint16 ledBlink100msCnt = 0;
	uint8  ms100Cnt=0;//ѧϰ��ʱ��ʱ��
	//uint16 EdgeTimeSlotCnt = 0;
	uint8 SAddr,PAddr,BAddr,CAPF_BAddr,CAPF_Byte,CAPF_Bit;//������ҳ���ֽڵ�ַ
	uint8 pageNum,byteNum,pageCnt;
	//uint16 IRCodeIndex = 0;//������������0~1023 ��1024��������롣
	if(IRCodeIndex > 629)//ֻ��0~629 �ű���ѧϰ
		return;
	
	P1_5 = 1;//open the power for receive pin 
	Delay_1u(65000);//ѧϰʱ�ܿ����չ��ϵ�˲�����ŵ�ƽ��ת��Ӱ��
	SAddr = (IRCodeIndex / 21);//ÿ���������21��256page/12page��21.333��ȡ21�������롣һ������ռ3000�ֽڣ�Ϊ�����12��ҳ��3072�ֽڣ� 

	//m25p16_beWIP();
	//m25p16_writeEnable();
	//m25p16_sectorErase(0,0,0);				//������m25p16_SectorCnt+1����
	
	irCodeRxFlag = 1;
	edgeCnt = 0;
    //P1SEL &= ~(0x08);//p1.3 GPIO  
    //T3CCTL0 = 0x44;		//0x44--�Ƚ������modulo,CH0 interrupt,���ڱ��ؼ�ʱ 
	//0x04 �Ƚ�������� ����ߵ�ƽ
    T3CCTL1 = 0x43;     //T3 cha1 config as capture mode;both edges 
    TIMER3_START(1);       //����T3,
    TIMER34_START(4,1);    //����T4,
	//HAL_TURN_ON_LED2();//on led
    while(1)
	{
		if(!edgeCnt)
		{
			//�ޱ��أ���û���롣��10s�ȴ�����
			if(10000 == us10Cnt) //100ms
			{
				ms100Cnt++;
				us10Cnt = 0;
                                ledBlink100msCnt++;
			}
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
                                LED2_SBIT = 1;//off led
                                                               
				return;
			}
                        if(ledBlink100msCnt == 5)//500ms
                        {
                          LED2_SBIT = ~LED2_SBIT;
                          ledBlink100msCnt = 0;
                        }
		}
		else
		{
                        LED2_SBIT = 0;//on led
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
				
				Hal_SPI_Init();
				m25p16_wakeUP(); //����m25p16��
				
				//��ȡ����ı�̱�־
				m25p16_beWIP();
				m25p16_readData(30,IRCodeIndex / 256,IRCodeIndex % 256,(uint8 *)&CAPF_Byte,1);
				//CAPF_Bit = IRCodeIndex % 8;
				//if(CAPF_Byte & (1 << CAPF_Bit))
				if(0xFF == CAPF_Byte)
				{
					//�ñ�������δ��̣�����ֱ��д���������
					PAddr = (IRCodeIndex % 21) * 12 + 11;
					BAddr = 254;
					m25p16_beWIP();
					m25p16_writeEnable();
					m25p16_pageProgram(SAddr,PAddr,BAddr,(uint8 *)&(edgeNum),2);
					//���� EdgeTimeSlot �� m25p16
					for(edgeCnt = 0; edgeCnt < edgeNum; edgeCnt++)
					{
						PAddr = (IRCodeIndex % 21) * 12 + (edgeCnt*3) / 255;
						BAddr = (edgeCnt*3) % 255;
						
						m25p16_beWIP();
						m25p16_writeEnable();
						m25p16_pageProgram(SAddr,PAddr,BAddr,(uint8 *)&(EdgeTimeSlot_LR[edgeCnt]),2);
						
						PAddr = (IRCodeIndex % 21) * 12 + (edgeCnt*3+2) / 255;
						BAddr = (edgeCnt*3+2) % 255;
						m25p16_beWIP();
						m25p16_writeEnable();
						m25p16_pageProgram(SAddr,PAddr,BAddr,(uint8 *)&(EdgeTimeSlot_HR[edgeCnt]),1);
					}
					//���ݱ����Ժ���Ҫ��д��̱�־λ
					//CAPF_Byte &= ~(1 << CAPF_Bit);
					CAPF_Byte = 0;
					m25p16_writeEnable();
					m25p16_pageProgram(30,IRCodeIndex / 256,IRCodeIndex % 256,(uint8 *)&CAPF_Byte,1);
                                        

				}
				else
				{
					//�ñ��������Ѿ���̣�����ֱ��д�����ݣ�ֻ���޸ı������ݣ��ȵ���ͬһ�������ڲ��߱������ݣ��ٲ�������������д��������
					m25p16_pageModify(SAddr,
									  (IRCodeIndex % 21) * 12,
									  (IRCodeIndex % 21) * 12 + 12,
									  EdgeTimeSlot_LR,
									  EdgeTimeSlot_HR,
									  edgeNum);
				}
				irCodeRxFlag = 0;
				P1_5 = 0;//close  the power for low power model
                                m25p16_beWIP();
				m25p16_deepPowerDown();//m25p16˯��
				powerDown();//2530 ����ر�
                                HalLedBlink ( HAL_LED_2, 3, 50, 200 );//��BLINKģ����������Ϩ��
			
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

void irCodeTx(uint16 IRCodeIndex)
{
	uint16 i;
	uint8 SAddr,PAddr,BAddr;//������ҳ���ֽڵ�ַ
	uint8 pageNum,byteNum,pageCnt;
	//uint16 IRCodeIndex = 0;//������������0~1023 ��1024��������롣
	Hal_SPI_Init();
	m25p16_wakeUP();
	irCodeTxFlag = 1;
	//	T3CTL |= 0x10;    //����T3,��ʼ��ʱ
	//	T4CTL |= 0x10;    //����T4,��ʼ��ʱ������5s�����������ѧϰ
	//	T4CTL |= 0x08;    //���ж�
	SAddr = (IRCodeIndex / 21);//ÿ���������21��256page/12page��21.333��ȡ21�������롣һ������ռ3000�ֽڣ�Ϊ�����12��ҳ��3072�ֽڣ� 
	PAddr = (IRCodeIndex % 21) * 12 + 11;
	BAddr = 254;
	m25p16_beWIP();
	m25p16_readData(SAddr,PAddr,BAddr,(uint8 *)&(edgeNum),2);
	if(edgeNum > 799)//���볤��Ϊ800������
		return;
	for(edgeCnt = 0; edgeCnt < edgeNum; edgeCnt++)
	{
		PAddr = (IRCodeIndex % 21) * 12 + (edgeCnt*3) / 255;
		BAddr = (edgeCnt*3) % 255;
		
		m25p16_beWIP();
		//m25p16_writeEnable();
		m25p16_readData(SAddr,PAddr,BAddr,(uint8 *)&(EdgeTimeSlot_LR[edgeCnt]),2);
		
		PAddr = (IRCodeIndex % 21) * 12 + (edgeCnt*3+2) / 255;
		BAddr = (edgeCnt*3+2) % 255;
		m25p16_beWIP();
		//m25p16_writeEnable();
		m25p16_readData(SAddr,PAddr,BAddr,(uint8 *)&(EdgeTimeSlot_HR[edgeCnt]),1);
		
	}
	
	//us10Cnt = EdgeTimeSlot[0];//װ�ر��صĳ�ʼֵ����ֵ��ʵ��Ч��
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
                LED2_SBIT = ~LED2_SBIT;
		while(!singleEdgeEndFlag);
		
	}
	TIMER34_START(4,0);    //�ر�T4,
	TIMER3_START(0);       //�ر�T3,
	T3CCTL0 = 0;		//0x14--�Ƚ������0->105 ʱ��ת�Ƚ��������
	m25p16_deepPowerDown();//m25p16˯��
	powerDown();  //2530 ����ر�
        LED2_SBIT = 1;//off led
	irCodeTxFlag = 0;
}

uint8 StudyIRCode(uint8 Type,uint8 KeyCode,uint8 TempL,uint8 TempH)
{
	
}

uint8 ControlAC(uint8 Type,uint8 Statue,uint8 KeyCode,uint8 Temp)
{
	
}


/*
void Delay_1u(uint32 microSecs) 
{
  while(microSecs--)
  {
    //32 NOPs == 1 usecs 
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop");
  }
}
*/




void Hal_SPI_Init(void)
{
  P0SEL |=0x04;		// P0_2 are peripherals,SPI's MI
  P0SEL |=0x08;		// P0_3 are peripherals,SPI's MO
  //P0SEL &= ~0x10;	// P0_5 are GPIO,SPI's SS
  P0SEL |=0x20;		// P0_5 are peripherals,SPI's CS
  P0DIR |= 0x10; //m25p16 cs

  //������������λ��
  //PERCFG &= ~0x01; //USART 0 I/O location  Alternative 1 location  
  
  //�������ȼ���
  //P2DIR &=  ~0xC0;//Port 0 peripheral priority control,1st priority: USART 0

  U0CSR &= 0x7f;	//����Ϊspiģʽ
  U0CSR  = 0x40;	//����Ϊspi����,�������
  U0BAUD = 0xFF;//216;
  U0GCR  = 0x30;//0x30;	//�������A.�����ʵķ���ֵ,B.LSB����MSB,C.ʱ����λ���Ǽ���
  //����ʱ��Ϊ11059200
}


void powerDown(void)
{
  U0CSR  = 0;	
  U0CSR  = 0;	
  U0BAUD = 0;	
  U0GCR  = 0;	

  P0SEL &= ~0x04;		// P0_2 are peripherals,SPI's MI
  P0SEL &= ~0x08;		// P0_3 are peripherals,SPI's MO
  P0SEL &= ~0x20;		// P0_5 are peripherals,SPI's CS

  P0DIR &= ~0x04;
  P0DIR &= ~0x08;
  P0DIR &= ~0x10;
  P0DIR &= ~0x20;
}




/*****************************************
//T3��ʼ��
*****************************************/
void Init_T3(void)
{
    TIMER34_INIT(3);                  //��ʼ��T3
    TIMER34_ENABLE_OVERFLOW_INT(3,1);  //��T3�ж�
    EA = 1;
    T3IE = 1;

    //T3CTL |= 0XA0;                    //ʱ��32��Ƶ101
    TIMER3_SET_CLOCK_DIVIDE(4);			//32m����ƵΪ16M����T3��ʱ��Դ
    TIMER3_SET_MODE(2);                 //up-down ģʽ 0��>105->0 ������
    T3CC0 = 104;		//16000000 / (105+105) / 2 =38095Hz ��38KHz �����ز�
						//��1/16MHz��*210=13.125uS ��������жϣ����ڲ�׽ʱ��ʱ��
	//T3CCTL0 = 0x14;
    T3CCTL0 = 0;//0x14;		//0x14--�Ƚ������0->104 ʱ��ת�Ƚ�������� 
						//0x04 �Ƚ�������� ����ߵ�ƽ
    T3CCTL1 = 0;//0x43;     //T3 cha1 config as capture mode;both edges 
						//=0x43 T3ͨ��1�жϿ�
    TIMER3_START(0);    //������T3,��׽��T3�ļ������Ƕ�����
}



/*****************************************
//T4��ʼ��
*****************************************/
void Init_T4(void)
{
    TIMER34_INIT(4);                  //��ʼ��T4
    TIMER34_ENABLE_OVERFLOW_INT(4,1);  //��T4�ж�
    EA = 1;
    T4IE = 1;

    //T4CTL |= 0XA0;                    //ʱ��32��Ƶ101
    TIMER34_SET_CLOCK_DIVIDE(4,4);
    TIMER34_SET_MODE(4,2);                 //�Զ���װ00��>T4CC0,mode1 mode3 can generated interreput

    T4CCTL0 = 0x54;		//0x14--�Ƚ������0->105->0 ʱ��ת�Ƚ�������� 
						//0x04 �Ƚ�������� ����ߵ�ƽ
//    T4CCTL1 = 0x03;     //T3 cha1 config as capture mode;both edges 
						//=0x43 T3ͨ��1�жϿ�
    T4CC0 = 79;                          //timer4 �жϷֱ���Ϊ100us����ʱʱ��Ϊ1us
    IP0 = 0x10;                             //set timer4 priority higher than timer3
    TIMER34_START(4,0);                    //����
}


/*
#pragma vector = T3_VECTOR
 __interrupt void T3_ISR(void)
 {
  if(!edgeCnt)
  {
    TIMER34_START(4,1);                    //����
  }
  EdgeTimeSlot[edgeCnt] = usCnt;
  usCnt = 0;
  edgeCntPre = edgeCnt;
  edgeCnt++;
 }

#pragma vector = T4_VECTOR
        
 __interrupt void T4_ISR(void)
 {
   if(irCodeRxFlag)
   usCnt++;
   if(irCodeTxFlag)
   if(usCnt)usCnt--;
 }

*/ 
 
 
/*******************************************************************************
*
*功能描述：TIMER3中断200次LED闪烁一次
*
*作者：Rfstorm studio QQ群：51109148
*
*时间：2010-12-7
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
//定义全局变量
*****************************************/
uchar counter = 0;
//uint16 pluseWide[127];
//uint16 pluseWide[127];
//uint16 rEdgeTimeSlot[100];  //上升沿时间间隔
//uint16 fEdgeTimeSlot[100];   //下降沿时间间隔

uint16 EdgeTimeSlot;   
uint16 EdgeTimeSlot_LR[500];   //边沿时间间隔= EdgeTimeSlot_LRsl * us10Cnt + EdgeTimeSlot_HRsl
uint16 EdgeTimeSlot_LRsl = 0;   //边沿时间间隔 低分辨率  10us

uint8  EdgeTimeSlot_HR[500];   //边沿时间间隔 高分辨率  0.125us
uint8  EdgeTimeSlot_HRsl = 0;   //边沿时间间隔 高分辨率  0.125us

volatile uint16 ledBlinkCnt = 0;        //

volatile uint16 us10Cnt = 0;        //t4 计时器时隙
volatile uint16  edgeCnt = 0;       //边沿计数
volatile uint16  edgeCnt_temp = 0;  //边沿计数

uint16  edgeNum = 0;
uint8 singleEdgeEndFlag = 0;

uint8  m0 = 0;
uint8  NV_ACType = '0'; //保存在NV中的空调类别初始化时为'0','1'是A类，'2'是B类
uint8  NV_CTType = '0'; //保存在NV中的空调类别初始化时为'0',

//uint8  edgeCntPre = 255;       //边沿计数
uint8  callIrRxFunFlag = 0; //调用红外编码接收函数标志
uint8  callIrTxFunFlag = 0; //调用红外编码发送函数标志
//uint8 i;
volatile uint8 irCodeTxFlag = 0;
volatile uint8 irCodeRxFlag = 0;
//void keyCheck();
//void irCodeRx();
//void irCodeTx();
/****************************
//初始化程序
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
	
    P1SEL |= 0x10;//p1.4 红外捕捉 peripheral
    P1SEL |= 0x08;//p1.3 红外发射 peripheral

	//P1SEL |= 0x01;//p1.0 测试 peripheral T4 ch0

    P1DIR |= 0X09;//p1.0，p1.3 OUTPUT
    P1DIR |= 0X02;//p1.1 OUTPUT

    P1DIR |= 0X20;//p1.5 红外POWERctrl OUTPUT

    P1INP |= 0x10;//P1.4 红外接收，设为输入三态

	LED1_SBIT = 1;
	LED2_SBIT = 1;
	P1_5 = 0;
}

void irrInit(void)
{
  irrIOInit();
  Hal_SPI_Init();
  m25p16_deepPowerDown(); //m25p16睡眠
  powerDown();//2530 外设关闭
  Init_T3();
  Init_T4();
}


void irCodeRx(uint16 IRCodeIndex)
{
        uint16 ledBlink100msCnt = 0;
	uint8  ms100Cnt=0;//学习超时计时用
	//uint16 EdgeTimeSlotCnt = 0;
	uint8 SAddr,PAddr,BAddr,CAPF_BAddr,CAPF_Byte,CAPF_Bit;//扇区，页，字节地址
	uint8 pageNum,byteNum,pageCnt;
	//uint16 IRCodeIndex = 0;//红外编码的索引0~1023 共1024个红外编码。
	if(IRCodeIndex > 629)//只对0~629 号编码学习
		return;
	
	P1_5 = 1;//open the power for receive pin 
	Delay_1u(65000);//学习时避开接收管上电瞬间引脚电平翻转的影响
	SAddr = (IRCodeIndex / 21);//每个扇区存放21（256page/12page≈21.333，取21）个编码。一个编码占3000字节（为其分配12个页共3072字节） 

	//m25p16_beWIP();
	//m25p16_writeEnable();
	//m25p16_sectorErase(0,0,0);				//擦除第m25p16_SectorCnt+1扇区
	
	irCodeRxFlag = 1;
	edgeCnt = 0;
    //P1SEL &= ~(0x08);//p1.3 GPIO  
    //T3CCTL0 = 0x44;		//0x44--比较输出，modulo,CH0 interrupt,用于边沿计时 
	//0x04 比较输出引脚 输出高电平
    T3CCTL1 = 0x43;     //T3 cha1 config as capture mode;both edges 
    TIMER3_START(1);       //启动T3,
    TIMER34_START(4,1);    //启动T4,
	//HAL_TURN_ON_LED2();//on led
    while(1)
	{
		if(!edgeCnt)
		{
			//无边沿，即没编码。作10s等待处理
			if(10000 == us10Cnt) //100ms
			{
				ms100Cnt++;
				us10Cnt = 0;
                                ledBlink100msCnt++;
			}
			if(100 == ms100Cnt)//10s
			{
                                TIMER34_START(4,0);    //关闭T4,
				TIMER3_START(0);       //关闭T3,
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
			//已经捕捉到边沿
			if(60000 < us10Cnt) //600ms 收不到数据，或者无新数据，学习结束
			{
				TIMER34_START(4,0);    //关闭T4,
				TIMER3_START(0);       //关闭T3,
				//T3CCTL0 = 0;     	 
				T3CCTL1 = 0;
				us10Cnt = 0;
				edgeNum = edgeCnt;
				edgeCnt_temp = 0;
				edgeCnt = 0;
				
				Hal_SPI_Init();
				m25p16_wakeUP(); //唤醒m25p16，
				
				//读取编码的编程标志
				m25p16_beWIP();
				m25p16_readData(30,IRCodeIndex / 256,IRCodeIndex % 256,(uint8 *)&CAPF_Byte,1);
				//CAPF_Bit = IRCodeIndex % 8;
				//if(CAPF_Byte & (1 << CAPF_Bit))
				if(0xFF == CAPF_Byte)
				{
					//该编码区域未编程，可以直接写入编码数据
					PAddr = (IRCodeIndex % 21) * 12 + 11;
					BAddr = 254;
					m25p16_beWIP();
					m25p16_writeEnable();
					m25p16_pageProgram(SAddr,PAddr,BAddr,(uint8 *)&(edgeNum),2);
					//保存 EdgeTimeSlot 至 m25p16
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
					//数据保存以后需要改写编程标志位
					//CAPF_Byte &= ~(1 << CAPF_Bit);
					CAPF_Byte = 0;
					m25p16_writeEnable();
					m25p16_pageProgram(30,IRCodeIndex / 256,IRCodeIndex % 256,(uint8 *)&CAPF_Byte,1);
                                        

				}
				else
				{
					//该编码区域已经编程，不能直接写入数据，只能修改编码数据，先导出同一个扇区内不边编码数据，再擦除扇区，接着写入新数据
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
				m25p16_deepPowerDown();//m25p16睡眠
				powerDown();//2530 外设关闭
                                HalLedBlink ( HAL_LED_2, 3, 50, 200 );//用BLINK模拟灯亮两秒后熄灭
			
                                return;
			}
			else
			{
				//保存编码中边沿间隔的时间值至NV
				if(edgeCnt_temp != edgeCnt)
				{
					//保存 EdgeTimeSlot 和 EdgeTimeSlot_L 至NV
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
	uint8 SAddr,PAddr,BAddr;//扇区，页，字节地址
	uint8 pageNum,byteNum,pageCnt;
	//uint16 IRCodeIndex = 0;//红外编码的索引0~1023 共1024个红外编码。
	Hal_SPI_Init();
	m25p16_wakeUP();
	irCodeTxFlag = 1;
	//	T3CTL |= 0x10;    //启动T3,开始计时
	//	T4CTL |= 0x10;    //启动T4,开始计时，超过5s，无码则结束学习
	//	T4CTL |= 0x08;    //开中断
	SAddr = (IRCodeIndex / 21);//每个扇区存放21（256page/12page≈21.333，取21）个编码。一个编码占3000字节（为其分配12个页共3072字节） 
	PAddr = (IRCodeIndex % 21) * 12 + 11;
	BAddr = 254;
	m25p16_beWIP();
	m25p16_readData(SAddr,PAddr,BAddr,(uint8 *)&(edgeNum),2);
	if(edgeNum > 799)//编码长度为800个边沿
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
	
	//us10Cnt = EdgeTimeSlot[0];//装载边沿的初始值，此值无实际效果
    T3CCTL0 = 0x04;     //0x14--比较输出，0->104 时翻转比较输出引脚 产生38K载波
						//0x04 比较输出引脚 输出高电平
    TIMER3_START(1);       //启动T3,
    TIMER34_START(4,1);    //启动T4,
	for(edgeCnt = 0; edgeCnt < edgeNum; edgeCnt++)
	{
		us10Cnt = EdgeTimeSlot_LR[edgeCnt];
		if(edgeCnt % 2)
		{
			T3CCTL0 = 0x14;		//0x14--比较输出，0->105->0 时翻转比较输出引脚 
			//0x04 比较输出引脚 输出高电平
		}
		else
		{
			T3CCTL0 = 0x04;		//0x14--比较输出，0->105->0 时翻转比较输出引脚 
			//0x04 比较输出引脚 输出高电平
		}
		//T4CTL |= 0x04;    //清零counter
		//read us10Cnt from NV
		
		//while(us10Cnt);
		//T4CTL |= 0x04;    //清零counter
		//T4CC0 = 100 - EdgeTimeSlot_L[i];
		//EdgeTimeSlot_HRsl = EdgeTimeSlot_HR[i];
		singleEdgeEndFlag = 0;
                LED2_SBIT = ~LED2_SBIT;
		while(!singleEdgeEndFlag);
		
	}
	TIMER34_START(4,0);    //关闭T4,
	TIMER3_START(0);       //关闭T3,
	T3CCTL0 = 0;		//0x14--比较输出，0->105 时翻转比较输出引脚
	m25p16_deepPowerDown();//m25p16睡眠
	powerDown();  //2530 外设关闭
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

  //设置外设引脚位置
  //PERCFG &= ~0x01; //USART 0 I/O location  Alternative 1 location  
  
  //设置优先级别
  //P2DIR &=  ~0xC0;//Port 0 peripheral priority control,1st priority: USART 0

  U0CSR &= 0x7f;	//设置为spi模式
  U0CSR  = 0x40;	//设置为spi主机,允许接收
  U0BAUD = 0xFF;//216;
  U0GCR  = 0x30;//0x30;	//这个决定A.波特率的分配值,B.LSB还是MSB,C.时钟相位还是极性
  //设置时钟为11059200
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
//T3初始化
*****************************************/
void Init_T3(void)
{
    TIMER34_INIT(3);                  //初始化T3
    TIMER34_ENABLE_OVERFLOW_INT(3,1);  //开T3中断
    EA = 1;
    T3IE = 1;

    //T3CTL |= 0XA0;                    //时钟32分频101
    TIMER3_SET_CLOCK_DIVIDE(4);			//32m两分频为16M用于T3的时钟源
    TIMER3_SET_MODE(2);                 //up-down 模式 0－>105->0 计数，
    T3CC0 = 104;		//16000000 / (105+105) / 2 =38095Hz ≈38KHz 红外载波
						//（1/16MHz）*210=13.125uS 产生溢出中断，用于捕捉时计时。
	//T3CCTL0 = 0x14;
    T3CCTL0 = 0;//0x14;		//0x14--比较输出，0->104 时翻转比较输出引脚 
						//0x04 比较输出引脚 输出高电平
    T3CCTL1 = 0;//0x43;     //T3 cha1 config as capture mode;both edges 
						//=0x43 T3通道1中断开
    TIMER3_START(0);    //不启动T3,捕捉与T3的计数器是独立的
}



/*****************************************
//T4初始化
*****************************************/
void Init_T4(void)
{
    TIMER34_INIT(4);                  //初始化T4
    TIMER34_ENABLE_OVERFLOW_INT(4,1);  //开T4中断
    EA = 1;
    T4IE = 1;

    //T4CTL |= 0XA0;                    //时钟32分频101
    TIMER34_SET_CLOCK_DIVIDE(4,4);
    TIMER34_SET_MODE(4,2);                 //自动重装00－>T4CC0,mode1 mode3 can generated interreput

    T4CCTL0 = 0x54;		//0x14--比较输出，0->105->0 时翻转比较输出引脚 
						//0x04 比较输出引脚 输出高电平
//    T4CCTL1 = 0x03;     //T3 cha1 config as capture mode;both edges 
						//=0x43 T3通道1中断开
    T4CC0 = 79;                          //timer4 中断分辨率为100us，计时时基为1us
    IP0 = 0x10;                             //set timer4 priority higher than timer3
    TIMER34_START(4,0);                    //启动
}


/*
#pragma vector = T3_VECTOR
 __interrupt void T3_ISR(void)
 {
  if(!edgeCnt)
  {
    TIMER34_START(4,1);                    //启动
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
 
 
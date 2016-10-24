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
//uchar counter = 0;
//uint16 pluseWide[127];
//uint16 pluseWide[127];
//uint16 rEdgeTimeSlot[100];  //上升沿时间间隔
//uint16 fEdgeTimeSlot[100];   //下降沿时间间隔

//uint16 EdgeTimeSlot;   
//uint16 EdgeTimeSlot_LR[500];   //边沿时间间隔= EdgeTimeSlot_LRsl * us10Cnt + EdgeTimeSlot_HRsl
//uint16 EdgeTimeSlot_LRsl = 0;   //边沿时间间隔 低分辨率  10us

//uint8  EdgeTimeSlot_HR[500];   //边沿时间间隔 高分辨率  0.125us
//uint8  EdgeTimeSlot_HRsl = 0;   //边沿时间间隔 高分辨率  0.125us

/*
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
*/

void irCodeRx4Test(void);
void irCodeTx4Test(void);
void flashTest(void);
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



void irCodeRx4Test(void)
{
  uint8 i;
        uint16 ledBlink100msCnt = 0;
	uint8  ms100Cnt=0;//学习超时计时用
	
	P1_5 = 1;//open the power for receive pin 
	Delay_1u(65000);//学习时避开接收管上电瞬间引脚电平翻转的影响
	
	irCodeRxFlag = 1;
	edgeCnt = 0;
    T3CCTL1 = 0x43;     //T3 cha1 config as capture mode;both edges 
    TIMER3_START(1);       //启动T3,
    TIMER34_START(4,1);    //启动T4,
    while(1)
	{
		if(!edgeCnt)
		{
			//无边沿，即没编码。作10s等待处理
			if(10000 == us10Cnt) //100ms
			{
				//ms100Cnt++;
				us10Cnt = 0;
                                ledBlink100msCnt++;
			}
                        /*
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
                                //HalLedBlink ( HAL_LED_2, 3, 50, 200 );//用BLINK模拟灯亮两秒后熄灭
                                P1_1 = 1;//off led			
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

//把数据发回去
void irCodeTx4Test(void)
{
    uint16 i;

    for(i= 0 ;i<10;i++)
    {
      EdgeTimeSlot_LR[i] = 200;
      EdgeTimeSlot_HR[i] = 5;
    }
    edgeNum = 10;
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
                P1_1 = ~P1_1;
		while(!singleEdgeEndFlag);
		
	}
	TIMER34_START(4,0);    //关闭T4,
	TIMER3_START(0);       //关闭T3,
	T3CCTL0 = 0;		//0x14--比较输出，0->105 时翻转比较输出引脚
	irCodeTxFlag = 0;
        P1_1 = 1;//off led
}

//测试flash读写
void flashTest(void)
{
  uint8 m25p16_wrBuff[20] = {1,2,3,4,5,6,7,8,9,10};
  uint8 i;
  //判断ID是否正确
  Hal_SPI_Init();
  m25p16_wakeUP(); //唤醒m25p16，
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
    //ID不对
    P1_1 = 0;//亮红灯
    while(1);
  }
  //读写操作是否OK
  m25p16_beWIP();
  m25p16_writeEnable();
  m25p16_sectorErase(0,0,0);//擦除第0扇区
  
  m25p16_beWIP();
  m25p16_writeEnable();
  m25p16_pageProgram(0,0,0x00,(uint8 *)&(m25p16_wrBuff),10);//写0三区 0页 第0字节 ，写10个
  
  //clear buff
  for(i = 0;i < 10;i++)
    m25p16_wrBuff[i] = 0;
  
  //读出写入的数据
  m25p16_beWIP();
  m25p16_readData(0,0,0x00,(uint8 *)&(m25p16_wrBuff),10);
  
  //对比数据
  for(i = 0;i < 10;i++)
  {
    if(m25p16_wrBuff[i] == i + 1)
    {
      
    }
    else
    {
      //flash fail
      P1_1 = 0;//亮红灯
      while(1);
    }
  }
  
//读写OK，清flash  
  //但不能改写，只能擦除扇区，插0扇区
  m25p16_beWIP();
  m25p16_writeEnable();
  m25p16_sectorErase(0,0,0);//擦除第0扇区
  
  m25p16_beWIP();
  m25p16_readData(0,0,0x00,(uint8 *)&(m25p16_wrBuff),20);//多读10个来判断，看是否被擦除
  
  for(i = 0;i < 20;i++)
  {
    if(m25p16_wrBuff[i] == 0xFF)
    {
      
    }
    else
    {
      //flash fail
      P1_1 = 0;//亮红灯
      while(1);
    }
  }
  //pass
  m25p16_beWIP();
  m25p16_deepPowerDown();//m25p16睡眠
  powerDown();//2530 外设关闭
  
}

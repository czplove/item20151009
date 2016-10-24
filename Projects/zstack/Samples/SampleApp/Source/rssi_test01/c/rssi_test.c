#include <board.h>
#include <int.h>
#include "mcu.h"
#include "button.h"
#include "rf.h"
#include "basic_rf.h"
#include "stdio.h"
#include "hal_assert.h"
#include "io_config.h"
#include "hal_key.h"
#include "hal_led.h"
/***********************************************************************************
* CONSTANTS
*/
// Application parameters

#if defined(test_13_channel)
#define RF_CHANNEL                13      // 2.4 GHz RF channel
#elif defined(test_13_channel)
                17
                  
                  21
                    
#else                   
                    25



// BasicRF address definitions
#define PAN_ID                0x2007
#define SLAVE_ADDR           0x2520
#define MASTER_ADDR            0xBEEF
#define APP_PAYLOAD_LENGTH        7
#define TEST_CMD          0x01


// Application states
#define IDLE                      0
#define SEND_CMD                  1

// Application role
#define NONE                      0
#define APP_MODES                 2

/***********************************************************************************
* LOCAL VARIABLES
*/
static uint8 pTxData[APP_PAYLOAD_LENGTH];
static uint8 pRxData[APP_PAYLOAD_LENGTH];
static basicRfCfg_t basicRfConfig;

static void appMaster(void);
void rssi_test(void);
 uint8 function_test(void);

uint8 ALRAM_FLAG=0;

  #if defined (IR_GENERAL)
   uint8 have_send_flag=0;
  extern void irCodeRx4Test(void);
  extern void irCodeTx4Test(void);
  extern void flashTest(void);
    void test_irr(void);
#endif
 
 int16 Convert(int16 n);

static void appMaster(void)
{
    
    //halRfSetPower();
    int16 Rssi;
    uint8 Rssi_temp[2]={0,0};
    pTxData[0] = 0xFD;
    pTxData[1] = 0x01;
    pTxData[2] = 0x01;
    pTxData[3] = 0x00;
    pTxData[4] = 0xFE;
    pTxData[5] = 0x0D;
    pTxData[6] = 0x0A;
    
    // Initialize BasicRF
    basicRfConfig.myAddr = MASTER_ADDR;
    
 if(basicRfInit(&basicRfConfig)==1) {
      HAL_ASSERT(FALSE);
    }
    basicRfReceiveOn();
   
    basicRfSendPacket(SLAVE_ADDR, pTxData, APP_PAYLOAD_LENGTH);
     
    while (1) 
    {
      
      while(!basicRfPacketIsReady());
      /*
      while(1)
      {
        
       function_test();
       if(basicRfPacketIsReady())
         break;
      }
      */
  
       if(basicRfReceive(pRxData, APP_PAYLOAD_LENGTH, &Rssi)>0) 
          if(pRxData[1]==TEST_CMD)
          {
           //  Rssi_temp[0] = (uint8)Rssi;
           //  Rssi_temp[1] = (uint8)(Rssi>>8);
             pTxData[1] = 0x02;
             pTxData[3]=(uint8)Convert(Rssi);
       //      pTxData[3] = (uint8)Rssi;
            // basicRfSendPacket(SLAVE_ADDR,Rssi_temp,APP_PAYLOAD_LENGTH);
             basicRfSendPacket(SLAVE_ADDR,pTxData,APP_PAYLOAD_LENGTH);
             break;
          }
    }
    while(1)
    {
      uint8 i=0;
      i=function_test();
     if(i==1)
     {
      break;
     }
    }
    HAL_SYSTEM_RESET(); 
}

void rssi_test(void)
{
    // Config basicRF
    basicRfConfig.panId = PAN_ID;
    basicRfConfig.channel = RF_CHANNEL;
    basicRfConfig.ackRequest = TRUE;
#ifdef SECURITY_CCM
    basicRfConfig.securityKey = key;
#endif
    
    // Initalise board peripherals
    halBoardInit();
    appMaster();
}
    

uint8 function_test(void)
 {

  #if defined (SR_PIR) || (SR_MENCI)
       //尝试添加检测状态的报警 
    if( (ALRAM_FLAG==0) && (PUSH4_SBIT==1))
    {
      //处于消警状态， 但告警了
      ALRAM_FLAG=1;
      pTxData[1]=3;
      pTxData[2]=1;
      pTxData[3]=1;
      basicRfSendPacket(SLAVE_ADDR, pTxData, APP_PAYLOAD_LENGTH);
      #if defined (SR_PIR)
      HAL_TURN_ON_LED2();
      #elif defined (SR_MENCI)
      HAL_TURN_ON_LED1();
      #endif
      Delay_1u(50000);
      #if defined (SR_PIR)
      HAL_TURN_OFF_LED2();
      #elif defined (SR_MENCI)
      HAL_TURN_OFF_LED1();
      #endif
      Delay_1u(50000);
      #if defined (SR_PIR)
      HAL_TURN_ON_LED2();
      #elif defined (SR_MENCI)
      HAL_TURN_ON_LED1();
      #endif
      Delay_1u(50000);
      #if defined (SR_PIR)
      HAL_TURN_OFF_LED2();
      #elif defined (SR_MENCI)
      HAL_TURN_OFF_LED1();
      #endif
      return 0;
    }
    
    else if( (ALRAM_FLAG==1) && (PUSH4_SBIT==0))
    {
      //处于报警状态， 但消除警告了
      ALRAM_FLAG=0;
      pTxData[1]=3;
      pTxData[2]=1;
      pTxData[3]=0;
      basicRfSendPacket(SLAVE_ADDR, pTxData, APP_PAYLOAD_LENGTH);
      #if defined (SR_PIR)
        HAL_TURN_ON_LED2();
      #elif defined (SR_MENCI)
        HAL_TURN_ON_LED1();
      #endif
        
        Delay_1u(100000);
      
      #if defined (SR_PIR)
        HAL_TURN_OFF_LED2();
      #elif defined (SR_MENCI)
        HAL_TURN_OFF_LED1();
      #endif
         Delay_1u(500000);
         return 1;
    }
         return 0;
    // 进行收发测试
  #elif defined (IR_GENERAL)
    
   // if(have_send_flag==0)
    {
      test_irr();
      pTxData[1]=4;
      pTxData[2]=1;
      pTxData[3]=0;
      basicRfSendPacket(SLAVE_ADDR, pTxData, APP_PAYLOAD_LENGTH);
      have_send_flag=1;
      
      HAL_TURN_ON_LED2();
      Delay_1u(100000);
      HAL_TURN_OFF_LED2();
      Delay_1u(100000);
      HAL_TURN_ON_LED2();
      Delay_1u(100000);
      HAL_TURN_OFF_LED2();
       return 1;
    }
    
#else
     Delay_1u(100000);
     return 1;
  #endif
}

  #if defined (IR_GENERAL)
  void test_irr(void)
  {
//if(have_send_flag==0)
    {
      flashTest(); 
      irCodeTx4Test();
    //  Delay_1u(100000);
      irCodeRx4Test();
      


    }
  }
   #endif


int16 Convert(int16 n)
{
  int16 m;
  if(n<0)
  {
    m = (~(n-1))|0x8000;
    return m;
  }
  else
  {
    return n;
  }  
}

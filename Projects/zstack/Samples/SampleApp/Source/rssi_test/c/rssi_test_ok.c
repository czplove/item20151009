#include <board.h>
#include <int.h>
#include "mcu.h"
#include "button.h"
#include "rf.h"
#include "basic_rf.h"
#include "stdio.h"
#include "hal_assert.h"
/***********************************************************************************
* CONSTANTS
*/
// Application parameters
#define RF_CHANNEL                25      // 2.4 GHz RF channel

// BasicRF address definitions
#define PAN_ID                0x2007
#define SLAVE_ADDR           0x2520
#define MASTER_ADDR            0xBEEF
#define APP_PAYLOAD_LENGTH        2
#define TEST_CMD          0x0f


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
//int16 Time_MS = 0;
//static uint8 flag = FALSE;

static void appMaster(void);
static void appSlave(void);
//static void timer4_init(void);



static void appMaster(void)
{

    int16 Rssi;
    uint8 Rssi_temp[2]={0,0};
    pTxData[0] = 0;
    pTxData[1] = TEST_CMD;
    
    // Initialize BasicRF
    basicRfConfig.myAddr = MASTER_ADDR;
    if(basicRfInit(&basicRfConfig)==FAILED) {
      HAL_ASSERT(FALSE);
    }
    
    basicRfReceiveOn();
    
    basicRfSendPacket(SLAVE_ADDR, pTxData, APP_PAYLOAD_LENGTH);
    
    while (TRUE) 
    {
//       if(flag)
//       {
//          basicRfSendPacket(SLAVE_ADDR, pTxData, APP_PAYLOAD_LENGTH);
//          flag = FALSE;
//       }
      //putchar('C'); 
      while(!basicRfPacketIsReady());

       if(basicRfReceive(pRxData, APP_PAYLOAD_LENGTH, &Rssi)>0) 
          if(pRxData[1]==TEST_CMD)
          {
             Rssi_temp[0] = (uint8)Rssi;
             Rssi_temp[1] = (uint8)(Rssi>>8);
             basicRfSendPacket(SLAVE_ADDR,Rssi_temp,APP_PAYLOAD_LENGTH);
          }
    }
}

void Init_Watchdog(void)
{
    WDCTL = 0x00;
	//间隔一秒，看门狗模式
    WDCTL |= 0x08;
	//启动看门狗
}

static void appSlave(void)
{
    Init_Watchdog();
    //pTxData[0] = 0;
    putchar(0x00);
    //halRfSetPower();
    int16 Rssi_Tx;
    int16 Rssi_Rx;
    // Initialize BasicRF
    basicRfConfig.myAddr = SLAVE_ADDR;
    if(basicRfInit(&basicRfConfig)==FAILED) {
      HAL_ASSERT(FALSE);
    }

    basicRfReceiveOn();
    
    //basicRfSendPacket(SLAVE_ADDR, pTxData, APP_PAYLOAD_LENGTH);
    while (TRUE)
    {
        //printf("RSSI_RX:%d",12);
      
       while(!basicRfPacketIsReady())
       {
          WDCTL = 0xa0;
          WDCTL = 0x50;
       }
      
       if(basicRfReceive(pRxData, APP_PAYLOAD_LENGTH, &Rssi_Tx)>0) 
       {
          if(pRxData[1]==TEST_CMD)
          {
              
              basicRfSendPacket(MASTER_ADDR, pRxData, APP_PAYLOAD_LENGTH);
          }
          else
          {
              //printf("RSSI_TX:%d",pRxData[0]);
              Rssi_Rx = pRxData[0]|(((int16)pRxData[1])<<8);
             
          }
       }
    }
}

void main(void)
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
    uartInit();
    
#ifdef SLAVE
        // No return from here
        appSlave();
#endif
        // Receiver application
#ifdef MASTER
        // No return from here
        appMaster();
#endif
}

/***********************************************************************

#pragma vector = T4_VECTOR      
__interrupt void T4_ISR(void)
{
  T4IF = 0;
  if(++Time_MS == 5000)
  {
    flag = TRUE;
    Time_MS = 0;
  }
}

*************************************************************************/
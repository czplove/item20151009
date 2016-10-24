/**************************************************************************************************
  Filename:       SampleApp.c
  Revised:        $Date: 2007-10-27 17:16:54 -0700 (Sat, 27 Oct 2007) $
  Revision:       $Revision: 15793 $

  Description:    Sample Application (no Profile).


  Copyright 2007 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED �AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
  This application isn't intended to do anything useful, it is
  intended to be a simple example of an application's structure.

  This application sends it's messages either as broadcast or
  broadcast filtered group messages.  The other (more normal)
  message addressing is unicast.  Most of the other sample
  applications are written to support the unicast message model.

  Key control:
    SW1:  Sends a flash command to all devices in Group 1.
    SW2:  Adds/Removes (toggles) this device in and out
          of Group 1.  This will enable and disable the
          reception of the flash command.
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */



//#define COMMUNICATION
#include "OSAL.h"
#include "ZGlobals.h"
#include "AF.h"
#include "aps_groups.h"
#include "ZDApp.h"
//#include "device.h"
#include "OnBoard.h"
#include "MT_UART.h"//2430 �� SPIMGR  2530�� MT_UART  ���˸�����
#include "mac_rx.h"

/* HAL */
#include "hal_led.h"
#include "hal_key.h"
#include "OSAL_Nv.h"

#include "AddrMgr.h"
#include "nwk_util.h"
#include "ZDObject.h"
#include "ZDO_LIB.h"
#include "common_device.h"
#include "hal_drivers.h"
#include "hal_key.h"
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */


    uint8 Command_t,Sent_t;
    
    devStates_t SampleApp_NwkState;
    

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES��ȫ�ֱ���
 */

const cId_t SampleApp_ClusterList[SAMPLEAPP_MAX_CLUSTERS] =
{
  SAMPLEAPP_ONOFF_CLUSTERID, //������Ϣ��ID=1
  SAMPLEAPP_FLASH_CLUSTERID,    //��˸��Ϣ��ID=2
  SAMPLEAPP_ADDR_CLUSTERID,
  SAMPLEAPP_COMMAND_CLUSTERID,
  SAMPLEAPP_PERIODIC_CLUSTERID,
  SAMPLEAPP_ANNCE_REQ_CLUSTERID,
  SAMPLEAPP_PERIODIC_COORDINATOR_CLUSTERID,
};


const SimpleDescriptionFormat_t SampleApp_SimpleDesc =
{
  SAMPLEAPP_ENDPOINT,              //  int Endpoint;
  SAMPLEAPP_PROFID,                //  uint16 AppProfId[2];/**/
  SAMPLEAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  SAMPLEAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  SAMPLEAPP_FLAGS,                 //  int   AppFlags:4;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList,  //  uint8 *pAppInClusterList;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList   //  uint8 *pAppInClusterList;
};

//endPointDesc_t SampleApp_epDesc;



afAddrType_t SampleApp_Periodic_DstAddr;       //�����¼�Ѱַģʽ
afAddrType_t SampleApp_Flash_DstAddr;          //��˸�¼�Ѱַģʽ
afAddrType_t SampleApp_SPI_SendData_DstAddr;   //����������Ϣ
afAddrType_t SampleApp_SPI_SendCommand_DstAddr;//����������Ϣ
afAddrType_t SampleApp_Addr_SendData_DstAddr;  //��������
afAddrType_t SampleApp_annce_req_DstAddr;    //annce ������Ա������Ϣ
afAddrType_t SampleApp_BIND_DstAddr;



/*********************************************************************
 * LOCAL FUNCTIONS
 */
//void SampleApp_HandleKeys( uint8 shift, uint8 keys );//���û�Ӧ�ð�������
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );//���û�Ӧ����Ϣ�ص�����
void SampleApp_SendPeriodicMessage( void );         //���û�Ӧ�÷���������Ϣ����


void SampleApp_ProcessNwkaddrMessage(afIncomingMSGPacket_t *pkt);
//void SampleAPP_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
void SampleApp_ProcessCommandMessage(afIncomingMSGPacket_t *pkt);
void SampleApp_ROUTER_ProcessPeriodicMessage_Data(afIncomingMSGPacket_t *pkt);

void SampleApp_SendInMessage(void);

void SampleApp_ProcessBINDINGMessage(afIncomingMSGPacket_t *pkt);


void  Delay_1u(uint32 microSecs);


uint8 zb_ReadConfiguration( uint8 configId, uint8 len, void *pValue );
uint8 zb_WriteConfiguration( uint8 configId, uint8 len, void *pValue );
void zb_StartRequest ( void );
void Init_Watchdog(void);
void FeetDog(void);
/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SampleApp_Init
 *
 * @brief   Initialization function for the Generic App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SampleApp_Init( uint8 task_id )
{
  
    #if defined ( LEAVE_NET_SET )
    LEAVE_NET_SET_FLAG =1;
  #else
    LEAVE_NET_SET_FLAG =0;
  #endif
    
  #if defined ( RTR_NWK )
    RTR_NWK_FLAG =1;
  #else
    RTR_NWK_FLAG =0;
  #endif
  
  #if defined ( POWER_SAVING )
    POWER_SAVING_FLAG =1;
  #else
    POWER_SAVING_FLAG =0;
  #endif
  
  #if defined ( POWER_PA )
    POWER_PA_FLAG =1;
  #else
    POWER_PA_FLAG =0;
  #endif
    
  
 
  SampleApp_TaskID = task_id;
  SampleApp_NwkState = DEV_INIT;
  SampleApp_TransID = 0;



  SampleApp_epDesc.endPoint = SAMPLEAPP_ENDPOINT;//SampleApp EP��������EP�ţ�20
  SampleApp_epDesc.task_id = &SampleApp_TaskID;//SampleApp EP������������ID��0
  SampleApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&SampleApp_SimpleDesc;//SampleApp EP��������
  SampleApp_epDesc.latencyReq = noLatencyReqs;//��ʱ����

  
  // Register the endpoint description with the AF
  afRegister( &SampleApp_epDesc );
 
   APP_JOIN_INIT();

}

/*********************************************************************
 * @fn      SampleApp_ProcessEvent
 *
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
uint16 SampleApp_ProcessEvent( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
 
  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    while ( MSGpkt )                                           
    {                                                                     
      switch ( MSGpkt->hdr.event )
      {
                                         

        case AF_INCOMING_MSG_CMD:
          SampleApp_MessageMSGCB( MSGpkt );
          break;
 
       
          
        case ZDO_STATE_CHANGE:  
          SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (SampleApp_NwkState == DEV_ZB_COORD)
              || (SampleApp_NwkState == DEV_ROUTER)
              || (SampleApp_NwkState == DEV_END_DEVICE) )
          {
      
            APP_JOIN_DEAL_WITH();
            
            USER_APP_JOIN_MSG(); // ����������Ӧ�õ�����
          }
          else
          {
            // Device is no longer in the network
          }
          break;

        default:
          break;
      }

      // Release the memory
      //�ͷ���Ϣռ�õ��ڴ�
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next - if one is available
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    }//end�� while ( MSGpkt )

    return (events ^ SYS_EVENT_MSG);
  }
  
  if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT )//����������Ϣ�¼�
  {
   //   Heart_mesg_count = 0;
      SampleApp_SendPeriodicMessage();
      osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
                          55000+ 30*(osal_rand() & 0x00FF));

    return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT);
  }
  

//********************** 

/*��COMMUNICTION����������Ϣ*/
  if ( events & SAMPLEAPP_SEND_NWKADDR_EVT )
  {
      SampleApp_SendNwkaddrMessage(JOIN_ROLE,DEVICE_ID);

    return ( events ^ SAMPLEAPP_SEND_NWKADDR_EVT);
  }

    if ( events & SAMPLEAPP_SEND_RS_EVT )
  {

    USER_SET_RS_MSG();//����RS����
    sampleAPP_sendRS_periodMessage(JOIN_ROLE,DEVICE_ID,RS_LEN,RS_STR);
    
     USER_AFTER_RS_MSG();//RS�����Ƿ�Ҫ��ӵĺ���
    return ( events ^ SAMPLEAPP_SEND_RS_EVT);
  }
  
   
  
      if ( events & NEW_JOIN_ROLE_EVT )
  {
       APP_JOIN_START_EVT();
     return ( events ^ NEW_JOIN_ROLE_EVT);
    
  }
  

if ( events & SAMPLEAPP_RESTORE_EVT )
  {
    
      Change_Net_Set();
    
    return ( events ^ SAMPLEAPP_RESTORE_EVT);
  }
  
  

     if ( events & ASSOCIA_MENTAIN_EVT )
  {
     APP_DEAL_WITH_ASSOCIATED();
     return (events ^ ASSOCIA_MENTAIN_EVT );
  } 
  
 
  
//**********************     
  // Discard unknown events
  return 0;
}


/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      SampleApp_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */

void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  //uint16 flashTime;
  switch ( pkt->clusterId )
  {
  

    case SAMPLEAPP_COMMAND_CLUSTERID:

      SampleApp_ProcessCommandMessage(pkt);
      break;  
      
      
       case SAMPLEAPP_ONOFF_CLUSTERID:   
      
      SampleApp_ProcessBINDINGMessage(pkt);
    break; 
      
    
     case SAMPLEAPP_ADDR_CLUSTERID: //�豸��������Ϣ����

      SampleApp_ProcessNwkaddrMessage(pkt);

      break; 
      
     case SAMPLEAPP_PERIODIC_CLUSTERID:

      SampleApp_ROUTER_ProcessPeriodicMessage_Data(pkt);

      break;  
      
  default:break;
    
  }
}





/*********************************************************************
 * @fn      SampleApp_ProcessCommandMessage
 *
 * @brief   Prcoess the (single) message command from the coordinataor.
 *
 * @param   pkt
 *
 * @return  none
 */
/*0 1 2 3 4 5 6  7  8    9 10  11  12   13   14 ...*/
/*_ _ _ / C / U  U  D   A    T    A ...*/

void SampleApp_ProcessCommandMessage(afIncomingMSGPacket_t *pkt)
{
   uint8 str[30];

 // uint8 *Sample_Cmd_shorAddr;
 // uint8 *Sample_extendaddr;
 // uint8 data_sucess_flag=0;
  uint8 active_count=0;
  uint8 SET_str[10];

  for(uint8 i=0;i<4;i++)
  {
   str[i] = SHORT_ADRESS[i];
  }
  str[4]='/';
  str[5]='C';
  str[6]='/';
  

   str[7]=pkt->cmd.Data[6];//�����豸��������
   Sent_t=str[7];
      
  SET_str[0] = str[0];
  SET_str[1] = str[1];
  SET_str[2] = str[2];
  SET_str[3] = str[3];
  SET_str[4] = pkt->cmd.Data[8];
  SET_str[5] = pkt->cmd.Data[13];
  SET_str[6] = pkt->cmd.Data[14];
  SET_str[7] = pkt->cmd.Data[15];
  SET_str[8] = pkt->cmd.Data[16];
  SET_str[9] = ABS(pkt->rssi);
  
  APP_NORMAL_COMMAND(SET_str); 
   
     //�����豸������  ����������
   if(pkt->cmd.Data[8]==DEFINED_CONTROL_CHAR)
   {
    active_count = USER_APP_COMMAND(&pkt->cmd.Data[13],&str[9]);
    str[8]='U';//���η���
    SampleApp_SPI_SendCommand_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
    SampleApp_SPI_SendCommand_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
    SampleApp_SPI_SendCommand_DstAddr.addr.shortAddr = pkt->srcAddr.addr.shortAddr;//���͸������豸

    if ( AF_DataRequest( &SampleApp_SPI_SendCommand_DstAddr,
                         (endPointDesc_t *)&SampleApp_epDesc,
                          SAMPLEAPP_COMMAND_CLUSTERID, //�ն˴���ȥ��Э����������
                          active_count, str,
                          &SampleApp_TransID, 
                          AF_DISCV_ROUTE, 
                          AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
    {
     }
    else
     {
     }
   }
 
 
    else if(pkt->cmd.Data[8]=='L')//v:118
 {
  uint8 blink_count=0;
  blink_count = (pkt->cmd.Data[13]-0x30)*100 + (pkt->cmd.Data[14]-0x30)*10 + (pkt->cmd.Data[15]-0x30);
  HalLedBlink ( HAL_LED_1, blink_count, 50, 800 );

 }
 
 else if(pkt->cmd.Data[8]==82)//R:86 ˢ���豸�б� ���⴦��ֱ�Ӱ�һ����ʽ�ϱ�
 {
  
   osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_RS_EVT,100);

 }
 


}





void Delay_1u(uint32 microSecs) 
{
  while(microSecs--)
  {
    /* 32 NOPs == 1 usecs */
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop");
  }
}


/************************************************************
*���Ź���ʼ��
*************************************************************/
void Init_Watchdog(void)
{
	WDCTL = 0x00;
	//ʱ����һ�룬���Ź�ģʽ
	WDCTL |= 0x08;
	//�������Ź�
}


/************************************************************
*ι������
*************************************************************/
void FeetDog(void)
{
	WDCTL = 0xa0;
	WDCTL = 0x50;
}

void SampleApp_PermitJoin(uint8 permit_time);
void SampleApp_PermitJoin(uint8 permit_time)
{
    zAddrType_t dstAddr;
 if( (devState==DEV_END_DEVICE) || (devState==DEV_ROUTER) )
      {
        if(permit_time>0)
        HalLedBlink ( HAL_LED_1, 1, 50, 600 );//Ӧ��
        else
         HalLedBlink ( HAL_LED_1, 2, 50, 600 );//Ӧ��  
        NLME_PermitJoiningRequest(permit_time);
        dstAddr.addrMode = AddrBroadcast;
        dstAddr.addr.shortAddr = 0XFFFC;   // Coordinator makes the match
        ZDP_MgmtPermitJoinReq( &dstAddr, permit_time, 0, 0);
      }
}


void SampleApp_send_in_binding_req(uint8 endpoint,uint16 *bindingINClusters,uint8 in_len);
void SampleApp_send_in_binding_req(uint8 endpoint,uint16 *bindingINClusters,uint8 in_len)
{
  zAddrType_t dstAddr;
 if((devState == DEV_END_DEVICE) || (devState == DEV_ROUTER)) 
            {
                    HalLedBlink ( HAL_LED_1, 1, 50, 1000 );
                    dstAddr.addrMode = afAddr16Bit;
                    dstAddr.addr.shortAddr = 0;   
                    ZDP_EndDeviceBindReq( &dstAddr,
                                                            NLME_GetShortAddr(),
                                                            endpoint,
                                                            0x0F08,in_len,
                                                            bindingINClusters,   // No incoming clusters to bind   0, NULL
                                                            0, NULL, FALSE );
            }
}


void SampleApp_send_out_binding_req(uint8 endpoint,uint16 *bindingOUTClusters,uint8 out_len);
void SampleApp_send_out_binding_req(uint8 endpoint,uint16 *bindingOUTClusters,uint8 out_len)
{
  zAddrType_t dstAddr;
 if((devState == DEV_END_DEVICE) || (devState == DEV_ROUTER)) 
            {
                    HalLedBlink ( HAL_LED_1, 1, 50, 1000 );
                    dstAddr.addrMode = afAddr16Bit;
                    dstAddr.addr.shortAddr = 0;   
                    ZDP_EndDeviceBindReq( &dstAddr,
                                                            NLME_GetShortAddr(),
                                                            endpoint,
                                                            0x0F08,0,
                                                            NULL,
                                                            out_len, bindingOUTClusters, FALSE );
            }
}




void USER_SEND_ADD_STR(uint8 *str);
void USER_SEND_ADD_STR(uint8 *str)
{
  for(uint8 i=0;i<4;i++)
  {
   str[i] = SHORT_ADRESS[i];
  }
  for(uint8 j=0;j<5;j++)
  {
   str[4+j] = '/';
  }
}


void USER_APP_SEND_IN(uint8 count,uint8 *str);
void USER_APP_SEND_IN(uint8 count,uint8 *str)
{  

  SampleApp_SPI_SendCommand_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  SampleApp_SPI_SendCommand_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_SPI_SendCommand_DstAddr.addr.shortAddr = 0X0000;
    
    if ( AF_DataRequest( &SampleApp_SPI_SendCommand_DstAddr,
                         (endPointDesc_t *)&SampleApp_epDesc,
                          SAMPLEAPP_COMMAND_CLUSTERID, //�ն˴���ȥ��Э����������
                          count, str,
                          &SampleApp_TransID, 
                          AF_DISCV_ROUTE, 
                          AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
    {
      
    }
    else
    {
      
    }   

}

void HalKeyHoldCheck(void);
void HalKeyHoldCheck(void)
{
  uint8 keys = 0;
  keys |= HAL_KEY_SW_8;
  if(!(READ_NETKEY()))
  {
	key_hold_flag = 1;//������Ч��  key_double_flag ��ʾ�������µĴ���
	if (keys && (pHalKeyProcessFunction))
	{
	  (pHalKeyProcessFunction) (keys, HAL_KEY_STATE_NORMAL);
	  // key_ISR_FLAG=0;
	  KEY_NET_ISR_FLAG=0;
	}
  }
  key_holdtime_waiting_flag = 0;	//����ȴ�����ʱ���־
  KEY_NET_ISR_FLAG=0;
}

void HalKeyMultiClickCheck(void);
void HalKeyMultiClickCheck(void)
{
  uint8 keys = 0;
  
  keys |= HAL_KEY_SW_8;
  key_double_flag=KEY_NET_ISR_FLAG;//�����ΰ���ֵ����״̬
  if(1 == key_double_flag)
  {
	if((READ_NETKEY()))
	{
	  key_hold_flag = 0;
	  if (keys && (pHalKeyProcessFunction))
	  {
		(pHalKeyProcessFunction) (keys, HAL_KEY_STATE_NORMAL);

		KEY_NET_ISR_FLAG=0;
	  }
	  
	}
	else
	{
	(pHalKeyProcessFunction) (keys, HAL_KEY_STATE_NORMAL);
	 	osal_start_timerEx (Hal_TaskID, HAL_KEY_HOLD_EVENT, HAL_KEY_HOLD_VALUE);//����ʱ�䣨10s���ﵽ�Ժ������Ӧ�Ĵ����¼�
		key_holdtime_waiting_flag = 1;
	}
  }
  else
  {
	
	key_hold_flag = 0;//������Ч��  key_double_flag ��ʾ�������µĴ���
	if (keys && (pHalKeyProcessFunction))
	{
	  (pHalKeyProcessFunction) (keys, HAL_KEY_STATE_NORMAL);
	  // key_ISR_FLAG=0;
	  KEY_NET_ISR_FLAG=0;
	}
  }
  
}
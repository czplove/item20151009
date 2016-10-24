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


#include "device.h"
//#include "SampleAppHw.h"

#include "OnBoard.h"
//#include "SPIMgr.h"//###
#include "MT_UART.h"//2430 �� SPIMGR  2530�� MT_UART  ���˸�����
#include "mac_rx.h"

/* HAL */
#include "hal_led.h"
#include "hal_key.h"
#include "OSAL_Nv.h"
#include "my_time.h"

#include "AddrMgr.h"
#include "nwk_util.h"
#include "ZDObject.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

    uint8 *shortddr_mem;  
    uint8 *extendaddr_mem;
    uint8 Command_t,Sent_t;
 //   uint16 Communication_saddr; 
     extern uint8 key_double_P1_5_flag;
      extern uint8 key_double_P1_6_flag;
       extern uint8 key_double_P1_7_flag;

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES��ȫ�ֱ���
 */
#define GENGER_ONOFF_BINDINGLIST       1
static cId_t bindingINClusters[GENGER_ONOFF_BINDINGLIST] =
{
  SAMPLEAPP_ONOFF_CLUSTERID
};    
    
     
const cId_t SampleApp_ClusterList[SAMPLEAPP_MAX_CLUSTERS] =
{
  SAMPLEAPP_ONOFF_CLUSTERID, //������Ϣ��ID=1
  SAMPLEAPP_FLASH_CLUSTERID,    //��˸��Ϣ��ID=2
  SAMPLEAPP_ADDR_CLUSTERID,
  SAMPLEAPP_COMMAND_CLUSTERID,
  SAMPLEAPP_PERIODIC_CLUSTERID,
  SAMPLEAPP_ANNCE_REQ_CLUSTERID,
  SAMPLEAPP_PERIODIC_COORDINATOR_CLUSTERID,
  SAMPLEAPP_ACTIVE_COMMAND_CLUSTERID
};



const SimpleDescriptionFormat_t SampleApp_SimpleDesc1 =
{
  SAMPLEAPP_ENDPOINT1,              //  int Endpoint;
  SAMPLEAPP_PROFID,                //  uint16 AppProfId[2];/**/
  SAMPLEAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  SAMPLEAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  SAMPLEAPP_FLAGS,                 //  int   AppFlags:4;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList,  //  uint8 *pAppInClusterList;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList   //  uint8 *pAppInClusterList;
};



const SimpleDescriptionFormat_t SampleApp_SimpleDesc2 =
{
  SAMPLEAPP_ENDPOINT2,              //  int Endpoint;
  SAMPLEAPP_PROFID,                //  uint16 AppProfId[2];/**/
  SAMPLEAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  SAMPLEAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  SAMPLEAPP_FLAGS,                 //  int   AppFlags:4;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList,  //  uint8 *pAppInClusterList;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList   //  uint8 *pAppInClusterList;
};



const SimpleDescriptionFormat_t SampleApp_SimpleDesc3 =
{
  SAMPLEAPP_ENDPOINT3,              //  int Endpoint;
  SAMPLEAPP_PROFID,                //  uint16 AppProfId[2];/**/
  SAMPLEAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  SAMPLEAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  SAMPLEAPP_FLAGS,                 //  int   AppFlags:4;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList,  //  uint8 *pAppInClusterList;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList   //  uint8 *pAppInClusterList;
};


const SimpleDescriptionFormat_t SampleApp_SimpleDesc4 =
{
  SAMPLEAPP_ENDPOINT4,              //  int Endpoint;
  SAMPLEAPP_PROFID,                //  uint16 AppProfId[2];/**/
  SAMPLEAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  SAMPLEAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  SAMPLEAPP_FLAGS,                 //  int   AppFlags:4;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList,  //  uint8 *pAppInClusterList;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList   //  uint8 *pAppInClusterList;
};



endPointDesc_t SampleApp_epDesc1;
endPointDesc_t SampleApp_epDesc2;
endPointDesc_t SampleApp_epDesc3;
endPointDesc_t SampleApp_epDesc4;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
uint8 SampleApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // SampleApp_Init() is called.
devStates_t SampleApp_NwkState;

uint8 SampleApp_TransID;  // This is the unique message ID (counter)



afAddrType_t SampleApp_Periodic_DstAddr;       //�����¼�Ѱַģʽ
afAddrType_t SampleApp_Flash_DstAddr;          //��˸�¼�Ѱַģʽ
afAddrType_t SampleApp_SPI_SendData_DstAddr;   //����������Ϣ
afAddrType_t SampleApp_SPI_SendCommand_DstAddr;//����������Ϣ
afAddrType_t SampleApp_Addr_SendData_DstAddr;  //��������
afAddrType_t SampleApp_annce_req_DstAddr;    //annce ������Ա������Ϣ
afAddrType_t SampleApp_BIND_DstAddr;

#ifndef ZDO_COORDINATOR
//aps_Group_t SampleApp_Group1;
//aps_Group_t SampleApp_Group2;
aps_Group_t SampleApp_Group3; //��
#endif


uint8 SampleAppPeriodicCounter = 0;
uint8 SampleAppFlashCounter = 0;
uint16 dst_short_ddr=0;


//���ڹ�����
#if defined(RTR_NWK)
  Child_device_t Child_list[21];
#endif

//��������
//static uint8 myAppState = APP_INIT;
 uint8 logicalType1=0;
// Application States
 
uint8 ON_OFF_ASSOCIATE_flag=0;
uint8 ANNCE_REQ_FLAG=0;


    uint8 OUT_FLAG=0;
    uint8 delay_stop_out_flag=0;
    uint8 delay_mintues_count=0;
    
    uint8 OUT1_SBIT_FLAG=0;
    uint8 OUT2_SBIT_FLAG=0;
    uint8 OUT3_SBIT_FLAG=0; 
    uint8 OUT4_SBIT_FLAG=0; 
/*********************************************************************
 * LOCAL FUNCTIONS
 */
void SampleApp_HandleKeys( uint8 shift, uint8 keys );//���û�Ӧ�ð�������
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );//���û�Ӧ����Ϣ�ص�����
void SampleApp_SendPeriodicMessage( void );         //���û�Ӧ�÷���������Ϣ����

void SampleApp_SendNwkaddrMessage(void);
void SampleApp_ProcessNwkaddrMessage(afIncomingMSGPacket_t *pkt);
void SampleAPP_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
void SampleApp_ProcessCommandMessage(afIncomingMSGPacket_t *pkt);
void SampleApp_ROUTER_ProcessPeriodicMessage_Data(afIncomingMSGPacket_t *pkt);
void ZDApp_PROESS_AnnounceAddress_CONFLICT(afIncomingMSGPacket_t *pkt);


void SampleApp_SendInMessage(void);

void SampleApp_ProcessBINDINGMessage(afIncomingMSGPacket_t *pkt);

uint8 *SampleApp_GetShortAddr(void);
uint8 *SampleApp_GetExtendAddr(void);
uint8 *SampleApp_GetExtendAddr_MAC(void);
uint8 hextoword1(uint8 t );
uint8 hextoword2(uint8 t);
uint8 wordtohex(uint8 x,uint8 y);
void PowerMode(uint8 sel);



uint8 RemoveStaleNode(uint8 index);
uint8 RemoveStaleRouter(uint8 index);
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
  SampleApp_TaskID = task_id;
  SampleApp_NwkState = DEV_INIT;
  SampleApp_TransID = 0;


  SampleApp_BIND_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  SampleApp_BIND_DstAddr.endPoint = 0;
  SampleApp_BIND_DstAddr.addr.shortAddr = 0;
//------------------
  
// Fill out the endpoint description.
  SampleApp_epDesc1.endPoint = SAMPLEAPP_ENDPOINT1;//SampleApp EP��������EP�ţ�20
  SampleApp_epDesc1.task_id = &SampleApp_TaskID;//SampleApp EP����������ID��0
  SampleApp_epDesc1.simpleDesc = (SimpleDescriptionFormat_t *)&SampleApp_SimpleDesc1;//SampleApp EP��������
  SampleApp_epDesc1.latencyReq = noLatencyReqs;//��ʱ����

  SampleApp_epDesc2.endPoint = SAMPLEAPP_ENDPOINT2;//SampleApp EP��������EP�ţ�20
  SampleApp_epDesc2.task_id = &SampleApp_TaskID;//SampleApp EP������������ID��0
  SampleApp_epDesc2.simpleDesc  = (SimpleDescriptionFormat_t *)&SampleApp_SimpleDesc2;//SampleApp EP��������
  SampleApp_epDesc2.latencyReq = noLatencyReqs;//��ʱ����
  
  SampleApp_epDesc3.endPoint = SAMPLEAPP_ENDPOINT3;//SampleApp EP������EP�ţ�20
  SampleApp_epDesc3.task_id = &SampleApp_TaskID;//SampleApp EP������������ID��0
  SampleApp_epDesc3.simpleDesc    = (SimpleDescriptionFormat_t *)&SampleApp_SimpleDesc3;//SampleApp EP��������
  SampleApp_epDesc3.latencyReq = noLatencyReqs;//��ʱ����
  
  SampleApp_epDesc4.endPoint = SAMPLEAPP_ENDPOINT4;//SampleApp EP������EP�ţ�20
  SampleApp_epDesc4.task_id = &SampleApp_TaskID;//SampleApp EP������������ID��0
  SampleApp_epDesc4.simpleDesc    = (SimpleDescriptionFormat_t *)&SampleApp_SimpleDesc4;//SampleApp EP��������
  SampleApp_epDesc4.latencyReq = noLatencyReqs;//��ʱ����
  
  // Register the endpoint description with the AF
  afRegister( &SampleApp_epDesc1 );
  afRegister( &SampleApp_epDesc2 );
  afRegister( &SampleApp_epDesc3 );  // Register for all key events - This app will handle all key events
  afRegister( &SampleApp_epDesc4 );
  
  RegisterForKeys( SampleApp_TaskID ); 
  
  ZDO_RegisterForZDOMsg( SampleApp_TaskID, End_Device_Bind_rsp );
  
  // By default, all devices start out in Group 1  
  //ÿ���豸�������ʱ�����趨��������Ϊ0x0003���飬Ȼ��ѱ��豸EP20��ӵ��������


    
  osal_set_event(task_id, ZB_ENTRY_EVENT);
  
  //���ݹ������ʼ�����豸�� 
#if defined(RTR_NWK)
 uint8 nr;
for(uint8 i=0;i<NWK_MAX_DEVICES;i++)
 { 
   nr =  AssociatedDevList[i].nodeRelation;
   if (nr == CHILD_RFD || nr == CHILD_RFD_RX_IDLE )//|| nr == CHILD_FFD || nr == CHILD_FFD_RX_IDLE) 
   {
  //  Child_list[i].isActive=true;
   }
   else
   {
  //  Child_list[i].isActive=false;
   }
   Child_list[i].heart_count=0;
 }
#endif

#if !defined ( POWER_SAVING )
 //Init_Watchdog();//�������Ź� 1S����һ�� 
 osal_start_timerEx( SampleApp_TaskID,SAMPLEAPP_FEEDDOG_EVT,500 );//ι���¼� 500ms ι��һ��
#endif

  OUT1_SBIT_FLAG= OUT1_SBIT;
  OUT2_SBIT_FLAG= OUT2_SBIT;
  OUT3_SBIT_FLAG= OUT3_SBIT;
  OUT3_SBIT_FLAG= OUT4_SBIT;
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
        case KEY_CHANGE:      
          SampleApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;                                             

        case AF_INCOMING_MSG_CMD:
          SampleApp_MessageMSGCB( MSGpkt );
          break;
 
         case ZDO_CB_MSG:
          SampleAPP_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
          break; 
          
        case ZDO_STATE_CHANGE:  
       //   P1DIR &= ~0X20;//INput
        //  P1IEN |= 0X20;
          SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (SampleApp_NwkState == DEV_ZB_COORD)
              || (SampleApp_NwkState == DEV_ROUTER)
              || (SampleApp_NwkState == DEV_END_DEVICE) )
          {
        
         #if defined(RTR_NWK)
      
           NLME_PermitJoiningRequest(0);
           
         #endif   
           
           HalLedBlink ( HAL_LED_1, 1, 99, 2500 );//��BLINKģ����������Ϩ��
 
           osal_set_event(SampleApp_TaskID,SAMPLEAPP_SEND_NWKADDR_EVT );//��COMMUNICATION�����豸��Ϣ
           osal_start_timerEx( SampleApp_TaskID,SAMPLEAPP_SEND_PERIODIC_MSG_EVT, 30000 );//������Ϣ   ·���豸��Ӧ�ò������ն��豸����ÿ��POLL��ʱ����
                          
            //���������¼� ��ʱ����Ӵ���һ��
            #if defined(RTR_NWK)
              osal_start_timerEx( SampleApp_TaskID,ASSOCIA_MENTAIN_EVT, 60000 );//2011_10_22 ��ʱ�رմ˹��� ����                  
            #endif
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
//--------------------------
  
  if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT )//����������Ϣ�¼�
  {
    // Send the periodic message
    SampleApp_SendPeriodicMessage();
   
  
    // Setup to send message again in normal period (+ a little jitter)
    osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
        SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT);//+ (osal_rand() & 0x00FF)

    // return unprocessed events
    return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT);
  }
  

//********************** 

/*��COMMUNICTION����������Ϣ*/
  if ( events & SAMPLEAPP_SEND_NWKADDR_EVT )
  {
    SampleApp_SendNwkaddrMessage();

    return ( events ^ SAMPLEAPP_SEND_NWKADDR_EVT);
  }

  
  
   /*Ӧ�ü����¼�*/ 
  if ( events & ZB_ENTRY_EVENT )
  {
    uint8 startOptions;

    zb_ReadConfiguration( ZCD_NV_STARTUP_OPTION, sizeof(uint8), &startOptions );
    if ( startOptions & ZCD_STARTOPT_AUTO_START )
    {
      zb_StartRequest();
    }
    return (events ^ ZB_ENTRY_EVENT );
  }
  
   if ( events & SAMPLEAPP_RESTORE_EVT )
  {
      SystemResetSoft();
       
  }
  
    #if defined(RTR_NWK)
   if ( events & ASSOCIA_MENTAIN_EVT )
  {
    uint8 nr;
    for(uint8 i=0;i<NWK_MAX_DEVICES;i++)
    {
     nr =  AssociatedDevList[i].nodeRelation;
     if (nr == CHILD_RFD || nr == CHILD_RFD_RX_IDLE)// || nr == CHILD_FFD || nr == CHILD_FFD_RX_IDLE)
     {
      Child_list[i].heart_count++;
     }
     
     else
     {
       Child_list[i].heart_count = 0;
     }  
     
    if  (Child_list[i].heart_count>3)// 2�ȽϺ��� ��4���޲��� && (Child_list[i].isActive==true) )// �������3�λ�δ���������Ŀǰ�����ߵ�״̬��
    {  
      // nr =  AssociatedDevList[i].nodeRelation;
       //if (nr == CHILD_RFD || nr == CHILD_RFD_RX_IDLE )
     if (nr == CHILD_RFD || nr == CHILD_RFD_RX_IDLE )//|| nr == CHILD_FFD || nr == CHILD_FFD_RX_IDLE)
      {
       // Child_list[i].isActive=false;
        if(RemoveStaleNode(AssociatedDevList[i].addrIdx))
        {
          Child_list[i].heart_count=0;
         // break;  //test ɾ��һ�������������ȴ���һ��ɾ������һ������ʱ��
        }
    //   AssociatedDevList[i].shortAddr
    //   AssocRemove( byte *extAddr );
      }
     /* 
      else if( nr == CHILD_FFD || nr == CHILD_FFD_RX_IDLE)
      {
        Child_list[i].isActive=false;
       if(RemoveStaleRouter(AssociatedDevList[i].addrIdx))//do noting,because router can,t be removed now
       {
         Child_list[i].heart_count=0;
       }
      }
     */
     }
    }
     osal_start_timerEx( SampleApp_TaskID,ASSOCIA_MENTAIN_EVT, 60000 );  
     return (events ^ ASSOCIA_MENTAIN_EVT );
  } 

  
  
    if ( events & ANNCE_REQ_EVT )
  {
     uint8 ANNCE_REQ_len;
     uint8 *ANNCE_REQ_buf;
     uint8 assi_count;
     uint8 assi_active_count=0;
     assi_count = AssocCount(1,2);//Ŀǰֻ���RFD 
     ANNCE_REQ_len=3+1+2*assi_count;
     uint8 nr;
     if(assi_count>0)
     {
        ANNCE_REQ_buf=osal_mem_alloc(ANNCE_REQ_len); 
        ANNCE_REQ_buf[0]='A';
        ANNCE_REQ_buf[1]='N';
        ANNCE_REQ_buf[2]='R';
        ANNCE_REQ_buf[3]=assi_count;
     
       //���͹��������е�RFD��Ϣ������·����ȥ��Ȼ��·��ƥ�俴�Ƿ�һ�¶̵�ַ��Ϣ�������ANNCE
       for(uint8 i=0;i<NWK_MAX_DEVICES;i++)
       {
        nr =  AssociatedDevList[i].nodeRelation;
        if ( (nr == CHILD_RFD || nr == CHILD_RFD_RX_IDLE) && (AssociatedDevList[i].shortAddr!=0XFFFF) )    
       {       
         ANNCE_REQ_buf[4+(2*assi_active_count)]=AssociatedDevList[i].shortAddr/256;
         ANNCE_REQ_buf[5+(2*assi_active_count)]=AssociatedDevList[i].shortAddr%256;
          assi_active_count++;
       }
      }
     
    
              SampleApp_annce_req_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
              SampleApp_annce_req_DstAddr.endPoint = SAMPLEAPP_ENDPOINT1;
              SampleApp_annce_req_DstAddr.addr.shortAddr = 0xFFFC;
              if ( AF_DataRequest( &SampleApp_annce_req_DstAddr,
                               (endPointDesc_t *)&SampleApp_epDesc1,
                                SAMPLEAPP_ANNCE_REQ_CLUSTERID,
                                ANNCE_REQ_len, ANNCE_REQ_buf,
                                &SampleApp_TransID, 
                                AF_DISCV_ROUTE, 
                                AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
        {
            osal_mem_free( ANNCE_REQ_buf );
          if(ANNCE_REQ_FLAG<=2)
           {
            ANNCE_REQ_FLAG++;
           }
        }
        else
        {
           osal_mem_free( ANNCE_REQ_buf );
        } 
        
        if(ANNCE_REQ_FLAG<2)
        {
         osal_start_timerEx( SampleApp_TaskID,ANNCE_REQ_EVT,30000 );//10s���ڸ�һ��
        }
     
    }
    return ( events ^ ANNCE_REQ_EVT);
  }
   #endif
 
  
  
   if ( events & SAMPLEAPP_FEEDDOG_EVT )
  {
    FeetDog(); //ι��
    osal_start_timerEx( SampleApp_TaskID,SAMPLEAPP_FEEDDOG_EVT,500 );
    return ( events ^ SAMPLEAPP_FEEDDOG_EVT);
  } 

  
  
//**********************     
  // Discard unknown events
  return 0;
}


/*********************************************************************
 * Event Generation Functions
 */
/*********************************************************************
 * @fn      SampleApp_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
//��������������Э��������1�����º��Թ㲥�ķ�ʽ��������ȥ����1С����˸��
void SampleApp_HandleKeys( uint8 shift, uint8 keys )
{
  uint8 startOptions;
 // uint8 logicalType;
  zAddrType_t dstAddr;

  //p1.3
  if ( keys &  HAL_KEY_SW_6 )
  {    
    if(key_double_flag==1)
    { 
     
      HalLedBlink ( HAL_LED_1, 1, 50, 500 );
      if(OUT1_SBIT ==1)
      {
        OUT1_SBIT =0;
      }
      else
      {
       OUT1_SBIT =1;
      }
      
    if( (devState==DEV_END_DEVICE) || (devState==DEV_ROUTER) )
       {
         if(OUT1_SBIT_FLAG !=OUT1_SBIT)
         {
          SampleApp_SendInMessage();
         }
          OUT1_SBIT_FLAG = OUT1_SBIT;
       }
      
    }
    

    if(key_double_flag==3)
    { 
      if( (devState==DEV_END_DEVICE) || (devState==DEV_ROUTER) )
    {
     dstAddr.addrMode = afAddr16Bit;
     dstAddr.addr.shortAddr = 0;   // Coordinator makes the match 
     ZDP_EndDeviceBindReq( &dstAddr, NLME_GetShortAddr(),
                             SAMPLEAPP_ENDPOINT1,
                             0x0F08,
                             1, bindingINClusters,   // No incoming clusters to bind   0, NULL
                             0, NULL,
                             FALSE );
     
    }
      
    }
    
  else if(key_double_flag==4)
    {
     
       zb_ReadConfiguration( ZCD_NV_STARTUP_OPTION, sizeof(uint8), &startOptions );
       startOptions = ZCD_STARTOPT_AUTO_START;
       zb_WriteConfiguration( ZCD_NV_STARTUP_OPTION, sizeof(uint8), &startOptions );
       HAL_SYSTEM_RESET();  
     }

    if(key_hold_flag)
    {
      key_hold_flag=0;
      HalLedBlink ( HAL_LED_1, 4, 50, 500 );
      startOptions=0;    
      BindSetDefaultNV();
      osal_nv_item_init(ZCD_NV_STARTUP_OPTION, 1, &startOptions);    
      startOptions |= ZCD_STARTOPT_DEFAULT_NETWORK_STATE;
      startOptions &= ~ZCD_STARTOPT_AUTO_START;
      osal_nv_write(ZCD_NV_STARTUP_OPTION, 0, 1, &startOptions);
      osal_start_timerEx( SampleApp_TaskID,SAMPLEAPP_RESTORE_EVT,2500 );
    }
    
   }
  
    //P1.5  
   else if ( keys &  HAL_KEY_SW_7 )
  { 
    
   if( key_double_P1_5_flag == 1)
   {
      HalLedBlink ( HAL_LED_2, 1, 50, 500 );
      
       if(OUT2_SBIT==1)
      {
        OUT2_SBIT =0;
      }
      
      else
      {
       OUT2_SBIT =1;
      }
      
   if( (devState==DEV_END_DEVICE) || (devState==DEV_ROUTER) )
       {
         if(OUT2_SBIT_FLAG !=OUT2_SBIT)
         {
          SampleApp_SendInMessage();
         }
          OUT2_SBIT_FLAG = OUT2_SBIT;
       }
   }
    
   else if( key_double_P1_5_flag == 3)
    {
         if( (devState==DEV_END_DEVICE) || (devState==DEV_ROUTER) )
      {
       dstAddr.addrMode = afAddr16Bit;
       dstAddr.addr.shortAddr = 0;   // Coordinator makes the match
       
       ZDP_EndDeviceBindReq( &dstAddr, NLME_GetShortAddr(),
                               SAMPLEAPP_ENDPOINT2,
                               0x0F08,
                               1, bindingINClusters,   // No incoming clusters to bind   0, NULL
                               0, NULL,
                               FALSE );
       
      }
    }
  }
  
   //P1.6 
   else if ( keys &  HAL_KEY_SW_8 )
  { 
    
   if( key_double_P1_6_flag == 1)
   {
      HalLedBlink ( HAL_LED_3, 1, 50, 500 );
       if(OUT3_SBIT ==1)
      {
        OUT3_SBIT =0;
      }
      
      else
      {
       OUT3_SBIT =1;
      }
      
    if( (devState==DEV_END_DEVICE) || (devState==DEV_ROUTER) )
       {
         if(OUT3_SBIT_FLAG !=OUT3_SBIT)
         {
          SampleApp_SendInMessage();
         }
          OUT3_SBIT_FLAG = OUT3_SBIT;
       }
   }
    
   else if( key_double_P1_6_flag == 3)
    {
     if( (devState==DEV_END_DEVICE) || (devState==DEV_ROUTER) )
      {
       dstAddr.addrMode = afAddr16Bit;
       dstAddr.addr.shortAddr = 0;   // Coordinator makes the match
       
       ZDP_EndDeviceBindReq( &dstAddr, NLME_GetShortAddr(),
                               SAMPLEAPP_ENDPOINT3,
                               0x0F08,
                               1, bindingINClusters,   // No incoming clusters to bind   0, NULL
                               0, NULL,
                               FALSE );
       
      }
    }
  }

  
      //P1.7 
   else if ( keys &  HAL_KEY_SW_9 )
  { 
    
   if( key_double_P1_7_flag == 1)
   {
      HalLedBlink ( HAL_LED_4, 1, 50, 500 );
       if(OUT4_SBIT ==1)
      {
        OUT4_SBIT =0;
      }
      
      else
      {
       OUT4_SBIT =1;
      }
      
    if( (devState==DEV_END_DEVICE) || (devState==DEV_ROUTER) )
       {
         if(OUT4_SBIT_FLAG !=OUT4_SBIT)
         {
          SampleApp_SendInMessage();
         }
          OUT4_SBIT_FLAG = OUT4_SBIT;
       }
   }
    
   else if( key_double_P1_7_flag == 3)
    {
         if( (devState==DEV_END_DEVICE) || (devState==DEV_ROUTER) )
      {
       dstAddr.addrMode = afAddr16Bit;
       dstAddr.addr.shortAddr = 0;   // Coordinator makes the match
       
       ZDP_EndDeviceBindReq( &dstAddr, NLME_GetShortAddr(),
                               SAMPLEAPP_ENDPOINT4,
                               0x0F08,
                               1, bindingINClusters,   // No incoming clusters to bind   0, NULL
                               0, NULL,
                               FALSE );
       
      }
    }
  }

}

void SampleAPP_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg )
{
 
  switch ( inMsg->clusterID )
  {
    case End_Device_Bind_rsp:
      if ( ZDO_ParseBindRsp( inMsg ) == ZSuccess )
      {
        // Light LED
       HalLedBlink ( HAL_LED_1, 2, 50, 1000 );
      }
#if defined(BLINK_LEDS)
      else
      {
        HalLedBlink ( HAL_LED_1, 6, 50, 500 );
      }
#endif
      break;
  }

}




void SampleApp_ProcessBINDINGMessage(afIncomingMSGPacket_t *pkt)
{
   if(pkt->endPoint == SAMPLEAPP_ENDPOINT1)
  {
  // HalLedBlink ( HAL_LED_1, 1, 50, 500 );
    
   if(pkt->cmd.Data[0]==1)
   {
     OUT1_SBIT=1;
   }
   
   else if(pkt->cmd.Data[0]==0)
   {
     OUT1_SBIT=0;
   }

  }
  
  else if(pkt->endPoint == SAMPLEAPP_ENDPOINT2)
  {
  //  HalLedBlink ( HAL_LED_2, 1, 50, 500 );
   if(pkt->cmd.Data[0]==1)
   {
     OUT2_SBIT=1;
   }
   else if(pkt->cmd.Data[0]==0)
   {
     OUT2_SBIT=0;
   }

  }
  
    else if(pkt->endPoint == SAMPLEAPP_ENDPOINT3)
  {
   // HalLedBlink ( HAL_LED_3, 1, 50, 500 );
    
   if(pkt->cmd.Data[0]==1)
   {
     OUT3_SBIT=1;
   }
   else if(pkt->cmd.Data[0]==0)
   {
     OUT3_SBIT=0;
   }

  }
  
   else if(pkt->endPoint == SAMPLEAPP_ENDPOINT4)
  {
   // HalLedBlink ( HAL_LED_4, 1, 50, 500 );
    
   if(pkt->cmd.Data[0]==1)
   {
     OUT4_SBIT=1;
   }
   else if(pkt->cmd.Data[0]==0)
   {
     OUT4_SBIT=0;
   }

  }
  
   if( (OUT2_SBIT !=OUT2_SBIT_FLAG) || (OUT1_SBIT !=OUT1_SBIT_FLAG) || (OUT3_SBIT !=OUT3_SBIT_FLAG) || (OUT4_SBIT !=OUT4_SBIT_FLAG))
 { 
    OUT2_SBIT_FLAG = OUT2_SBIT;
    OUT1_SBIT_FLAG = OUT1_SBIT;
    OUT3_SBIT_FLAG = OUT3_SBIT;
    OUT4_SBIT_FLAG = OUT4_SBIT;
    
    if( (devState==DEV_END_DEVICE) || (devState==DEV_ROUTER) )
         SampleApp_SendInMessage();
 }

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
      
       /* 3 �� 7�� ��Ӵ��� */
     case SAMPLEAPP_ADDR_CLUSTERID: //�豸��������Ϣ����

      SampleApp_ProcessNwkaddrMessage(pkt);

      break; 
      
      
     case SAMPLEAPP_ONOFF_CLUSTERID:   
      
      SampleApp_ProcessBINDINGMessage(pkt);
    break; 
      
    
     case SAMPLEAPP_ANNCE_REQ_CLUSTERID:
   #if defined(RTR_NWK)
    if( (pkt->cmd.Data[0]=='A') && (pkt->cmd.Data[1]=='N') && (pkt->cmd.Data[2]=='R'))
    { 
      //ZDApp_AnnounceNewAddress();  
      ZDApp_PROESS_AnnounceAddress_CONFLICT(pkt);  
    }
    #endif
   break; 
   
     case SAMPLEAPP_PERIODIC_CLUSTERID:
#if defined(RTR_NWK)
      SampleApp_ROUTER_ProcessPeriodicMessage_Data(pkt);
#endif
      break;  
      
  default:break;
    
  }
}

/*********************************************************************
 * @fn      SampleApp_SendPeriodicMessage
 *
 * @brief   Send the periodic message.
 *
 * @param   none
 *
 * @return  none
 */

//����������Ϣ
void SampleApp_SendPeriodicMessage( void )
{
  /*##########################################*/
  uint8 buffer[4];
  uint8 *Sample_Cmd_shorAddr;
  uint16 parent_short_adress;
  
  Sample_Cmd_shorAddr=SampleApp_GetShortAddr();
  buffer[0]=*Sample_Cmd_shorAddr++;
  buffer[1]=*Sample_Cmd_shorAddr++;
  buffer[2]=*Sample_Cmd_shorAddr++;
  buffer[3]=*Sample_Cmd_shorAddr;
  
  osal_mem_free( shortddr_mem );
  #if defined(RTR_NWK)
    parent_short_adress=0X0000;//�����·�ɵĻ���ֱ�ӷ���Э����
  #else
     parent_short_adress= NLME_GetCoordShortAddr();//������ն˵Ļ���ת�������ڵ�get the parent adress
  #endif
    
  SampleApp_Periodic_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  SampleApp_Periodic_DstAddr.endPoint = SAMPLEAPP_ENDPOINT1;
 // SampleApp_Periodic_DstAddr.addr.shortAddr = 0X0000;
  SampleApp_Periodic_DstAddr.addr.shortAddr = parent_short_adress;
    
  if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc1,
                       SAMPLEAPP_PERIODIC_CLUSTERID,
                       4,  //���ݳ���
                       buffer,//###��ӵķ�������        
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  }
  else
  {
    // Error occurred in request to send.
  }
}


/* 3 7 ���*/
void SampleApp_ProcessNwkaddrMessage(afIncomingMSGPacket_t *pkt)
{
   
   uint16 short_adress=0;
   uint8 *ieee_adress;
  
   
     #ifdef ZDO_COORDINATOR
    
      uint8 NWK_ADDR_STR[30];

      NWK_ADDR_STR[0]='O';
      NWK_ADDR_STR[1]='S';
      NWK_ADDR_STR[2]=':';
      NWK_ADDR_STR[3]=pkt->cmd.Data[0]+'0';  //Type;
      NWK_ADDR_STR[4]=(pkt->cmd.Data[1]/10)+'0'; //App_Type Ӧ���������ӵ�2λ
      NWK_ADDR_STR[5]=(pkt->cmd.Data[1]%10)+'0'; //App_Type Ӧ���������ӵ�2λ
      
      for(uint8 i=0;i<4;i++)
      {
      NWK_ADDR_STR[6+i]=pkt->cmd.Data[2+i];  //get the short adress
      }
      
      for(uint8 j=0;j<16;j++)
      {
      NWK_ADDR_STR[10+j]=pkt->cmd.Data[6+j];  //get the ieee adress
      }
      
      NWK_ADDR_STR[26]=':';
      NWK_ADDR_STR[27]='N';
      NWK_ADDR_STR[28]=0x0D;
      NWK_ADDR_STR[29]=0x0A;
      HalUARTWrite ( MT_UART_DEFAULT_PORT , NWK_ADDR_STR, 30 );
      
     #else
      SampleApp_Periodic_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
      SampleApp_Periodic_DstAddr.endPoint = SAMPLEAPP_ENDPOINT1;
      SampleApp_Periodic_DstAddr.addr.shortAddr = 0X0000;  //give the coodinator
    //  SampleApp_Periodic_DstAddr.addr.shortAddr = parent_short_adress;//test
      
      if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc1,
                         SAMPLEAPP_ADDR_CLUSTERID,
                         23,  //���ݳ���
                         pkt->cmd.Data,//###��ӵķ�������        
                         &SampleApp_TransID,
                         AF_DISCV_ROUTE,
                         AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
      {
      }
      else
      {
      // Error occurred in request to send.
      }
      #endif
    
      
   short_adress = BUILD_UINT16 (wordtohex(pkt->cmd.Data[4],pkt->cmd.Data[5]),wordtohex(pkt->cmd.Data[2],pkt->cmd.Data[3]));
      
   /* 
    //���������������Ƿ�����
    uint8 loss_flag =1;//Ĭ�϶�ʧ
    uint8 nr=0;
    AddrMgrEntry_t addrEntry;
    
    for(uint8 j=0;j<NWK_MAX_DEVICES;j++)
    {
     nr =  AssociatedDevList[j].nodeRelation;
     if (nr == CHILD_RFD || nr == CHILD_RFD_RX_IDLE)// || nr == CHILD_FFD || nr == CHILD_FFD_RX_IDLE)    
     {
      if(AssociatedDevList[j].shortAddr==short_adress)
      {
        loss_flag=0;
        break;
      }
     }
    }

    //ֻ�����ն��豸
      if ( (loss_flag == 1) && (pkt->cmd.Data[0]==1))
      {
         for(uint8 i=0;i<8;i++)
        {
         ieee_adress[i] = wordtohex(pkt->cmd.Data[6+(2*i)],pkt->cmd.Data[7+(2*i)]);
        }
          AssocAddNew( short_adress, ieee_adress,  CHILD_RFD );
          AssocWriteNV();     
          addrEntry.user = ADDRMGR_USER_DEFAULT;
          addrEntry.nwkAddr = short_adress;
          osal_cpyExtAddr(addrEntry.extAddr, ieee_adress);
          AddrMgrEntryUpdate( &addrEntry );
          ZDApp_NVUpdate();
      }
    */
    
     
    //�����յ�OS�������������豸������ȷ�� ���¿�һ��CLUSTRE������
    //֡��ʽΪ12345.  ���� SAMPLEAPP_FLASH_CLUSTERID  ֻ���ն��豸��������
   
  
  
  if (  pkt->cmd.Data[0]==1)
 {  
    
  uint8 osack[3]={1,2,3};
  uint8 nr=0;
  
  SampleApp_Flash_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;//
  SampleApp_Flash_DstAddr.endPoint = SAMPLEAPP_ENDPOINT1;//
  SampleApp_Flash_DstAddr.addr.shortAddr = short_adress;//
    
  if ( AF_DataRequest( &SampleApp_Flash_DstAddr, &SampleApp_epDesc1,
                         SAMPLEAPP_FLASH_CLUSTERID,
                         3,  //���ݳ���
                         osack,//###��ӵķ�������        
                         &SampleApp_TransID,
                         AF_DISCV_ROUTE,
                         AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
      {
      }
      else
      {
      // Error occurred in request to send.
      }
  
  for(uint8 j=0;j<NWK_MAX_DEVICES;j++)
    {
       nr =  AssociatedDevList[j].nodeRelation;
     if (nr == CHILD_RFD || nr == CHILD_RFD_RX_IDLE)// || nr == CHILD_FFD || nr == CHILD_FFD_RX_IDLE)    
     {
      if(AssociatedDevList[j].shortAddr==short_adress)
      {
         Child_list[j].heart_count=0; 
       //  Child_list[j].isActive=true;//�յ����������ߵ�
         break;
      }
     }
    }
  
    
   }
    
   
}

/*********************************************************************
 * @fn      SampleApp_GetExtendAddr()
 *
 * @brief   get the entendaddr
 *
 * @param   none
 *
 * @return  uin8 *my_extendaddr
 */
uint8 *SampleApp_GetExtendAddr(void)
{
  uint8 i;
  uint8 *ext_addr;
  uint8 *extaddr;
//  uint8 *extendaddr_mem;
  
  extendaddr_mem=osal_mem_alloc(16);
  ext_addr=NLME_GetExtAddr();
  for(i=0;i<15;i=i+2)
  {
    extendaddr_mem[i]=hextoword1(*ext_addr);
    extendaddr_mem[i+1]=hextoword2(*ext_addr++);
  }
  extaddr=extendaddr_mem;
  return extaddr;
}
//����δ����������
/*********************************************************************
 * @fn      SampleApp_GetExtendAddr_MAC()
 *
 * @brief   get the entendaddr
 *
 * @param   none
 *
 * @return  uin8 extendaddr
 */
/*
uint8 *SampleApp_GetExtendAddr_MAC(void)
{
  uint8 i;
  uint8 Pextaddr_buffer[8];
  uint8 *Pextaddr;
  uint8 *extaddr_mac;
  
  extendaddr_mem=osal_mem_alloc(16);
  ZMacGetReq( ZMacExtAddr, Pextaddr_buffer );
//MAC_MlmeGetReq( MAC_EXTENDED_ADDRESS, &Pextaddr_buffer[0] );
  Pextaddr=&Pextaddr_buffer[0];
  for(i=0;i<15;i=i+2)
  {
    extendaddr_mac_mem[i]=hextoword1(*Pextaddr);
    extendaddr_mac_mem[i+1]=hextoword2(*Pextaddr++);
  }
  extaddr_mac=extendaddr_mem;
  return extaddr_mac;
}
*/
/*********************************************************************
 * @fn      SampleApp_GetShortAddr()
 *
 * @brief   get the short address
 *
 * @param   none
 *
 * @return  uin8 *shortddr
 */

uint8 *SampleApp_GetShortAddr(void)
{
  uint16 short_ddr;
  uint8 *shortddr;
  uint8 yy1;
  uint8 yy2; 
//  uint8 *shortddr_mem;
  
  shortddr_mem=osal_mem_alloc(4); 
  short_ddr=NLME_GetShortAddr();
  yy1=(uint8)((short_ddr&0xff00)>>8);
  yy2=(uint8)short_ddr;
  shortddr_mem[0]=hextoword1(yy1);
  shortddr_mem[1]=hextoword2(yy1);
  shortddr_mem[2]=hextoword1(yy2);
  shortddr_mem[3]=hextoword2(yy2);
  shortddr=shortddr_mem;
  
  return shortddr;
}


/*********************************************************************
 * @fn      SampleApp_SendNwkaddrMessage
 *
 * @brief   Send the network address message to communication.
 *
 * @param   none �������ͨѶ�豸������ͨѶ�豸��������ĳ����̵�ַ��ͨѶ�豸�����ڿ��Ǽ����豸���͵ȡ�
 *
 * @return  none ���ݽṹ ��1�ֹ�Ϊ�豸���� �ڶ��ֽ�Ϊ�豸�ͺ� ��4�ֽ�Ϊ�̵�ַ ��16�ֽ�Ϊ����ַ ���һ�ֽ�Ϊ��ID  
 */
void SampleApp_SendNwkaddrMessage(void)
{

//  extern uint8 *shortddr_mem;
  uint8 *Sample_myshort;
  uint8 *Sample_extendaddr;
  uint8 str[23];
  uint8 i;
  uint16 parent_short_adress=0;
  
#if defined(RTR_NWK)
  str[0]=Send_Router;
#else
  str[0]=Send_Endvice;
#endif  
  
  str[1]=TOUCH_4_ID; 

  
  Sample_myshort=SampleApp_GetShortAddr();
  str[2]=*Sample_myshort++;
  str[3]=*Sample_myshort++;
  str[4]=*Sample_myshort++;
  str[5]=*Sample_myshort;
  osal_mem_free( shortddr_mem );
  
  Sample_extendaddr=SampleApp_GetExtendAddr();
  for(i=0;i<16;i++)
  {
    str[i+6]=*Sample_extendaddr++;
  }
   osal_mem_free( extendaddr_mem );
 

  
   /*2012 3 7 ����� �ն˵�������Ϣ���������豸*/
    #if defined(RTR_NWK)
    parent_short_adress=0X0000;//�����·�ɵĻ���ֱ�ӷ���Э����
   #else
     parent_short_adress= NLME_GetCoordShortAddr();//������ն˵Ļ���ת�������ڵ�get the parent adress
   #endif
    
  SampleApp_Addr_SendData_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  SampleApp_Addr_SendData_DstAddr.endPoint = SAMPLEAPP_ENDPOINT1;
 // SampleApp_Periodic_DstAddr.addr.shortAddr = 0X0000;
  SampleApp_Addr_SendData_DstAddr.addr.shortAddr = parent_short_adress;
  /*�޸����*/  
  
  if ( AF_DataRequest( &SampleApp_Addr_SendData_DstAddr,
                         (endPointDesc_t *)&SampleApp_epDesc1,
                          SAMPLEAPP_ADDR_CLUSTERID ,
                          23, str,
                          &SampleApp_TransID, 
                          AF_DISCV_ROUTE, 
                          AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  }
  else
  {   
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
  uint8 str[40];
  uint8 *Sample_Cmd_shorAddr;
  uint8 *Sample_extendaddr;
  uint16 dimmer_temp;
  uint8 data_sucess_flag=0;
  uint8 active_count=0;
  
  Sample_Cmd_shorAddr=SampleApp_GetShortAddr();
  str[0]=*Sample_Cmd_shorAddr++;
  str[1]=*Sample_Cmd_shorAddr++;
  str[2]=*Sample_Cmd_shorAddr++;
  str[3]=*Sample_Cmd_shorAddr;
  str[4]='/';
  str[5]='C';
  str[6]='/';
  
    
   str[7]=pkt->cmd.Data[6];//�����豸��������
   Sent_t=str[7];
  
  //test ��ȡ��������Ϣ���͵�����
   if(pkt->cmd.Data[8]==97)//a:97 ��ȡ������
   {
     uint8 assi_count;
     uint8 active_assi_count=0;
     uint8 total_count=0;
     data_sucess_flag=2;
     uint8 *SET_str;
     
   #if defined(RTR_NWK)
     assi_count = AssocCount( (pkt->cmd.Data[13]-0x30), (pkt->cmd.Data[14]-0x30) );//get the assiot count 
     total_count=12+(assi_count*2);
    // SET_str=osal_mem_alloc(10+2+(assi_count*2));//2Ϊ��ȡ�ĸ����Լ�ʵ�ʵĸ��� assioct_countΪ�̵�ַ����
     SET_str=osal_mem_alloc(total_count);
     SET_str[0]=str[0];
     SET_str[1]=str[1];
     SET_str[2]=str[2];
     SET_str[3]=str[3];
     SET_str[4]='/';
     SET_str[5]='C';
     SET_str[6]='/';
     SET_str[7]='U';
     SET_str[8]='U';
     SET_str[9]=0xA0; //TEST #��ʾΪ�����������͸���Ļ��ơ�ES�� :N
     SET_str[10]=3;
     SET_str[11]=assi_count;
     for(uint8 i=0;i<NWK_MAX_DEVICES;i++)
     {
     if(AssociatedDevList[i].nodeRelation!=0xff)
      {   
       
        //TRANSET THE SHORT ADRESS
        SET_str[12+(2*active_assi_count)]=HI_UINT16(AssociatedDevList[i].shortAddr);
        SET_str[13+(2*active_assi_count)]=LO_UINT16(AssociatedDevList[i].shortAddr);
        active_assi_count++;
      }
     }
     
   #else
     uint16 parent_short_adress;
     assi_count=1;
     total_count=14;
     
     SET_str=osal_mem_alloc(total_count);
     SET_str[0]=str[0];
     SET_str[1]=str[1];
     SET_str[2]=str[2];
     SET_str[3]=str[3];
     SET_str[4]='/';
     SET_str[5]='C';
     SET_str[6]='/';
     SET_str[7]='U';
     SET_str[8]='U';
     SET_str[9]=0xA0;
     SET_str[10]=3;
     SET_str[11]=assi_count;
     parent_short_adress=NLME_GetCoordShortAddr();
     SET_str[12]=HI_UINT16(parent_short_adress);
     SET_str[13]=LO_UINT16(parent_short_adress);
 #endif
     
     
      if(data_sucess_flag==2)
      {    
        SampleApp_SPI_SendCommand_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
        SampleApp_SPI_SendCommand_DstAddr.endPoint = SAMPLEAPP_ENDPOINT1;
       // SampleApp_SPI_SendCommand_DstAddr.addr.shortAddr = 0X0000;
        SampleApp_SPI_SendCommand_DstAddr.addr.shortAddr = pkt->srcAddr.addr.shortAddr;//���͸������豸
  
        if ( AF_DataRequest( &SampleApp_SPI_SendCommand_DstAddr,
                       (endPointDesc_t *)&SampleApp_epDesc1,
                        SAMPLEAPP_COMMAND_CLUSTERID, //�ն˴���ȥ��Э����������
                        total_count, SET_str,
                        &SampleApp_TransID, 
                        AF_DISCV_ROUTE, 
                        AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
       {
        osal_mem_free(SET_str);
       }
      else
       {
         osal_mem_free(SET_str);
       }  
      }
     //osal_mem_free(SET_str);
    }
  
 
  //test ��ȡ��ʵ��RSSIֵ����
  else if(pkt->cmd.Data[8]=='f')//f
   {
     uint8 *SET_str;
     data_sucess_flag=3;
     SET_str=osal_mem_alloc(12);
     SET_str[0]=str[0];
     SET_str[1]=str[1];
     SET_str[2]=str[2];
     SET_str[3]=str[3];
     SET_str[4]='/';
     SET_str[5]='C';
     SET_str[6]='/';
     SET_str[7]='U';
     SET_str[8]='U';
     SET_str[9]=0xA0;
     SET_str[10]=2;
     SET_str[11]=ABS(pkt->rssi);//get the rssi
      if(data_sucess_flag==3)
     {    
        SampleApp_SPI_SendCommand_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
        SampleApp_SPI_SendCommand_DstAddr.endPoint = SAMPLEAPP_ENDPOINT1;
       // SampleApp_SPI_SendCommand_DstAddr.addr.shortAddr = 0X0000;
      SampleApp_SPI_SendCommand_DstAddr.addr.shortAddr = pkt->srcAddr.addr.shortAddr;//���͸������豸
 
        if ( AF_DataRequest( &SampleApp_SPI_SendCommand_DstAddr,
                       (endPointDesc_t *)&SampleApp_epDesc1,
                        SAMPLEAPP_COMMAND_CLUSTERID, //�ն˴���ȥ��Э����������
                        12, SET_str,
                        &SampleApp_TransID, 
                        AF_DISCV_ROUTE, 
                        AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
       {
           osal_mem_free(SET_str);
       }
      else
       {
            osal_mem_free(SET_str);
       }  
   }
  }
  
   #if defined(RTR_NWK)
  else if(pkt->cmd.Data[8]=='V')
    {
     uint8 permit_time;
     uint8 permit_str[12];
      
    permit_time=((pkt->cmd.Data[13]-'0')*100) + ((pkt->cmd.Data[14]-'0')*10) + (pkt->cmd.Data[15]-'0');
  
     permit_str[0]=str[0];
     permit_str[1]=str[1];
     permit_str[2]=str[2];
     permit_str[3]=str[3];
     permit_str[4]='/';
     permit_str[5]='C';
     permit_str[6]='/';
     permit_str[7]='U';
     permit_str[8]='U';
     permit_str[9]=0xA0;
     permit_str[10]=1;
     permit_str[11]=permit_time;//get the rssi
  
     NLME_PermitJoiningRequest(permit_time);
   
     SampleApp_SPI_SendCommand_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
     SampleApp_SPI_SendCommand_DstAddr.endPoint = SAMPLEAPP_ENDPOINT1;
       // SampleApp_SPI_SendCommand_DstAddr.addr.shortAddr = 0X0000;
     SampleApp_SPI_SendCommand_DstAddr.addr.shortAddr = pkt->srcAddr.addr.shortAddr;//���͸������豸
     if ( AF_DataRequest( &SampleApp_SPI_SendCommand_DstAddr,
                       (endPointDesc_t *)&SampleApp_epDesc1,
                        SAMPLEAPP_COMMAND_CLUSTERID, //�ն˴���ȥ��Э����������
                        12, permit_str,
                        &SampleApp_TransID, 
                        AF_DISCV_ROUTE, 
                        AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
       {
       }
      else
       {   
       } 
    }
  
     else if(pkt->cmd.Data[8]=='W')//W:87 ·���豸��ֹ���� 
    {
     uint8 permit_str[12];
     permit_str[0]=str[0];
     permit_str[1]=str[1];
     permit_str[2]=str[2];
     permit_str[3]=str[3];
     permit_str[4]='/';
     permit_str[5]='C';
     permit_str[6]='/';
     permit_str[7]='U';
     permit_str[8]='U';
     permit_str[9]=0xA0;
     permit_str[10]=1;
     permit_str[11]=0;//get the rssi
  
     NLME_PermitJoiningRequest(0);
   
     SampleApp_SPI_SendCommand_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
     SampleApp_SPI_SendCommand_DstAddr.endPoint = SAMPLEAPP_ENDPOINT1;
       // SampleApp_SPI_SendCommand_DstAddr.addr.shortAddr = 0X0000;
     SampleApp_SPI_SendCommand_DstAddr.addr.shortAddr = pkt->srcAddr.addr.shortAddr;//���͸������豸
     if ( AF_DataRequest( &SampleApp_SPI_SendCommand_DstAddr,
                       (endPointDesc_t *)&SampleApp_epDesc1,
                        SAMPLEAPP_COMMAND_CLUSTERID, //�ն˴���ȥ��Э����������
                        12, permit_str,
                        &SampleApp_TransID, 
                        AF_DISCV_ROUTE, 
                        AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
       {
       }
      else
       {
    
       } 
    }
   #endif
   
  
  //�����豸������  ����������
 if(pkt->cmd.Data[8]==118)//v:118
 {
    data_sucess_flag=1;
   
   if(pkt->cmd.Data[13]=='1' )//��һ���̵���
   { 
       str[9]='/';//2011 11 �����µ�Э�飬�������豸ȫ���ԡ�\��������
       if(pkt->cmd.Data[14]=='1')
       {
         OUT1_SBIT=1;//��
         str[10]=1;
         str[11]=OUT1_SBIT;
       }
       
       else if(pkt->cmd.Data[14]=='0')
       {
         OUT1_SBIT=0;//�ر�
         str[10]=1;
         str[11]=OUT1_SBIT;
       }  
       
       else if(pkt->cmd.Data[14]=='2')
       {
        str[10]=1;
        str[11]=OUT1_SBIT;
       }  
      else
      {
        str[10]=0xFF;
        str[11]=0xFF;
      }
   }
   
   else
   {
       str[9]='/'; 
       str[10]=0xFF;
       str[11]=0xFF;
   }
   
  if(pkt->cmd.Data[15]=='2' )//��
  {  
    if(pkt->cmd.Data[16]=='1')
       {
         OUT2_SBIT=1;//��
         str[12]=2;
         str[13]=OUT2_SBIT;
       }
    
   else if(pkt->cmd.Data[16]=='0')
       {
         OUT2_SBIT=0;//�ر�
         str[12]=2;
         str[13]=OUT2_SBIT;
       }
    
  else if(pkt->cmd.Data[16]=='2')
       {
        str[12]=2;
        str[13]=OUT2_SBIT;
       }
    
  else
      {
        str[12]=0xFF;
        str[13]=0xFF;
      }
   }
   
   else
   {
       str[12]=0xFF;
       str[13]=0xFF;
   }
   
 if(pkt->cmd.Data[17]=='3' )//��
   {  
    if(pkt->cmd.Data[18]=='1')
       {
         OUT3_SBIT=1;//��
         str[14]=3;
         str[15]=OUT3_SBIT;
       }
    
      else if(pkt->cmd.Data[18]=='0')
      {
          OUT3_SBIT=0;//�ر�
          str[14]=3;
          str[15]=OUT3_SBIT;
      }
     
        else if(pkt->cmd.Data[18]=='2')
       {
        str[14]=3;
        str[15]=OUT3_SBIT;
       }
    
   else
      {
        str[14]=0xFF;
        str[15]=0xFF;
      }
   }
   
   else
   {
       str[14]=0xFF;
       str[15]=0xFF;
   }
   
   
    if(pkt->cmd.Data[19]=='4' )//��
   {  
    if(pkt->cmd.Data[20]=='1')
       {
         OUT4_SBIT=1;//��
         str[16]=4;
         str[17]=OUT4_SBIT;
       }
    
      else if(pkt->cmd.Data[20]=='0')
      {
          OUT4_SBIT=0;//�ر�
          str[16]=4;
          str[17]=OUT4_SBIT;
      }
     
        else if(pkt->cmd.Data[20]=='2')
       {
        str[16]=4;
        str[17]=OUT4_SBIT;
       }
    
   else
      {
        str[16]=0xFF;
        str[17]=0xFF;
      }
   }
   
   else
   {
       str[16]=0xFF;
       str[17]=0xFF;
   }
   
   active_count=18;
 }
 
 else if(pkt->cmd.Data[8]==82)//R:86 ˢ���豸�б� ���⴦��ֱ�Ӱ�һ����ʽ�ϱ�
 {
   str[9]='R';
   Command_t=str[9];
//   uint8 str[23];
   uint8 i;
#if defined(RTR_NWK)
  str[10]=Send_Router;
#else
  str[10]=Send_Endvice;
#endif  
  
  str[11]=TOUCH_4_ID; 
  

  Sample_extendaddr=SampleApp_GetExtendAddr();
  for(i=0;i<16;i++)
  {
    str[i+12]=*Sample_extendaddr++;
  }
   osal_mem_free( extendaddr_mem );
 
 
  str[28]=0;

  str[29]=OUT1_SBIT;
  str[30]=OUT2_SBIT;
  str[31]=OUT3_SBIT;
  str[32]=OUT4_SBIT;
  
  data_sucess_flag=1;
 
  active_count =33;

 }
 //�����ڴ˴�������������������
  
  osal_mem_free( shortddr_mem );
  
//---------------------------------    
  if((pkt->cmd.Data[7]==85) && data_sucess_flag==1)//command: /C/XU start single data transport ���η���
  {
    str[8]='U';//���η���
    SampleApp_SPI_SendCommand_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
    SampleApp_SPI_SendCommand_DstAddr.endPoint = SAMPLEAPP_ENDPOINT1;
   // SampleApp_SPI_SendCommand_DstAddr.addr.shortAddr = 0X0000;
     SampleApp_SPI_SendCommand_DstAddr.addr.shortAddr = pkt->srcAddr.addr.shortAddr;//���͸������豸

    if ( AF_DataRequest( &SampleApp_SPI_SendCommand_DstAddr,
                         (endPointDesc_t *)&SampleApp_epDesc1,
                          SAMPLEAPP_COMMAND_CLUSTERID, //�ն˴���ȥ��Э����������
                          active_count, str,
                          &SampleApp_TransID, 
                          AF_DISCV_ROUTE, 
                          AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
    {
//      HalLedBlink( HAL_LED_4, 4, 50, (1000 / 4) ); //test
    }
    else
    {
      
    }   
  }

}



#if defined(RTR_NWK)
/*
�����ַ��ͻ����
0 1 2   3     4 5     6 7
A N R COUNT  DATA1  DATA2
*/
void ZDApp_PROESS_AnnounceAddress_CONFLICT(afIncomingMSGPacket_t *pkt)
{
   uint16 Child_short_adress;
   uint8  adress_count;
   uint16  *conflict_address; //�����豸�����ĵ�ַ��
  // uint16  associt_adress;//�����豸�������ڵĵ�ַ
   uint8   associt_ExtAddr[8];
   uint8 nr;
   adress_count=pkt->cmd.Data[3];
   conflict_address=osal_mem_alloc(adress_count*2); 
   for(uint8 a=0;a<adress_count;a++)  
   {
    conflict_address[a]=BUILD_UINT16(pkt->cmd.Data[5+(2*a)],pkt->cmd.Data[4+(2*a)]);//get the short adress of the fomer children adress
   }
   
    
  for(uint8 i=0;i<NWK_MAX_DEVICES;i++)
  {
   nr =  AssociatedDevList[i].nodeRelation;
   if ( (nr == CHILD_RFD || nr == CHILD_RFD_RX_IDLE) && (AssociatedDevList[i].shortAddr!=0XFFFF) )    
     {    
       for(uint8 j=0;j<adress_count;j++)
       {
        if(AssociatedDevList[i].shortAddr==conflict_address[j])//if find the conflict adress message
        {
           if( AddrMgrExtAddrLookup( AssociatedDevList[i].shortAddr, associt_ExtAddr ))
           {
             Delay_1u(100);
             ZDP_DeviceAnnce( AssociatedDevList[i].shortAddr, associt_ExtAddr,
                           0, 0 );//�ݲ�����capabilities Ĭ��Ϊ0
           }
        }
       }  
    }
  }
      osal_mem_free( conflict_address );
}


/*
1 ת�������������շ���COORDINATOR
2 �����������������������ACTIVE OR INACTIVE��CHIRDREN
*/
void SampleApp_ROUTER_ProcessPeriodicMessage_Data(afIncomingMSGPacket_t *pkt)
{
  
  uint8 HEART_BUFFER[4];
  uint8 Child_short_adress_buffer[4];
 // uint8 *Sample_Cmd_shorAddr;
  uint8 shortAddr_H8;
  uint8 shortAddr_L8;
  uint16 Child_short_adress;
  uint16 parent_short_adress;
  uint8 nr;
  
  for(uint8 i=0;i<4;i++)
  {
   HEART_BUFFER[i]=pkt->cmd.Data[i];//get the heart message
  }
  
    for(uint8 i=0;i<4;i++)
  {//ת���̵�ַ ��ACII�뵽UINT16
    if((pkt->cmd.Data[i]>47)&&(pkt->cmd.Data[i]<58))      //0~9  
      Child_short_adress_buffer[i]=pkt->cmd.Data[i]-48;
    else if((pkt->cmd.Data[i]>64)&&(pkt->cmd.Data[i]<71)) //A~F
      Child_short_adress_buffer[i]=pkt->cmd.Data[i]-55;
  }
    shortAddr_H8=(Child_short_adress_buffer[0]<<4)+Child_short_adress_buffer[1];
    shortAddr_L8=(Child_short_adress_buffer[2]<<4)+Child_short_adress_buffer[3];
    Child_short_adress=((uint16)(shortAddr_H8)<<8)+((uint16)shortAddr_L8);
    
    parent_short_adress= NLME_GetShortAddr();//NLME_GetCoordShortAddr();//get the parent adress
    
    if( (parent_short_adress!=Child_short_adress) && (parent_short_adress!=0x0000)) //if it is not itself and it is not the coordinator
    {
      SampleApp_Periodic_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
      SampleApp_Periodic_DstAddr.endPoint = SAMPLEAPP_ENDPOINT1;
      SampleApp_Periodic_DstAddr.addr.shortAddr = 0X0000;  //give the coodinator
    //  SampleApp_Periodic_DstAddr.addr.shortAddr = parent_short_adress;//test
      
      if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc1,
                         SAMPLEAPP_PERIODIC_COORDINATOR_CLUSTERID,
                         4,  //���ݳ���
                         HEART_BUFFER,//###��ӵķ�������        
                         &SampleApp_TransID,
                         AF_DISCV_ROUTE,
                         AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
      {
      }
      else
      {
      // Error occurred in request to send.
      }
    }
    
    //���������������Ƿ�����
    
    for(uint8 j=0;j<NWK_MAX_DEVICES;j++)
    {
       nr =  AssociatedDevList[j].nodeRelation;
     if (nr == CHILD_RFD || nr == CHILD_RFD_RX_IDLE)// || nr == CHILD_FFD || nr == CHILD_FFD_RX_IDLE)    
     {
      if(AssociatedDevList[j].shortAddr==Child_short_adress)
      {
         Child_list[j].heart_count=0; 
       //  Child_list[j].isActive=true;//�յ����������ߵ�
         break;
      }
     }
    }
}
#endif

void SampleApp_SendInMessage(void)
{

  uint8 str[20];
  uint8 *Sample_Cmd_shorAddr;

  Sample_Cmd_shorAddr=SampleApp_GetShortAddr();
  str[0]=*Sample_Cmd_shorAddr++;
  str[1]=*Sample_Cmd_shorAddr++;
  str[2]=*Sample_Cmd_shorAddr++;
  str[3]=*Sample_Cmd_shorAddr;
  osal_mem_free( shortddr_mem );

  str[4]='/';
  str[5]='/';
  str[6]='/';
  str[7]='/';
  str[8]='/';
  str[9]='/';//�����豸���Ͷ���ͬ��S��ʾ���ء�H��ʾ���� Ŀǰ�ֶ�����
 
 if (OUT1_SBIT==0)
 {
   str[10]=1;
   str[11]=0;//��һ������  
 }
 else if (OUT1_SBIT==1)
 {
   str[10]=1;
   str[11]=1;//��һ�����ر�
 }
 
 
  if(OUT2_SBIT==0)
 {
   str[12]=2;  
   str[13]=0;    //�ڶ�������
 }
 else if(OUT2_SBIT==1)
 {
   str[12]=2;  
   str[13]=1;   //�ڶ������ر�
 }
 
  if(OUT3_SBIT==0)
 {
   str[14]=3;  
   str[15]=0;    //�ڶ�������
 }
 else if(OUT3_SBIT==1)
 {
   str[14]=3;  
   str[15]=1;   //�ڶ������ر�
 }
 
  if(OUT4_SBIT==0)
 {
   str[16]=4;  
   str[17]=0;    //�ڶ�������
 }
 
 else if(OUT4_SBIT==1)
 {
   str[16]=4;  
   str[17]=1;   //�ڶ������ر�
 }
  
  
  SampleApp_SPI_SendCommand_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  SampleApp_SPI_SendCommand_DstAddr.endPoint = SAMPLEAPP_ENDPOINT1;
  SampleApp_SPI_SendCommand_DstAddr.addr.shortAddr = 0X0000;
    
    if ( AF_DataRequest( &SampleApp_SPI_SendCommand_DstAddr,
                         (endPointDesc_t *)&SampleApp_epDesc1,
                          SAMPLEAPP_COMMAND_CLUSTERID, //�ն˴���ȥ��Э����������
                          18, str,
                          &SampleApp_TransID, 
                          AF_DISCV_ROUTE, 
                          AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
    {
      
    }
    else
    {
      
    }   
}




/*********************************************************************
 * @fn      hextoword1/2
 *
 * @brief   translate the hex to word.
 *
 * @param   uint8 t
 *
 * @return  none
 */
//ʮ������ת�ַ�����
uint8 hextoword1(uint8 t )
{
  uint8 abc;
  uint8 cba;
  uint8 xx1;
  abc=t;
  cba=0xf0;
  abc=(abc&cba)>>4;
  if(abc<10)
  {
    xx1=abc+48;
  }
  else
  {
    xx1=abc+55;
  }
  return xx1;
}

uint8 hextoword2(uint8 t)
{
  uint8 abc;
  uint8 cba;
  uint8 xx2;
  abc=t;
  cba=0x0f;
  abc=abc&cba;
  if(abc<10)
  {
    xx2=abc+48;
  }
  else
  {
    xx2=abc+55;
  }
  return xx2;
}


uint8 wordtohex(uint8 x,uint8 y)
{
 uint8 z;
 if(x>0x39)
   {x=x-0x37;}		//���16�����е�A B C D E F ASICII��A��Ӧ16���Ƶ�0X41 B��Ӧ0X42   ���� ת��Ϊ16����ʱӦ�ü�ȥ0X37  (0X41-0X0A)
 else
   {x=x-0x30;}		 //ASICII��0~9  ��Ӧ��ʮ������Ϊ0X30~0X39    ��ȥ0X30  ���õ�ת��
 if(y>0x39)
  {y=y-0x37;}
  else
   {y=y-0x30;}
 x=x<<4;			    //�ŵ���λ�ϣ���Ϊ232ͨ�ŵķ�Χ������ǰ���w r : n��������0~F��������֮�� ��ת����16���ƺ������Ϊ4λ�����ֽ�
 x=x&0xf0;
 z=y|x;
 return(z);
}

/****************************************************************
*�������ܣ���ʼ����Դ					
*��ڲ�����para1,para2,para3,para4			
*����ֵ	����						
*˵  ��	��para1,ģʽѡ��						
*																*
* para1  0 	1	2	3											*
* mode 	PM0	PM1	PM2	PM3											*
*																*
****************************************************************/
void PowerMode(uint8 sel)
{
	uint8 i,j;
	i = sel;
	if(sel<4)
	{
		SLEEPCMD &= 0xfc;
		SLEEPCMD |= i;
		for(j=0;j<4;j++);
		PCON = 0x01;//˯��
	}
	else
	{
	    PCON = 0x00;//����
	}
}



//remove the stale nodes
uint8 RemoveStaleNode(uint8 index)
{
  AddrMgrEntry_t addrEntry;
  NLME_LeaveReq_t req;

  // Set up device info

  addrEntry.user  = ADDRMGR_USER_DEFAULT;
  addrEntry.index = index;

  if (AddrMgrEntryGet( &addrEntry )) 
  {
    // Remove device
    req.extAddr = addrEntry.extAddr;
    req.removeChildren = TRUE;
    req.rejoin = FALSE;
    req.silent = FALSE;

    NLME_LeaveReq( &req );
    
  // AssocRemove(addrEntry.extAddr); //remove this device
  // AddrMgrEntryRelease( &addrEntry );
    NLME_RemoveChild( addrEntry.extAddr, true );
   //AssocWriteNV(); 
  //  ZDApp_NVUpdate();
   return true;
  }
  return false;
}


//remove the stale router
uint8 RemoveStaleRouter(uint8 index)
{
  AddrMgrEntry_t addrEntry;
  NLME_LeaveReq_t req;
  APSME_RemoveDeviceReq_t remDevReq;
  // Set up device info

  addrEntry.user  = ADDRMGR_USER_DEFAULT;
  addrEntry.index = index;

  if (AddrMgrEntryGet( &addrEntry )) 
  {
    // Remove device
    req.extAddr = addrEntry.extAddr;
    req.removeChildren = FALSE;
    req.rejoin = FALSE;
    req.silent = TRUE;

    NLME_LeaveReq( &req );
    
 // AssocRemove(addrEntry.extAddr); //remove this device
 // AddrMgrEntryRelease( &addrEntry );
   //AssocWriteNV();                 //save the NV
   //ZDApp_NVUpdate();
   
  //  remDevReq.parentAddr   = NLME_GetShortAddr();
  //  remDevReq.childExtAddr = addrEntry.extAddr;
  //  APSME_RemoveDeviceReq( &remDevReq );
  // NLME_RemoveChild( addrEntry.extAddr, false );
   return true;
  }
  return false;
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
/***************************************

******************************/




/******************************************************************************
 * @fn          zb_ReadConfiguration
 *
 * @brief       The zb_ReadConfiguration function is used to get a
 *              Configuration Protperty from Nonvolatile memory.
 *
 * @param       configId - The identifier for the configuration property
 *              len - The size of the pValue buffer in bytes
 *              pValue - A buffer to hold the configuration property
 *
 * @return      none
 */
uint8 zb_ReadConfiguration( uint8 configId, uint8 len, void *pValue )
{
  uint8 size;

  size = (uint8)osal_nv_item_len( configId );
  if ( size > len )
  {
    return ZFailure;
  }
  else
  {
    return( osal_nv_read(configId, 0, size, pValue) );
  }
}

/******************************************************************************
 * @fn          zb_WriteConfiguration
 *
 * @brief       The zb_WriteConfiguration function is used to write a
 *              Configuration Property to nonvolatile memory.
 *
 * @param       configId - The identifier for the configuration property
 *              len - The size of the pValue buffer in bytes
 *              pValue - A buffer containing the new value of the
 *                       configuration property
 *
 * @return      none
 */
uint8 zb_WriteConfiguration( uint8 configId, uint8 len, void *pValue )
{
  return( osal_nv_write(configId, 0, len, pValue) );
}

/******************************************************************************
 * @fn          zb_StartRequest
 *
 * @brief       The zb_StartRequest function starts the ZigBee stack.  When the
 *              ZigBee stack starts, the device reads configuration parameters
 *              from Nonvolatile memory and the device joins its network.  The
 *              ZigBee stack calls the zb_StartConrifm callback function when
 *              the startup process completes.
 *
 * @param       none
 *
 * @return      none
 */
void zb_StartRequest()
{
 // uint8 logicalType1;
  uint8 zgStartDelay = START_DELAY;
/*
  zb_ReadConfiguration( ZCD_NV_LOGICAL_TYPE, sizeof(uint8), &logicalType1 );

  // Check for bad combinations of compile flag definitions and device type setting.
  if ((logicalType1 > ZG_DEVICETYPE_ENDDEVICE)      ||
#if !ZG_BUILD_ENDDEVICE_TYPE   // Only RTR or Coord possible.
      (logicalType1 == ZG_DEVICETYPE_ENDDEVICE)     ||
#endif
#if !ZG_BUILD_RTR_TYPE         // Only End Device possible.
      (logicalType1 == ZG_DEVICETYPE_ROUTER)        ||
      (logicalType1 == ZG_DEVICETYPE_COORDINATOR)   ||
#elif ZG_BUILD_RTRONLY_TYPE    // Only RTR possible.
      (logicalType1 == ZG_DEVICETYPE_COORDINATOR)   ||
#elif !ZG_BUILD_JOINING_TYPE   // Only Coord possible.
      (logicalType1 == ZG_DEVICETYPE_ROUTER)        ||
#endif
      (0))
  {
    logicalType1 = ZB_INVALID_PARAMETER;
    SAPI_SendCback(SAPICB_START_CNF, logicalType1, 0);
  }
  else*/
 // {
 //   logicalType1 = ZB_SUCCESS;
    devState = DEV_INIT;
    ZDOInitDevice(zgStartDelay);
  
 // }

  
  return;
}
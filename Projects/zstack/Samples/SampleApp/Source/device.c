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
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
#include "OSAL.h"
#include "ZGlobals.h"
#include "AF.h"
#include "aps_groups.h"
#include "ZDApp.h"
#include "OSAL_Nv.h"

#include "device.h"


#include <math.h>

#include "OnBoard.h"

#include "MT_UART.h"//2430 是 SPIMGR  2530叫 MT_UART  换了个名字
#include "mac_rx.h"

/* HAL */
#include "hal_led.h"
#include "hal_key.h"
#include "OSAL_Nv.h"

#include "AddrMgr.h"
#include "nwk_util.h"

#if (defined CO2)
#include "sh100.h"
#endif

#if (defined AIRCONIDTION) || (defined CURTAIN)
#include "irr.h"
#endif

#if (defined TEMPHUME) 
#include "sht10.h"
#endif



/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
    uint8  rxlen;//接收数据长度
    uint8 *databuf;//接收数据块指针
    uint8 *shortddr_mem;  
    uint8 *extendaddr_mem;
    uint8 *channelist_mem;

	uint8 Command_t,Sent_t,Report_Device_type;
	uint16 ReportTimeTotal_s = 0;//等待总时间 单位：秒
	uint16 ReportTime_s  = 0;//秒
	uint16 ReportTime_mi = 0;//分
	uint8  Load_ReportTime_flag = 0;
    uint8 Sent_type;//20110528
    uchar SHT100RxData[7]= {0,0,0,0,0,0,0};
    uint16 n,m;
	//uint16 test[600],test1[600];
    uint8 ACStatue = '0'; //空调状态： '1' 空调开机状态，'0'空调待机状态
    uint8 CTStatue = '0'; //窗帘状态： '1' 窗帘开机状态，'0'窗帘待机状态

	uint32 sysTimeDelayStart = 0;
	uint32 sysTime = 0;
	
    bool returnState;
 //   uint16 Communication_saddr;
    
    //关于关联表
#if defined(RTR_NWK)
  Child_device_t Child_list[21];
#endif
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES—全局变量
 */

const cId_t SampleApp_ClusterList[SAMPLEAPP_MAX_CLUSTERS] =
{
  SAMPLEAPP_SPIDATA_CLUSTERID, //周期信息簇ID=1
  SAMPLEAPP_FLASH_CLUSTERID,    //闪烁信息簇ID=2
  SAMPLEAPP_ADDR_CLUSTERID,
  SAMPLEAPP_COMMAND_CLUSTERID,
  SAMPLEAPP_PERIODIC_CLUSTERID,
  SAMPLEAPP_ANNCE_REQ_CLUSTERID,
  SAMPLEAPP_PERIODIC_COORDINATOR_CLUSTERID,
  SAMPLEAPP_ACTIVE_COMMAND_CLUSTERID
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

endPointDesc_t SampleApp_epDesc;

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



afAddrType_t SampleApp_Periodic_DstAddr;       //周期事件寻址模式
afAddrType_t SampleApp_SPI_SendData_DstAddr;   //串口数据信息
afAddrType_t SampleApp_SPI_SendCommand_DstAddr;//串口命令信息
afAddrType_t SampleApp_Addr_SendData_DstAddr;  //入网报告
afAddrType_t SampleApp_annce_req_DstAddr;    //annce 关联表对比清楚信息




uint8 SampleAppPeriodicCounter = 0;
uint8 SampleAppFlashCounter = 0;
uint16 dst_short_ddr=0;

//关于启动
//static uint8 myAppState = APP_INIT;
 uint8 logicalType1=0;
// Application States


/*********************************************************************
 * LOCAL FUNCTIONS
 */
void SampleApp_HandleKeys( uint8 shift, uint8 keys );//本用户应用按键函数
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );//本用户应用信息回调函数
void SampleApp_SendPeriodicMessage( void );         //本用户应用发送周期消息函数





void SampleApp_SendNwkaddrMessage(void);
void SampleApp_Reflash_NwkMessage(void);
void SampleApp_ProcessCommandMessage(afIncomingMSGPacket_t *pkt);
void SampleApp_PrcoessCommandPeriodicMessage(void);
void SampleApp_SendInMessage(void);


uint8 *SampleApp_GetShortAddr(void);
uint8 *SampleApp_GetExtendAddr(void);

uint8 *SampleApp_GetExtendAddr_MAC(void);
uint8 hextoword1(uint8 t );
uint8 hextoword2(uint8 t);
uint8 wordtohex(uint8 x,uint8 y);
void SampleApp_ReportTimeExpend(void); //20110622
void SampleApp_ROUTER_ProcessPeriodicMessage_Data(afIncomingMSGPacket_t *pkt);

uint8 zb_ReadConfiguration( uint8 configId, uint8 len, void *pValue );
uint8 zb_WriteConfiguration( uint8 configId, uint8 len, void *pValue );
void zb_StartRequest ( void );
uint8 RemoveStaleNode(uint8 index);


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


  
  // Fill out the endpoint description.
  SampleApp_epDesc.endPoint = SAMPLEAPP_ENDPOINT;//SampleApp EP描述符的EP号：20
  SampleApp_epDesc.task_id = &SampleApp_TaskID;//SampleApp EP描述符的任务ID：0
  SampleApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&SampleApp_SimpleDesc;//SampleApp EP简单描述符
  SampleApp_epDesc.latencyReq = noLatencyReqs;//延时策略

  
  // Register the endpoint description with the AF
  afRegister( &SampleApp_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( SampleApp_TaskID ); 
  
  osal_set_event(task_id, ZB_ENTRY_EVENT); 
  
  #if !defined ( POWER_SAVING )
   Init_Watchdog();//启动看门狗 1S启动一次 
   osal_start_timerEx( SampleApp_TaskID,SAMPLEAPP_FEEDDOG_EVT,500 );//喂狗事件 500ms 喂狗一次
  #endif
   
  //根据关联表初始化子设备表 
 #if defined(RTR_NWK)
 uint8 nr;
 for(uint8 i=0;i<NWK_MAX_DEVICES;i++)
 { 
   nr =  AssociatedDevList[i].nodeRelation;
   if (nr == CHILD_RFD || nr == CHILD_RFD_RX_IDLE || nr == CHILD_FFD || nr == CHILD_FFD_RX_IDLE) 
   {
   // Child_list[i].isActive=true;
   }
   else
   {
  //  Child_list[i].isActive=false;
   }
   Child_list[i].heart_count=0;
 }
#endif
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
  afDataConfirm_t *afDataConfirm;
 // uint8 sentStatus;
  
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
 
         case AF_DATA_CONFIRM_CMD:
          // This message is received as a confirmation of a data packet sent.
          // The status is of ZStatus_t type [defined in ZComDef.h]
          // The message fields are defined in AF.h
          afDataConfirm = (afDataConfirm_t *)MSGpkt;
        //  sentEP = afDataConfirm->endpoint;
          sentStatus = afDataConfirm->hdr.status;
        //  sentTransID = afDataConfirm->transID;
       //   (void)sentEP;
       //   (void)sentTransID;
       // Action taken when confirmation is received.
          if ( sentStatus != ZSuccess )
          {
            // The data wasn't delivered -- Do something
            // Ok then, I'll do something like re-send my device announce!
          }
          break;   
  
        case ZDO_STATE_CHANGE:  

          SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (SampleApp_NwkState == DEV_ZB_COORD)
              || (SampleApp_NwkState == DEV_ROUTER)
              || (SampleApp_NwkState == DEV_END_DEVICE) )
          {
	 //  HAL_TURN_ON_LED1();//test
         //  osal_start_timerEx( SampleApp_TaskID,TURN_OFF_LED1_EVT,2000 );//2s 以后关闭led1 ,加网成功
         HalLedBlink ( HAL_LED_1, 4, 99, 2000 );//用BLINK模拟灯亮两秒后熄灭

            
          //需要添加根据各种不同设备设置不同POLL时间的命令
         // NLME_SetPollRate( POLL_RATE );//set the poll rate to defult
	// NLME_SetQueuedPollRate( QUEUED_POLL_RATE );//set the Queued poll rate to defult
        // NLME_SetResponseRate( RESPONSE_POLL_RATE );//set the Response rate to defult
		
        osal_set_event(SampleApp_TaskID,SAMPLEAPP_SEND_NWKADDR_EVT );//向COMMUNICATION报告设备信息                    
        osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT,1000 );//心跳信息 
                        
       //处理路由设备的信息，包括设置应许加入功能以及关联表处理事件 ，关联表包括关联表定时维护以及复位后的ANNCE申请 
        #if defined(RTR_NWK)
           NLME_PermitJoiningRequest(0);   
           osal_start_timerEx( SampleApp_TaskID,ASSOCIA_MENTAIN_EVT, 30000 );
           osal_start_timerEx( SampleApp_TaskID,ANNCE_REQ_EVT,500 );//复位后请求ANNCE信息，以防关联表没清除
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
      //释放消息占用的内存
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next - if one is available
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    }//end： while ( MSGpkt )

    return (events ^ SYS_EVENT_MSG);
  }

  //发送周期消息事件
  if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT )
  {
    SampleApp_SendPeriodicMessage();
    osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT, SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT);
    return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT);
  }
  

//发送OS消息，上线消息
  if ( events & SAMPLEAPP_SEND_NWKADDR_EVT )
  {
    SampleApp_SendNwkaddrMessage();
    return ( events ^ SAMPLEAPP_SEND_NWKADDR_EVT);
  }

//处理周期性的命令事件，周期性返回的设备
  if ( events & SAMPLEAPP_COMMAND_PERIODIC_MSG_EVT )
  {
    SampleApp_PrcoessCommandPeriodicMessage();
    return ( events ^ SAMPLEAPP_COMMAND_PERIODIC_MSG_EVT);
  }    

//周期回传数据时 消耗等待时间 由于osal_start_timerEx()函数只能进行65535ms的延时，当超过该值时就需要调理
  if ( events & SAMPLEAPP_REPORTTIME_EXPEND_EVT )
  {
    SampleApp_ReportTimeExpend();
    return ( events ^ SAMPLEAPP_REPORTTIME_EXPEND_EVT);
  }    
 
 //应用加网事件
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

  //硬复位事件
   if ( events & SAMPLEAPP_RESTORE_EVT )
  {
    HAL_SYSTEM_RESET();

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
     
    if  (Child_list[i].heart_count>=6)// && (Child_list[i].isActive==true) )// 如果超过3次还未被清掉，且目前是在线的状态下
    {  
      // nr =  AssociatedDevList[i].nodeRelation;
     if (nr == CHILD_RFD || nr == CHILD_RFD_RX_IDLE )//|| nr == CHILD_FFD || nr == CHILD_FFD_RX_IDLE)
      {
       // Child_list[i].isActive=false;
        if(RemoveStaleNode(AssociatedDevList[i].addrIdx))
        {
          Child_list[i].heart_count=0;
          break;  //test 删除一个后先跳出，等待下一次删除另外一个。错开时间
        }
      }
     }
    }
     osal_start_timerEx( SampleApp_TaskID,ASSOCIA_MENTAIN_EVT, 30000 );  
     return (events ^ ASSOCIA_MENTAIN_EVT );
  } 
  
  
   if ( events & ANNCE_REQ_EVT )
  {
     uint8 ANNCE_REQ_len;
     uint8 *ANNCE_REQ_buf;
     uint8 assi_count;
     uint8 assi_active_count=0;
     assi_count = AssocCount(1,2);//目前只针对RFD 
     ANNCE_REQ_len=3+1+2*assi_count;
     uint8 nr;
     if(assi_count>0)
     {
        ANNCE_REQ_buf=osal_mem_alloc(ANNCE_REQ_len); 
        ANNCE_REQ_buf[0]='A';
        ANNCE_REQ_buf[1]='N';
        ANNCE_REQ_buf[2]='R';
        ANNCE_REQ_buf[3]=assi_count;
     
       //传送关联表里有的RFD信息到所有路由中去，然后路由匹配看是否一致短地址信息，如果有ANNCE
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
              SampleApp_annce_req_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
              SampleApp_annce_req_DstAddr.addr.shortAddr = 0xFFFC;
              if ( AF_DataRequest( &SampleApp_annce_req_DstAddr,
                               (endPointDesc_t *)&SampleApp_epDesc,
                                SAMPLEAPP_ANNCE_REQ_CLUSTERID,
                                ANNCE_REQ_len, ANNCE_REQ_buf,
                                &SampleApp_TransID, 
                                AF_DISCV_ROUTE, 
                                AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
        {
          osal_mem_free( ANNCE_REQ_buf );
        
        }
        else
        {
           osal_mem_free( ANNCE_REQ_buf );
        }    
    }
    return ( events ^ ANNCE_REQ_EVT);
  }
   #endif
  
#if !defined ( POWER_SAVING )
    if ( events & SAMPLEAPP_FEEDDOG_EVT )
  {
    FeetDog(); //喂狗
    osal_start_timerEx( SampleApp_TaskID,SAMPLEAPP_FEEDDOG_EVT,500 );
    return ( events ^ SAMPLEAPP_FEEDDOG_EVT);
  }   
#endif
  
  
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
//按键处理函数（当协调器按键1被按下后，以广播的方式发送数据去让组1小灯闪烁）
void SampleApp_HandleKeys( uint8 shift, uint8 keys )
{
  uint8 startOptions;

  if ( keys &  HAL_KEY_SW_8 )
  {    
    if(key_double_flag==1)
    {
      #if defined(BUTTON)//BUTTON  ZZZZZZZ
        if((devState == DEV_END_DEVICE) && (SET_ALARM==1))
         SampleApp_SendInMessage();  //test
      #endif
      
      #if defined(SOKET)//SOKET  
        if(OUT1_SBIT==1)
        {
          OUT1_SBIT = 0; 
          HAL_TURN_ON_LED2();//on led
        }
        else if(OUT1_SBIT==0)
        {
          OUT1_SBIT = 1; 
          HAL_TURN_OFF_LED2();//on led
        }
         if( (devState==DEV_END_DEVICE) || (devState==DEV_ROUTER) )
          SampleApp_SendInMessage();
      #endif 
    }
    
    else if(key_double_flag==2)
    {
      #if defined(BUTTON)//BUTTON  ZZZZZZZ
      if((devState == DEV_END_DEVICE) && (SET_ALARM==1))
       SampleApp_SendInMessage();  //test
      #endif
    }
    
    else if(key_double_flag==3)
    {
    // HAL_SYSTEM_RESET(); 
    }
    else if(key_double_flag==4)
    {
      // 申请加入网络    
      osal_nv_item_init(ZCD_NV_STARTUP_OPTION, 1, &startOptions); 
      startOptions = ZCD_STARTOPT_AUTO_START;
      osal_nv_write(ZCD_NV_STARTUP_OPTION, 0, 1, &startOptions);
      
      HAL_SYSTEM_RESET();  
    }
    else if(key_double_flag==5)
    {
	  //NLME_SetPollRate(500);

    }
    else if(key_double_flag==6)
    {
	  //NLME_SetPollRate(15000);
    }
    else if(key_double_flag==7)
    {
		//NwkPollReq(0);
    }
    if(key_hold_flag)
    {
      key_hold_flag=0;
      HalLedBlink ( HAL_LED_1, 4, 50, 500 );
      startOptions=0;    
      osal_nv_item_init(ZCD_NV_STARTUP_OPTION, 1, &startOptions);    
      startOptions |= ZCD_STARTOPT_DEFAULT_NETWORK_STATE;
      startOptions &= ~ZCD_STARTOPT_AUTO_START;
      osal_nv_write(ZCD_NV_STARTUP_OPTION, 0, 1, &startOptions);
      osal_start_timerEx( SampleApp_TaskID,SAMPLEAPP_RESTORE_EVT,2500 );
    }
    
  #if defined(SENDIN) //ZZZZZZZ  SENDIN表示需要本地上报的设备，通过宏来操作
  if ( (keys &  HAL_KEY_SW_9) || (keys &  HAL_KEY_SW_6) )
  { 
   if( (devState==DEV_END_DEVICE) || (devState==DEV_ROUTER) )
    SampleApp_SendInMessage();
  }
   #endif
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
  switch ( pkt->clusterId )
  {
    case SAMPLEAPP_COMMAND_CLUSTERID:   
      SampleApp_ProcessCommandMessage(pkt);
    break;     
     
    #if defined(RTR_NWK)
    case SAMPLEAPP_ANNCE_REQ_CLUSTERID:
     
      if( (pkt->cmd.Data[0]=='A') && (pkt->cmd.Data[1]=='N') && (pkt->cmd.Data[2]=='R'))
      { 
        //ZDApp_AnnounceNewAddress();  
        ZDApp_PROESS_AnnounceAddress_CONFLICT(pkt);  
      }    
     break; 
     
     case SAMPLEAPP_PERIODIC_CLUSTERID:
      SampleApp_ROUTER_ProcessPeriodicMessage_Data(pkt);
     break; 
    #endif
      
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

//发送周期消息
void SampleApp_SendPeriodicMessage( void )
{
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
    parent_short_adress=0X0000;//如果是路由的话则直接发给协调器
   #else
     parent_short_adress= NLME_GetCoordShortAddr();//如果是终端的话先转发给父节点get the parent adress
   #endif
    
  SampleApp_Periodic_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  SampleApp_Periodic_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Periodic_DstAddr.addr.shortAddr = parent_short_adress;
    
  if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_PERIODIC_CLUSTERID,
                       4,  //数据长度
                       buffer,//###添加的发送数据        
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
//以下未定！！！～
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
 * @param   none 如果不是通讯设备，则向通讯设备发送自身的长、短地址给通讯设备，后期考虑加入设备类型等。
 *
 * @return  none 数据结构 第1字節为设备类型 第二字节为设备型号 再4字节为短地址 再16字节为长地址 最后一字节为组ID  
 */
void SampleApp_SendNwkaddrMessage(void)
{

//  extern uint8 *shortddr_mem;
  uint8 *Sample_myshort;
  uint8 *Sample_extendaddr;
  uint8 str[23];
  uint8 i;
#if defined(RTR_NWK)
  str[0]=Send_Router;
#else
  str[0]=Send_Endvice;
#endif  
  
  //ZZZZZ需要添加其他设备，如电子栅栏等
 #if defined(ALARM)
  str[1]=1;
 #elif defined(PIR)
  str[1]=2;
 #elif defined(MAGNETISM)
  str[1]=3; 
 #elif defined(BUTTON)
  str[1]=4; 

  #elif defined(SMOKEDETECT)
  str[1]=7; 
  #elif defined(CH4)
  str[1]=9; 
  
  #elif defined(SWITH)
  str[1]=11; 
  #elif defined(DIMMER)
  str[1]=12; 

  #elif defined(SOCKET)
  str[1]=16; 
  #elif defined(TEMPHUME)
  str[1]=17; 
  #elif defined(CO2)
  str[1]=18;  
 
 #elif defined(AIRCONIDTION)
  str[1]=21; 
 #elif defined(TELEVISION)
  str[1]=22;
 #elif defined(CURTAIN)
  str[1]=23; 
 #else
  str[1]=0; //大概可以分为开关，插座，告警，显示型等等，
              //可以根据智能家居的标准参考，可以通过定义不同的C文件的头文件来识别
 #endif 

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
  
  SampleApp_Addr_SendData_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  SampleApp_Addr_SendData_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Addr_SendData_DstAddr.addr.shortAddr = 0X0000;
    
  if ( AF_DataRequest( &SampleApp_Addr_SendData_DstAddr,
                         (endPointDesc_t *)&SampleApp_epDesc,
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

//ZZZZZZ  重点修改
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
//  uint8 buf[]="SIGL_data";
 // uint8 i;
  uint8 *Sample_Cmd_shorAddr;
  uint8 *Sample_extendaddr;
  uint8 data_sucess_flag=0;
#if defined (AIRCONIDTION)   
  
  uint8 ACCmdType = 0;
  uint8 CmdType;  //命令类型： '1' 学习，'2' 使用
  uint8 ACType;   //空调类型： '1' A类空调，遥控器端键码不固定；'2' B类空调，遥控器端键码固定
//  uint8 ACStatue; //空调状态： '1' 空调开机状态，'0'空调待机状态
  uint8 OperativeMode;  //空调运行模式
  uint8 ACTemp;         //空调温度
  uint8 ACTempL;        //空调温度下限
  uint8 ACTempH;        //空调温度上限
#endif
#if defined (CURTAIN)    
  uint8 CTCmdType = 0;
  uint8 CmdType;  //命令类型： '1' 学习，'2' 使用
  uint8 CTType;   //空调类型： '1' A类空调，遥控器端键码不固定；'2' B类空调，遥控器端键码固定
//  uint8 CTStatue; //空调状态： '1' 空调开机状态，'0'空调待机状态
  uint8 OperativeMode;  //窗帘运行模式
  uint8 ACTemp;         //空调温度
  uint8 ACTempL;        //空调温度下限
  uint8 ACTempH;        //空调温度上限
#endif
  

  
  Sample_Cmd_shorAddr=SampleApp_GetShortAddr();
  str[0]=*Sample_Cmd_shorAddr++;
  str[1]=*Sample_Cmd_shorAddr++;
  str[2]=*Sample_Cmd_shorAddr++;
  str[3]=*Sample_Cmd_shorAddr;
  str[4]='/';
  str[5]='C';
  str[6]='/';
  
 // shortAddr_H8= pkt->cmd.Data[1];
 // shortAddr_L8= pkt->cmd.Data[2]; 
  //Communication_saddr=((uint16)(shortAddr_H8)<<8)+((uint16)shortAddr_L8);
    
  /*
 	pkt->cmd.Data[]与buf[]中的数据一一对应，buf[]数据是协调器收到的上位机串口数据，协调器将此数据发给终端设备 
  	buf[0] 为数据长度，buf[1],buf[2]为源地址高8位 低8位 
    buf[3]开始为接收到的上位机数据
    buf[3]=/,buf[4]=C,buf[5]=/,buf[6]=发送类型 U B G,buf[7]=数据返回类型 U P,buf[8]=设备类型，如H为温湿度
    buf[9,10,11,12]设备短地址...... buf[]与终端设备接收到的空中数据pkt->cmd.Data[]一一对应
    buf[]的详细信息参看MT_UART.C中rxCB()函数。
   */
   str[7]=pkt->cmd.Data[6];//处理设备发送类型
   Sent_t=str[7];
   
  #if defined(RTR_NWK)
   //XXX/C/AUVFFFCXXX  XXX/C/UUVXXXX XXX   
   uint8 permit_time;
  if(pkt->cmd.Data[8]==86)//V:86 路由设备应许加入 
  {
    str[9]='V';

    permit_time=((pkt->cmd.Data[13]-'0')*100) + ((pkt->cmd.Data[14]-'0')*10) + (pkt->cmd.Data[15]-'0');
    NLME_PermitJoiningRequest(permit_time);
    str[10]=permit_time;
    str[11]=0;
    str[12]=0;
    data_sucess_flag=1;
  }
   else if(pkt->cmd.Data[8]==87)//W:87 路由设备禁止加入 
   {
     str[9]='W';
    NLME_PermitJoiningRequest(0);
    str[10]=0;
    str[11]=0;
    str[12]=0;
    data_sucess_flag=1;
   }
  #endif
   
  
  //test 读取关联表信息发送到网关
   if(pkt->cmd.Data[8]==97)//a:97 读取关联表
   {
 
     uint8 assi_count;
     uint8 active_assi_count=0;
     uint8 *assi_str;
     uint8 total_count=0;
     data_sucess_flag=2;
     
   #if defined(RTR_NWK)
     assi_count = AssocCount( (pkt->cmd.Data[13]-0x30), (pkt->cmd.Data[14]-0x30) );//get the assiot count 
     total_count=10+2+(assi_count*2);
    // assi_str=osal_mem_alloc(10+2+(assi_count*2));//2为获取的个数以及实际的个数 assioct_count为短地址个数
     assi_str=osal_mem_alloc(total_count);
     assi_str[0]=str[0];
     assi_str[1]=str[1];
     assi_str[2]=str[2];
     assi_str[3]=str[3];
     assi_str[4]='/';
     assi_str[5]='C';
     assi_str[6]='/';
     assi_str[7]='U';
     assi_str[8]='U';
     assi_str[9]='a';
     assi_str[10]=assi_count;
     for(uint8 i=0;i<NWK_MAX_DEVICES;i++)
     {
     if(AssociatedDevList[i].nodeRelation!=0xff)
      {   
       
        //TRANSET THE SHORT ADRESS
        assi_str[11+(2*active_assi_count)]=HI_UINT16(AssociatedDevList[i].shortAddr);
        assi_str[12+(2*active_assi_count)]=LO_UINT16(AssociatedDevList[i].shortAddr);
         active_assi_count++;
      }
     }
      assi_str[total_count-1]=active_assi_count;
     
   #else
     uint16 parent_short_adress;
     assi_count=1;
     total_count=12+2;
     
     assi_str=osal_mem_alloc(total_count);
     assi_str[0]=str[0];
     assi_str[1]=str[1];
     assi_str[2]=str[2];
     assi_str[3]=str[3];
     assi_str[4]='/';
     assi_str[5]='C';
     assi_str[6]='/';
     assi_str[7]='U';
     assi_str[8]='U';
     assi_str[9]='a';
     assi_str[10]=assi_count;
     parent_short_adress=NLME_GetCoordShortAddr();
     assi_str[11]=HI_UINT16(parent_short_adress);
     assi_str[12]=LO_UINT16(parent_short_adress);
     assi_str[13]=1;  
 #endif
     
     
      if(data_sucess_flag==2)
      {    
        SampleApp_SPI_SendCommand_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
        SampleApp_SPI_SendCommand_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
        SampleApp_SPI_SendCommand_DstAddr.addr.shortAddr = 0X0000;
      
        if ( AF_DataRequest( &SampleApp_SPI_SendCommand_DstAddr,
                       (endPointDesc_t *)&SampleApp_epDesc,
                        SAMPLEAPP_COMMAND_CLUSTERID, //终端传过去，协调器处理了
                        total_count, assi_str,
                        &SampleApp_TransID, 
                        AF_DISCV_ROUTE, 
                        AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
       {
      //  osal_mem_free(assi_str);
       }
      else
       {
       //  osal_mem_free(assi_str);
       }  
      }
     osal_mem_free(assi_str);
    }
  
 
  //test 读取真实的RSSI值返回
  else if(pkt->cmd.Data[8]=='f')//f
   {
     uint8 *real_rssi_str;
 //    data_sucess_flag=3;
     real_rssi_str=osal_mem_alloc(11);
     real_rssi_str[0]=str[0];
     real_rssi_str[1]=str[1];
     real_rssi_str[2]=str[2];
     real_rssi_str[3]=str[3];
     real_rssi_str[4]='/';
     real_rssi_str[5]='C';
     real_rssi_str[6]='/';
     real_rssi_str[7]='U';
     real_rssi_str[8]='U';
     real_rssi_str[9]='f';
     real_rssi_str[10]=ABS(pkt->rssi);//get the rssi
 //     if(data_sucess_flag==3)
    //  {    
        SampleApp_SPI_SendCommand_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
        SampleApp_SPI_SendCommand_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
        SampleApp_SPI_SendCommand_DstAddr.addr.shortAddr = 0X0000;
      
        if ( AF_DataRequest( &SampleApp_SPI_SendCommand_DstAddr,
                       (endPointDesc_t *)&SampleApp_epDesc,
                        SAMPLEAPP_COMMAND_CLUSTERID, //终端传过去，协调器处理了
                        11, real_rssi_str,
                        &SampleApp_TransID, 
                        AF_DISCV_ROUTE, 
                        AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
       {
           osal_mem_free(real_rssi_str);
       }
      else
       {
            osal_mem_free(real_rssi_str);
       }  
 //  }
  }
  
 
  //处理设备命令类型
  if(pkt->cmd.Data[8]==90)//Z:76 切换LED
  {
     str[9]='Z';
    Command_t=str[9];
    HalLedSet(HAL_LED_1, HAL_LED_MODE_TOGGLE);//  切换LED1的状态 
    str[10]=LED1_SBIT;//读取LED2 P1_1的状态LED2_SBIT，发回给协调器
 //   str[11]=0;
     //可以添加其他LED灯代码
    data_sucess_flag=1;
  }
  
  else if(pkt->cmd.Data[8]==83)//S:83 插座设备
  {
     str[9]='S';
    Command_t=str[9];
     data_sucess_flag=1;
     
    if ('1' == pkt->cmd.Data[13])  //打开插座
    { 
      OUT1_SBIT = 0; 
      str[10]=1;
      HAL_TURN_ON_LED1();//on led
    } 
    else 
    { 
      OUT1_SBIT = 1;
      str[10]=0;
      HAL_TURN_OFF_LED1();//off led
    }
  //    str[11]=0;
    //可以添加其他开关设备代码
  }
  

 
 else if(pkt->cmd.Data[8]==72)//H:72 查询温湿度
 {
    data_sucess_flag=1;
#if defined (TEMPHUME)    
    str[9]='H';
    Command_t=str[9];
    Report_Device_type = str[9];
    Sent_type = str[7];//20110528
    calc_sth10_resault();
    temp_val_X10 = (int)(fabs(temp_val_f) * 10);
    humi_val_X10 = (int)(humi_val_f * 10);
    if(humi_val_f < 0)
    {
      str[10]= '-';//负温度
    }
    else
    {
      str[10]= '+';//正温度
    }
    str[11]= (temp_val_X10 >> 8) & 0xFF; //将温度，发回给协调器
    str[12]= temp_val_X10 & 0xFF;      //将温度，发回给协调器
    str[13]= (humi_val_X10 >> 8) & 0xFF; //将湿度，发回给协调器
    str[14]= humi_val_X10 & 0xFF;      //将湿度，发回给协调器
#endif	
 }
 else if(pkt->cmd.Data[8]==67)//C:67 查询CO2浓度
 {
    data_sucess_flag=1;
#if defined (CO2)   
    //关闭 /c/xPC  命令设置的定时事件

    str[9]='C';
    Command_t = str[9];
    Report_Device_type = str[9];
    Sent_type = str[7];//20110528

    returnState = GetCO2PPM((uchar *)SHT100RxData);
    str[10]=SHT100RxData[1];//将CO2浓度，发回给协调器
    str[11]=SHT100RxData[2];//将CO2浓度，发回给协调器
#endif
 }
 else if(pkt->cmd.Data[8]==75)//K:75 空调命令
 {
    data_sucess_flag=1;
#if defined (AIRCONIDTION)    
    str[9]='K';
    Command_t=str[9];
    
    CmdType = pkt->cmd.Data[13];
    ACType = pkt->cmd.Data[14];
    OperativeMode = pkt->cmd.Data[16];

    //    ACCmdType = pkt->cmd.Data[13]; 
    HAL_TURN_ON_LED1();//on led

	ACStatue = pkt->cmd.Data[15];
	
	str[10] = CmdType;
	str[11] = ACType;
//	str[12] = ACStatue;
	str[13] = OperativeMode;
	
	str[14] = pkt->cmd.Data[17];
	str[15] = pkt->cmd.Data[18];
	str[16] = pkt->cmd.Data[19];
	str[17] = pkt->cmd.Data[20];
	

    if('1' == CmdType)		//'1' 表示学习
    {
	  if(NV_ACType == ACType)
	  {
		//空调类别没改变
		
	  }else
	  {
		//空调类别改变，用于不同类别的空调学习
		NV_ACType = ACType;
		m = osal_nv_write(ZCD_NV_IRR_AC_COMMON, NV_Offset_ACType, 1, &NV_ACType);
	  }
		
      ACTempL = (pkt->cmd.Data[17] - '0') * 10 +(pkt->cmd.Data[18] - '0'); 
      ACTempH = (pkt->cmd.Data[19] - '0') * 10 +(pkt->cmd.Data[20] - '0');
      m = StudyIRCode(ACType,OperativeMode,ACTempL,ACTempH);         
	  str[12] = ACStatue;
	}
    else if('2' == CmdType)	//'2' 表示使用
    {
	  if(NV_ACType == ACType)
	  {
		//空调类别匹配可以控制
//		ACStatue = pkt->cmd.Data[15];
		ACTemp   = (pkt->cmd.Data[17] - '0') * 10 +(pkt->cmd.Data[18] - '0'); 
		m = ControlAC(ACType,ACStatue,OperativeMode,ACTemp);
        if('0' == OperativeMode)
		  ACStatue = '0';
		else 
		  ACStatue = '1';
		str[12] = ACStatue;
	  }else
	  {
		//空调类别不匹配，不进行控制
		
	  }
    }

    HAL_TURN_OFF_LED1();//off led
  
#endif
 } 
 else if(pkt->cmd.Data[8]==76)//L:76 窗帘命令
 {
    data_sucess_flag=1;
#if defined (CURTAIN)    
    str[9]='L';
    Command_t=str[9];
    
    CmdType = pkt->cmd.Data[13];
    CTType = pkt->cmd.Data[14];
    OperativeMode = pkt->cmd.Data[16];

    //    ACCmdType = pkt->cmd.Data[13]; 
    HAL_TURN_ON_LED1();//on led

	CTStatue = pkt->cmd.Data[15];
	
	str[10] = CmdType;
	str[11] = CTType;
//	str[12] = ACStatue;
	str[13] = OperativeMode;
	
	str[14] = pkt->cmd.Data[17];
	str[15] = pkt->cmd.Data[18];
	str[16] = pkt->cmd.Data[19];
	str[17] = pkt->cmd.Data[20];
	

    if('1' == CmdType)		//'1' 表示学习
    {
	  if(NV_CTType == CTType)
	  {
		//空调类别没改变
		
	  }else
	  {
		//空调类别改变，用于不同类别的空调学习
		NV_CTType = CTType;
		m = osal_nv_write(ZCD_NV_IRR_AC_COMMON, NV_Offset_CTType, 1, &NV_CTType);
	  }
		
      ACTempL = (pkt->cmd.Data[17] - '0') * 10 +(pkt->cmd.Data[18] - '0'); 
      ACTempH = (pkt->cmd.Data[19] - '0') * 10 +(pkt->cmd.Data[20] - '0');
      m = StudyIRCode(CTType,OperativeMode,ACTempL,ACTempH);         
	  str[12] = CTStatue;
	}
    else if('2' == CmdType)	//'2' 表示使用
    {
	  if(NV_CTType == CTType)
	  {
		//空调类别匹配可以控制
//		ACStatue = pkt->cmd.Data[15];
		ACTemp   = (pkt->cmd.Data[17] - '0') * 10 +(pkt->cmd.Data[18] - '0'); 
		m = ControlAC(CTType,CTStatue,OperativeMode,ACTemp);
        if('0' == OperativeMode)
		  CTStatue = '0';
		else 
		  CTStatue = '1';
		str[12] = ACStatue;
	  }else
	  {
		//空调类别不匹配，不进行控制
		
	  }
    }

    HAL_TURN_OFF_LED1();//off led
#endif
 } 
 else if(pkt->cmd.Data[8]==82)//R:86 刷新设备列表 特殊处理，直接按一定格式上报
 {
    data_sucess_flag=1;
   str[9]='R';
   Command_t=str[9];
//   uint8 str[23];
   uint8 i;
#if defined(RTR_NWK)
  str[10]=Send_Router;
#else
  str[10]=Send_Endvice;
#endif  
  
  //#define ALARM
 //#define PIR
 //#define MAGNETISM  
 //#define SOCKET
 //#define SWITH
  
 #if defined(ALARM)
  str[11]=1;
 #elif defined(PIR)
  str[11]=2;
 #elif defined(MAGNETISM)
  str[11]=3; 
  #elif defined(SMOKEDETECT)
  str[11]=7; 
  #elif defined(CH4)
  str[11]=9; 
  
  #elif defined(SWITH)
  str[11]=11; 
  #elif defined(DIMMER)
  str[11]=12; 

  #elif defined(SOCKET)
  str[11]=16; 
  #elif defined(TEMPHUME)
  str[11]=17; 
  #elif defined(CO2)
  str[11]=18;  
 
 #elif defined(AIRCONIDTION)
  str[11]=21; 
 #elif defined(TELEVISION)
  str[11]=22;
 #elif defined(CURTAIN)
  str[11]=23; 
 #else
  str[11]=0; //大概可以分为开关，插座，告警，显示型等等，
              //可以根据智能家居的标准参考，可以通过定义不同的C文件的头文件来识别
 #endif 
  Sample_extendaddr=SampleApp_GetExtendAddr();
  for(i=0;i<16;i++)
  {
    str[i+12]=*Sample_extendaddr++;
  }
   osal_mem_free( extendaddr_mem );
 
  //预留的组字节，以后研究
#ifndef ZDO_COORDINATOR
  if(SampleApp_Group3.ID==0x0001)
    str[28]=1;
  else if(SampleApp_Group3.ID==0x0002)
    str[28]=2;
  else if(SampleApp_Group3.ID==0x0003)
    str[28]=3;
#else
  str[28]=0;
#endif  

#if defined( SMOKEDETECT )         //  
   if ((PUSH4_SBIT==1) && (PUSH5_SBIT==1)) //恢复正常了和红外相反，应为加了个三极管
    { 
      str[29]=0;
    } 
  else if((PUSH4_SBIT==0) && (PUSH5_SBIT==0))//
    { 
      str[29]=1;
    }   
#elif defined( PIR )       
  
   if ((PUSH4_SBIT==1) && (PUSH5_SBIT==1)) //
    { 
      str[29]=1; 
    } 
  else if((PUSH4_SBIT==0) && (PUSH5_SBIT==0))//
    { 
      str[29]=0;
    }   
#elif defined( CO2 ) 
   
      str[29]=0;

#elif defined( AIRCONIDTION ) 
   
      str[29]=0;
#elif defined( CURTAIN ) 
   
      str[29]=0;
#elif defined( TEMPHUME ) 
   
      str[29]=0;
#elif defined( SOCKET ) 
   
    if (OUT1_SBIT)  //切换开关状态
    { 
      str[29]=0;
    } 
    else 
    { 
      str[29]=1;
    }
#endif
   
 
   // SampleApp_Reflash_NwkMessage();//刷新上报长短地址 设备类型
 }
 //可以在此处增加其他的命令类型
  
  osal_mem_free( shortddr_mem );
  
//---------------------------------    
  if((pkt->cmd.Data[7]==85) &&  data_sucess_flag==1)//command: /C/XU start single data transport 单次返回
  {
     data_sucess_flag=0;
    str[8]='U';//单次返回
    SampleApp_SPI_SendCommand_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
    SampleApp_SPI_SendCommand_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
    SampleApp_SPI_SendCommand_DstAddr.addr.shortAddr = 0X0000;
    
    if ( AF_DataRequest( &SampleApp_SPI_SendCommand_DstAddr,
                         (endPointDesc_t *)&SampleApp_epDesc,
                          SAMPLEAPP_COMMAND_CLUSTERID, //终端传过去，协调器处理了
                          30, str,
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
//---------------------------------  
  
  else if(pkt->cmd.Data[7]==80) //commmand: /C/XP start periodic data transport
  {   
    osal_stop_timerEx( SampleApp_TaskID,
                        SAMPLEAPP_REPORTTIME_EXPEND_EVT);//重新周期回传时要结束已经存在的等待
    osal_stop_timerEx( SampleApp_TaskID,
                        SAMPLEAPP_COMMAND_PERIODIC_MSG_EVT);//重新周期回传时要结束已经存在的等待	
	ReportTimeTotal_s = 	(pkt->cmd.Data[13] - '0') * 100 +
	  				  		(pkt->cmd.Data[14] - '0') * 10 +
					  		(pkt->cmd.Data[15] - '0');
	Load_ReportTime_flag = 1;
    osal_start_timerEx( SampleApp_TaskID,
                        SAMPLEAPP_REPORTTIME_EXPEND_EVT,0);
      
  }
  else if((pkt->cmd.Data[7]==83)) //command:  /C/XS stop the timer 且不等于L
  {
    osal_stop_timerEx( SampleApp_TaskID,
					  SAMPLEAPP_REPORTTIME_EXPEND_EVT);//重新周期回传时要结束已经存在的等待
    osal_stop_timerEx( SampleApp_TaskID,
					  SAMPLEAPP_COMMAND_PERIODIC_MSG_EVT);//重新周期回传时要结束已经存在的等待	
  //  HalLedBlink( HAL_LED_4, 4, 50, (1000 / 4) ); //test
  }
  
  

  
}
/*********************************************************************
 * @fn      SampleApp_PrcoessCommandPeriodicMessage
 *
 * @brief   Prcoess the (periodic) message command from the coordinator.
 *
 * @param   none
 *
 * @return  none
 */
/*012 3 4 56789 10  11  12   13   14 ...*/
/*0x_ _ _ _/C/_  P   D   A    T    A ...*/
void SampleApp_PrcoessCommandPeriodicMessage(void)
{
  uint8 str[30];
//  uint8 buf[]="PERI_data";
//  uint8 i;
  uint8 *Sample_Cmd_shorAddr;
  
  Sample_Cmd_shorAddr=SampleApp_GetShortAddr();
  str[0]=*Sample_Cmd_shorAddr++;
  str[1]=*Sample_Cmd_shorAddr++;
  str[2]=*Sample_Cmd_shorAddr++;
  str[3]=*Sample_Cmd_shorAddr;
  str[4]='/';
  str[5]='C';
  str[6]='/';
  str[7]= Sent_type;  //发送类型 B  U  G//20110528
  str[8]='P';    //周期性返回
  str[9]=Report_Device_type; //命令类型  L S T V ....设备类型
 
  if(str[9]==76)//L:76 切换LED
  {
 //   HalLedSet(HAL_LED_1, HAL_LED_MODE_TOGGLE);  //只返回状态，不切换了 
    str[10]=LED1_SBIT;//读取LED2 P1_1的状态LED2_SBIT，发回给协调器
     //可以添加其他LED灯代码
  }
  
  else if(str[9]==83)//S:83 开关设备
  {
    if (OUT1_SBIT)  //切换开关状态
    { 
 //     OUT1_SBIT = 0; 
      str[10]=0;
    } 
    else 
    { 
 //     OUT1_SBIT = 1;
      str[10]=1;
    }
    //可以添加其他开关设备代码
  }
  
#if defined (TEMPHUME)    
 else if(str[9]==72)//H:72 查询温湿度
 {
    calc_sth10_resault();
    temp_val_X10 = (int)(fabs(temp_val_f) * 10);
    humi_val_X10 = (int)(humi_val_f * 10);
    if(humi_val_f < 0)
    {
      str[10]= '-';//负温度
    }
    else
    {
      str[10]= '+';//正温度
    }
    str[11]= (temp_val_X10 >> 8) & 0xFF; //将温度，发回给协调器
    str[12]= temp_val_X10 & 0xFF;      //将温度，发回给协调器
    str[13]= (humi_val_X10 >> 8) & 0xFF; //将湿度，发回给协调器
    str[14]= humi_val_X10 & 0xFF;      //将湿度，发回给协调器
 }
#endif 
#if defined (CO2)   
 else if(str[9]==67)//C:67 查询CO2浓度
 {
    //关闭 /c/xPC  命令设置的定时事件
    returnState = GetCO2PPM((uchar *)SHT100RxData);
    str[10]=SHT100RxData[1];//将CO2浓度，发回给协调器
    str[11]=SHT100RxData[2];//将CO2浓度，发回给协调器
 }
#endif
  osal_mem_free( shortddr_mem );
  SampleApp_SPI_SendCommand_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  SampleApp_SPI_SendCommand_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_SPI_SendCommand_DstAddr.addr.shortAddr = 0X0000;
    
  if ( AF_DataRequest( &SampleApp_SPI_SendCommand_DstAddr,
                       (endPointDesc_t *)&SampleApp_epDesc,
                        SAMPLEAPP_COMMAND_CLUSTERID, //终端传过去，协调器处理了
                        30, str,
                        &SampleApp_TransID, 
                        AF_DISCV_ROUTE, 
                        AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    Load_ReportTime_flag = 1;//准备重新装载等待时间的分值和秒值
	osal_start_timerEx( SampleApp_TaskID,
                        SAMPLEAPP_REPORTTIME_EXPEND_EVT,0 );
 //   HalLedBlink( HAL_LED_4, 4, 50, (1000 / 4) ); //test
  }
  else
  {
  }   
}



#if defined(RTR_NWK)

/*
处理地址冲突问题
0 1 2   3     4 5     6 7
A N R COUNT  DATA1  DATA2
*/
void ZDApp_PROESS_AnnounceAddress_CONFLICT(afIncomingMSGPacket_t *pkt)
{
   uint16 Child_short_adress;
   uint8  adress_count;
   uint16  *conflict_address; //其他设备发来的地址表
  // uint16  associt_adress;//本机设备关联表内的地址
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
                           0, 0 );//暂不处理capabilities 默认为0
           }
        }
       }  
    }
  }
      osal_mem_free( conflict_address );
}


/*
1 转发心跳包，最终发给COORDINATOR
2 处理关联表，根据心跳包设置ACTIVE OR INACTIVE的CHIRDREN
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
  {//转换短地址 从ACII码到UINT16
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
      SampleApp_Periodic_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
      SampleApp_Periodic_DstAddr.addr.shortAddr = 0X0000;  //give the coodinator
    //  SampleApp_Periodic_DstAddr.addr.shortAddr = parent_short_adress;//test
      
      if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc,
                         SAMPLEAPP_PERIODIC_COORDINATOR_CLUSTERID,
                         4,  //数据长度
                         HEART_BUFFER,//###添加的发送数据        
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
    
    //处理心跳包。看是否还在线
    
    for(uint8 j=0;j<NWK_MAX_DEVICES;j++)
    {
       nr =  AssociatedDevList[j].nodeRelation;
     if (nr == CHILD_RFD || nr == CHILD_RFD_RX_IDLE)// || nr == CHILD_FFD || nr == CHILD_FFD_RX_IDLE)    
     {
      if(AssociatedDevList[j].shortAddr==Child_short_adress)
      {
         Child_list[j].heart_count=0; 
       //  Child_list[j].isActive=true;//收到心跳了在线的
         break;
      }
     }
    }
}
#endif

//告警输入处理函数，如红外和门磁或者报警器和开关等  ZZZZZ

void SampleApp_SendInMessage(void)
{

  uint8 str[11];
  uint8 *Sample_Cmd_shorAddr;

  Sample_Cmd_shorAddr=SampleApp_GetShortAddr();
  str[0]=*Sample_Cmd_shorAddr++;
  str[1]=*Sample_Cmd_shorAddr++;
  str[2]=*Sample_Cmd_shorAddr++;
  str[3]=*Sample_Cmd_shorAddr;
  osal_mem_free( shortddr_mem );

  str[4]='I';
  str[5]='/';
  str[6]='/';
  str[7]='/';
  str[8]='/';
  str[9]='S';//根据设备类型而不同，S表示开关、H表示红外 目前手动改下

#if defined( SWITH )//如果定义了SWITH表示是开关，那么如果有输入的话应该主动切换开关状态，将状态发给协调器
                   //无线信号也可以切换，在SampleApp_ProcessCommandMessage中
  if (OUT1_SBIT) 
   { OUT1_SBIT = 0; str[10]=0;} 
  else 
   { OUT1_SBIT = 1;str[10]=1;}
 
 #elif defined( PIR )         //如果不是开关型设备，如红外告警输入，则直接将输入值发送给协调器
  
   if ((PUSH4_SBIT==1) && (PUSH5_SBIT==1)) //
    { 
      str[10]=1; 
                    //所有设备的报警协议统一  1为报警 0 为待警
     HalLedBlink ( HAL_LED_1, 0, 50, 300 ); 
    } 
  else if((PUSH4_SBIT==0) && (PUSH5_SBIT==0))//
    { 
      str[10]=0;
      HalLedBlink ( HAL_LED_1, 0, 99, 20000 );
    }
  
   #elif defined( SMOKEDETECT )         //
  
   if ((PUSH4_SBIT==1) && (PUSH5_SBIT==1)) //恢复正常了和红外相反，应为加了个三极管
    { 
      str[10]=0;
     HalLedBlink ( HAL_LED_1, 0, 99, 20000 );
  
    } 
  else if((PUSH4_SBIT==0) && (PUSH5_SBIT==0))//
    { 
      str[10]=1;
      HalLedBlink ( HAL_LED_1, 0, 50, 300 ); 
    }
  
  
  
  
#elif defined( MAGNETISM )
      if ((PUSH4_SBIT==1) && (PUSH5_SBIT==1)) //门开了，都为高电平，此时是P1.5上升沿中断
    { str[10]=1;} 
  else if((PUSH4_SBIT==0) && (PUSH5_SBIT==0))//门闭合，都为低电平，此时是P0.4下降沿中断
    { str[10]=0;}
    
#endif
  
  SampleApp_Addr_SendData_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  SampleApp_Addr_SendData_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Addr_SendData_DstAddr.addr.shortAddr = 0X0000;
    
    if ( AF_DataRequest( &SampleApp_SPI_SendCommand_DstAddr,
                         (endPointDesc_t *)&SampleApp_epDesc,
                          SAMPLEAPP_COMMAND_CLUSTERID, //终端传过去，协调器处理了
                          11, str,
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
//十六进制转字符函数
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
   {x=x-0x37;}		//针对16进制中的A B C D E F ASICII码A对应16进制的0X41 B对应0X42   所以 转换为16进制时应该减去0X37  (0X41-0X0A)
 else
   {x=x-0x30;}		 //ASICII码0~9  对应的十六进制为0X30~0X39    减去0X30  即得到转换
 if(y>0x39)
  {y=y-0x37;}
  else
   {y=y-0x30;}
 x=x<<4;			    //放到高位上，因为232通信的范围（除了前面的w r : n）就是在0~F几个数字之间 ，转换成16进制后，最多至为4位，半字节
 x=x&0xf0;
 z=y|x;
 return(z);
}



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


#if defined(RTR_NWK)
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
    req.removeChildren = FALSE;
    req.rejoin = FALSE;
    req.silent = TRUE;

    NLME_LeaveReq( &req );
    
  // AssocRemove(addrEntry.extAddr); //remove this device
  // AddrMgrEntryRelease( &addrEntry );
 //   NLME_RemoveChild( addrEntry.extAddr, true );
   //AssocWriteNV(); 
  //  ZDApp_NVUpdate();
   return true;
  }
  return false;
}

#endif

//看门狗初始化
void Init_Watchdog(void)
{
	WDCTL = 0x00;
	//时间间隔一秒，看门狗模式
	WDCTL |= 0x08;
	//启动看门狗
}


//喂狗函数
void FeetDog(void)
{
	WDCTL = 0xa0;
	WDCTL = 0x50;
}




/*
 * peripheral Initialization for the Generic Application,
 * 在此函数里给出了除系统必须的初始化以外的初始化，如二氧化碳外设，红外门磁外设等
 * 的初始化，根据不同的应用，作相应的初始化。此函数在ZMain.c中进入系统前调用。
 * 如果某些初始化工作希望放在任务内完成，则可以不放在这里。
 * 这样处理，不会破坏协议栈源码的封装。使应用代码的移植更为方便。
 * 
 *移植步骤，以在工程中添加门磁应用为例：
 * 1、编写门磁应用程序"xxx.c", 以及头文件"xxx.h"。此文件中包含门磁的初始化及应用函数。
 * 2、在device.c中包含"xxx.h"
 * 3、在device.c中的Apps_Init()函数中调用门磁的初始化函数
 * 4、在device.c中相应的任务中调用门磁其它的应用函数
 * 5、在ZMain.c中 包含"device.h"，为了调用Apps_Init()函数
 * 6、在ZMain.c中进入系统之前调用Apps_Init();
 *
 * 整个移植过程中需要改动的文件
 * 1、Zmain.c，在该文件中包含"device.h"，并调用Apps_Init();
 * 2、device.c，在该文件中包含中包含"xxx.h"，并编写相关的Apps_Init();
 * 3、device.h，在该文件中定义相关应用的宏
 * 
 * Apps_Init();中除了调用应用程序初始化函数外，还可以调用与相应应用程序相关的系统初始化 
 * 比如应用程序初始化需要初始化NV，则NV的初始化可以放在Apps_Init();中，而不放在应用程序初始化中
 */

void Apps_Init(void)
{

#if defined (AIRCONIDTION) || defined (CURTAIN) 
/*
  for(n=0;n<600;n++)
		test[n]=n;	
*/  

  //系统相关的初始化，如NV初始化，这部分代码在协议栈中已经有了
	//  n = zb_ReadConfiguration(0xa1, 10, &test);//ZCD_NV_SAPI_ENDPOINT
	n = osal_nv_item_init(ZCD_NV_IRR_AC_COMMON, 20, NULL); 
	n = osal_nv_item_init(ZCD_NV_IRR_AC_OFF, 1200, NULL); 
	n = osal_nv_item_init(ZCD_NV_IRR_AC_HOT_ON, 1200, NULL); 
	n = osal_nv_item_init(ZCD_NV_IRR_AC_COL_ON, 1200, NULL); 
	n = osal_nv_item_init(ZCD_NV_IRR_AC_WIND_ON, 1200, NULL); 
/*	
	//清除nv
	n = osal_nv_write(ZCD_NV_IRR_AC_COMMON, 0, 20, &EdgeTimeSlot);
	n = osal_nv_write(ZCD_NV_IRR_AC_OFF, 0, 1200, &EdgeTimeSlot);
	n = osal_nv_write(ZCD_NV_IRR_AC_HOT_ON, 0, 1200, &EdgeTimeSlot);
	n = osal_nv_write(ZCD_NV_IRR_AC_COL_ON, 0, 1200, &EdgeTimeSlot);
	n = osal_nv_write(ZCD_NV_IRR_AC_WIND_ON, 0, 1200, &EdgeTimeSlot);
*/	
	//读取保存在NV中的空调类型或者说是红外类型
	n = osal_nv_read(ZCD_NV_IRR_AC_COMMON, NV_Offset_ACType, 1, &NV_ACType);

/*	
	n = osal_nv_read(ZCD_NV_IRR_AC_COMMON, 0, 20, &test1);
	n = osal_nv_read(ZCD_NV_IRR_AC_OFF, 0, 1200, &test1);
	n = osal_nv_read(ZCD_NV_IRR_AC_HOT_ON, 0, 1200, &test1);
	n = osal_nv_read(ZCD_NV_IRR_AC_COL_ON, 0, 1200, &test1);
	n = osal_nv_read(ZCD_NV_IRR_AC_WIND_ON, 0, 1200, &test1);
	
	n = osal_nv_write(ZCD_NV_IRR_AC_COMMON, 0, 20, &test);
	n = osal_nv_write(ZCD_NV_IRR_AC_OFF, 0, 1200, &test);
	n = osal_nv_write(ZCD_NV_IRR_AC_HOT_ON, 0, 1200, &test);
	n = osal_nv_write(ZCD_NV_IRR_AC_COL_ON, 0, 1200, &test);
	n = osal_nv_write(ZCD_NV_IRR_AC_WIND_ON, 0, 1200, &test);
	
	n = osal_nv_read(ZCD_NV_IRR_AC_COMMON, 0, 20, &test1);
	n = osal_nv_read(ZCD_NV_IRR_AC_OFF, 0, 1200, &test1);
	n = osal_nv_read(ZCD_NV_IRR_AC_HOT_ON, 0, 1200, &test1);
	n = osal_nv_read(ZCD_NV_IRR_AC_COL_ON, 0, 1200, &test1);
	n = osal_nv_read(ZCD_NV_IRR_AC_WIND_ON, 0, 1200, &test1);
*/	
	//应用程序相关初始化
	irrInit();
#endif

#if defined (CO2) 
    returnState = GetCO2PPM((uchar *)SHT100RxData);
#endif
	
	
}



/*周期回传数据时 消耗等待时间 由于osal_start_timerEx()函数只能进行65535ms的延时，当超过该值时就需要调理*/
void SampleApp_ReportTimeExpend(void)
{
  if(1 == Load_ReportTime_flag)//装载等待时间
  {
	ReportTime_mi = ReportTimeTotal_s / 60;//等待时间转换成分秒格式，分值
	ReportTime_s  = ReportTimeTotal_s % 60;//等待时间转换成分秒格式，秒值
	Load_ReportTime_flag = 0;//禁止装载等待时间值
  }
  if(ReportTime_mi)
  {
	ReportTime_mi--;
	osal_start_timerEx( SampleApp_TaskID,
					   SAMPLEAPP_REPORTTIME_EXPEND_EVT,60000);//60s以后再次进入该函数对应的事件
  }
  else
  {
    if(ReportTime_s)
	{	
	  osal_start_timerEx( SampleApp_TaskID,
						 SAMPLEAPP_COMMAND_PERIODIC_MSG_EVT,
						 (ReportTime_s * 1000) );//分值已经消耗殆尽，在秒值时间消耗殆尽以后进入周期发送事件
	}
	else
	{	
	  osal_start_timerEx( SampleApp_TaskID,
						 SAMPLEAPP_COMMAND_PERIODIC_MSG_EVT,1);//分值已经消耗殆尽，在秒值时间消耗殆尽以后进入周期发送事件
	}
	
  }
}
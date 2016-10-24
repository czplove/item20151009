
/******************** (C) COPYRIGHT 2011 Jami***********************************
* File Name		: WLDB_device.c
* Author		: JamiLiang At Gmail.com
* Date			: 2014/01/14
* Description	: This file provides all the xxx Module functions.
* Version		: V1.0
* ChangeLog	:
* Version		Name       		Date			Description
  0.1			JamiLiang		2014/01/14		Initial Version
  1.0
*******************************************************************************/


/*********************************************************************
 * INCLUDES
 */


#include "OSAL.h"
#include "ZGlobals.h"
#include "AF.h"
#include "aps_groups.h"
#include "ZDApp.h"
#include "device.h"
#include "OnBoard.h"
#include "MT_UART.h"
#include "mac_rx.h"

#include "hal_led.h"
#include "hal_key.h"
#include "OSAL_Nv.h"

#include "AddrMgr.h"
#include "nwk_util.h"
#include "ZDObject.h"
//#include "APP_LIB.h"

#include "WldbFun.h"


/*********************************************************************
 * CONSTANTS
 */
#define DEV_CMD_BIT						10


uint8 Command_t,Sent_t;
devStates_t SampleApp_NwkState;
const cId_t SampleApp_ClusterList[SAMPLEAPP_MAX_CLUSTERS] =
{
	SAMPLEAPP_ONOFF_CLUSTERID,
	SAMPLEAPP_FLASH_CLUSTERID,
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
	SAMPLEAPP_PROFID,                //  uint16 AppProfId[2];
	SAMPLEAPP_DEVICEID,              //  uint16 AppDeviceId[2];
	SAMPLEAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
	SAMPLEAPP_FLAGS,                 //  int   AppFlags:4;
	SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
	(cId_t *)SampleApp_ClusterList,  //  uint8 *pAppInClusterList;
	SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
	(cId_t *)SampleApp_ClusterList   //  uint8 *pAppInClusterList;
};

#define WALL_DIMMER_BINDINGLIST       1
static cId_t bindingINClusters[WALL_DIMMER_BINDINGLIST] =
{
	SAMPLEAPP_ONOFF_CLUSTERID,
};

devStates_t SampleApp_NwkState;

afAddrType_t SampleApp_Periodic_DstAddr;
afAddrType_t SampleApp_Flash_DstAddr;
afAddrType_t SampleApp_SPI_SendData_DstAddr;
afAddrType_t SampleApp_SPI_SendCommand_DstAddr;
afAddrType_t SampleApp_Addr_SendData_DstAddr;
afAddrType_t SampleApp_annce_req_DstAddr;
afAddrType_t SampleApp_BIND_DstAddr;

#ifndef ZDO_COORDINATOR
aps_Group_t SampleApp_Group3;
#endif

uint8 SampleAppPeriodicCounter = 0;
uint8 SampleAppFlashCounter = 0;
uint16 dst_short_ddr=0;

extern Child_device_t Child_list[21];

uint8 logicalType1=0;

uint8 ON_OFF_ASSOCIATE_flag=0;
uint8 ANNCE_REQ_FLAG=0;
uint8 OUT1_SBIT_FLAG=0;

uint8 Heart_mesg_count=0;		//Add 2013_1231_1047
//extern uint8 Heart_mesg_flag;

void SampleApp_HandleKeys( uint8 shift, uint8 keys );
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void SampleApp_SendPeriodicMessage( void );

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
extern void clrIo(void);
extern void clrDb(void);
extern uint8 getCurMelody(uint8 btn);
extern void swIo(uint8 tBtn,uint8 tBtm);
extern uint8 gFlagOp;
uint8 Bind_Glag=0;
extern uint8 ButtonEn;
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
	SampleApp_BIND_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
	SampleApp_BIND_DstAddr.endPoint = 0;
	SampleApp_BIND_DstAddr.addr.shortAddr = 0;

	// Fill out the endpoint description.
	SampleApp_epDesc.endPoint = SAMPLEAPP_ENDPOINT;
	SampleApp_epDesc.task_id = &SampleApp_TaskID;
	SampleApp_epDesc.simpleDesc
	    = (SimpleDescriptionFormat_t *)&SampleApp_SimpleDesc;
	SampleApp_epDesc.latencyReq = noLatencyReqs;


	// Register the endpoint description with the AF
	afRegister( &SampleApp_epDesc );

	// Register for all key events - This app will handle all key events
	RegisterForKeys( SampleApp_TaskID );

	ZDO_RegisterForZDOMsg( SampleApp_TaskID, End_Device_Bind_rsp );

#if defined(RTR_NWK)
	uint8 nr;
	for(uint8 i=0; i<NWK_MAX_DEVICES; i++)
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
           Init_Watchdog();//启动看门狗 1S启动一次 
#endif
	APP_JOIN_INIT();
	initWldb();
	HalLedBlink ( HAL_LED_1, 3, 10, 100 );
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
					SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
					if ( (SampleApp_NwkState == DEV_ZB_COORD)
					        || (SampleApp_NwkState == DEV_ROUTER)
					        || (SampleApp_NwkState == DEV_END_DEVICE) )
					{
						HalLedBlink ( HAL_LED_1, 1, 99, 2500 );
						APP_JOIN_DEAL_WITH();
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
			osal_msg_deallocate( (uint8 *)MSGpkt );
			// Next - if one is available
			MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
		}//end while ( MSGpkt )

		return (events ^ SYS_EVENT_MSG);
	}
	if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT )//发送周期消息事件
	{
		if(++Heart_mesg_count>=Heart_mesg_flag)
		{
			Heart_mesg_count = 0;
			SampleApp_SendPeriodicMessage();
		}
		osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
							50000+ 50*(osal_rand() & 0x00FF));
		return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT);	// return unprocessed events
		//*/
	}
	if ( events & SAMPLEAPP_SEND_NWKADDR_EVT )
	{
		SampleApp_SendNwkaddrMessage(0,DEVICE_TYPE_ID);
		return ( events ^ SAMPLEAPP_SEND_NWKADDR_EVT);
	}
	if ( events & NEW_JOIN_ROLE_EVT )
	{
		APP_JOIN_START_EVT();
		return ( events ^ NEW_JOIN_ROLE_EVT);
	}
	if ( events & SAMPLEAPP_RESTORE_EVT )
	{
		SystemResetSoft();
	}
	if ( events & ASSOCIA_MENTAIN_EVT )
	{
		APP_DEAL_WITH_ASSOCIATED();
		return (events ^ ASSOCIA_MENTAIN_EVT );
	}
	if ( events & WLDB_CLRIO_EVENT )
	{
		clrIo();
		return (events ^ WLDB_CLRIO_EVENT );
	}
	if ( events & WLDB_CLRDB_EVENT )
	{
		clrDb();
		return (events ^ WLDB_CLRDB_EVENT );
	}
	return 0;
}

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
extern uint8 key_ISR_FLAG;
void SampleApp_HandleKeys( uint8 shift, uint8 keys )
{
	zAddrType_t dstAddr;
	if ( keys &  HAL_KEY_SW_5 )//8
	{
		if((key_double_flag == 3) && (devState == DEV_ROUTER))
		{
			HalLedBlink ( HAL_LED_1, 1, 50, 800 );
			dstAddr.addrMode = afAddr16Bit;
			dstAddr.addr.shortAddr = 0;   // Coordinator makes the match
			ZDP_EndDeviceBindReq( &dstAddr,
								NLME_GetShortAddr(),
								SAMPLEAPP_ENDPOINT,
								0x0F08,	1,
								bindingINClusters,   // No incoming clusters to bind   0, NULL
								0, NULL, FALSE );
		}

		if(key_double_flag==4)
		{
			APPLY_TO_JOIN_OF_KEY();
		}

		if(key_hold_flag==1)
		{
			HalLedBlink ( HAL_LED_1, 4, 50, 1000 );
			RESTORE_TO_FACTORY();
			osal_start_timerEx( SampleApp_TaskID,SAMPLEAPP_RESTORE_EVT,5000 );
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
				HalLedBlink ( HAL_LED_1, 3, 50, 1000 );
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
#define PKT_CMD_BIT		0
void SampleApp_ProcessBINDINGMessage(afIncomingMSGPacket_t *pkt)
{
  if(pkt->endPoint == SAMPLEAPP_ENDPOINT)
  {
    if( (pkt->cmd.Data[0]==1) || (pkt->cmd.Data[0]==2))
    {
     if(ButtonEn)
     {
      swIo(1,1);//播放
     }
     Bind_Glag=1;
   //  SampleApp_SendInMessage();
    }
  }
}

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
		case SAMPLEAPP_ONOFF_CLUSTERID:

			SampleApp_ProcessBINDINGMessage(pkt);
			break;
			
			
			 case SAMPLEAPP_ADDR_CLUSTERID: //设备入网的信息处理

      SampleApp_ProcessNwkaddrMessage(pkt);

      break;

     case SAMPLEAPP_PERIODIC_CLUSTERID:

      SampleApp_ROUTER_ProcessPeriodicMessage_Data(pkt);

      break;
			
		default:
			break;
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
	uint8 str[50];

	uint8 *Sample_Cmd_shorAddr;
	uint8 *Sample_extendaddr;
	uint8 data_sucess_flag=0;
	uint8 active_count=0;
	uint8 SET_str[10];

	Sample_Cmd_shorAddr=SampleApp_GetShortAddr();
	str[0]=*Sample_Cmd_shorAddr++;
	str[1]=*Sample_Cmd_shorAddr++;
	str[2]=*Sample_Cmd_shorAddr++;
	str[3]=*Sample_Cmd_shorAddr;
	str[4]='/';
	str[5]='C';
	str[6]='/';

	str[7]=pkt->cmd.Data[6];
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

	 if(pkt->cmd.Data[8]=='L')
	{
		uint8 blink_count=0;
		blink_count = (pkt->cmd.Data[13]-0x30)*100 + (pkt->cmd.Data[14]-0x30)*10 + (pkt->cmd.Data[15]-0x30);
		HalLedBlink ( HAL_LED_3, blink_count, 50, 800 );
	}

	else if(pkt->cmd.Data[8]==82)//R:86
	{
		str[9]='R';
		Command_t=str[9];
		uint8 i;
#if defined(RTR_NWK)
		str[10]=Send_Router;
#else
		str[10]=Send_Endvice;
#endif
		str[11]=DEVICE_TYPE_ID;
		//str[28]=0;

		Sample_extendaddr=SampleApp_GetExtendAddr();
		for(i=0; i<16; i++)
		{
			str[i+12]=*Sample_extendaddr++;
		}
		osal_mem_free( extendaddr_mem );
		// init report
                str[29]=9;
		str[30]=getCurMelody(1);
		str[31]=1;//getCurMelody(2);
		str[32]=1;//getCurMelody(3);
		data_sucess_flag=1;
		active_count =33;
	}

	else if(pkt->cmd.Data[8]=='E')
	{
		str[9]='/';
		str[DEV_CMD_BIT]=pkt->cmd.Data[13]-'0';
		str[DEV_CMD_BIT+1]=pkt->cmd.Data[14]-'0';
		str[DEV_CMD_BIT+2]=pkt->cmd.Data[15]-'0';
		str[DEV_CMD_BIT+3]=pkt->cmd.Data[16]-'0';
		str[DEV_CMD_BIT+4]=pkt->cmd.Data[17]-'0';
		active_count = DEV_CMD_BIT+NWK_interface(&str[DEV_CMD_BIT],5);
		data_sucess_flag = 1;
	}
	osal_mem_free( shortddr_mem );

	if((pkt->cmd.Data[7]==85) && data_sucess_flag==1)//command: /C/XU start single data transport
	{
		str[8]='U';
		SampleApp_SPI_SendCommand_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
		SampleApp_SPI_SendCommand_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
		SampleApp_SPI_SendCommand_DstAddr.addr.shortAddr = pkt->srcAddr.addr.shortAddr;

		if ( AF_DataRequest( &SampleApp_SPI_SendCommand_DstAddr,
		                     (endPointDesc_t *)&SampleApp_epDesc,
		                     SAMPLEAPP_COMMAND_CLUSTERID,
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
}

#define MSG_LEN			17
void SampleApp_SendInMessage(void)
{
	//uint8 str[11];
	uint8 str[MSG_LEN];
	uint8 *Sample_Cmd_shorAddr;
	uint8 tArrIndex = DEV_CMD_BIT;

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
	str[9]='/';

        if(Bind_Glag)
        {
          str[tArrIndex] = 8;
        }        
        else
        {
	  str[tArrIndex] = 9;
	}
        str[++tArrIndex] = getCurMelody(1);
	str[++tArrIndex] = 1;//getCurMelody(2);
	str[++tArrIndex] = 1;//getCurMelody(3);

	SampleApp_SPI_SendCommand_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
	SampleApp_SPI_SendCommand_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
	SampleApp_SPI_SendCommand_DstAddr.addr.shortAddr = 0X0000;

	if ( AF_DataRequest( &SampleApp_SPI_SendCommand_DstAddr,
	                     (endPointDesc_t *)&SampleApp_epDesc,
	                     SAMPLEAPP_COMMAND_CLUSTERID,
	                     ++tArrIndex, str,		//index = 17
	                     &SampleApp_TransID,
	                     AF_DISCV_ROUTE,
	                     AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
	{
	}
	else
	{
	}
}


void Init_Watchdog(void)
{
	WDCTL = 0x00;
	WDCTL |= 0x08;
}

void FeetDog(void)
{
	WDCTL = 0xa0;
	WDCTL = 0x50;
}







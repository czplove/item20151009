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



//#define COMMUNICATION
#include "OSAL.h"
#include "ZGlobals.h"
#include "AF.h"
#include "aps_groups.h"
#include "ZDApp.h"
//#include "device.h"
#include "OnBoard.h"
#include "MT_UART.h"//2430 是 SPIMGR  2530叫 MT_UART  换了个名字
#include "mac_rx.h"

/* HAL */
#include "hal_led.h"
#include "hal_key.h"
#include "OSAL_Nv.h"
//#include "my_time.h"

#include "AddrMgr.h"
#include "nwk_util.h"
#include "ZDObject.h"
#include "ZDO_LIB.h"
#include "common_device.h"
#include "hal_drivers.h"
#include "hal_key.h"
#include "OSAL_PwrMgr.h"

#include "IRR.h"
#include "m25p16.h"
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
 * GLOBAL VARIABLES—全局变量
 */

const cId_t SampleApp_ClusterList[SAMPLEAPP_MAX_CLUSTERS] =
{
  SAMPLEAPP_ONOFF_CLUSTERID, //周期信息簇ID=1
  SAMPLEAPP_FLASH_CLUSTERID,    //闪烁信息簇ID=2
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




afAddrType_t SampleApp_Periodic_DstAddr;       //周期事件寻址模式
afAddrType_t SampleApp_Flash_DstAddr;          //闪烁事件寻址模式
afAddrType_t SampleApp_SPI_SendData_DstAddr;   //串口数据信息
afAddrType_t SampleApp_SPI_SendCommand_DstAddr;//串口命令信息
afAddrType_t SampleApp_Addr_SendData_DstAddr;  //入网报告
afAddrType_t SampleApp_annce_req_DstAddr;    //annce 关联表对比清楚信息
afAddrType_t SampleApp_BIND_DstAddr;




uint16 pollrate_time =0 ;
extern uint8 POLLRATE_TIME_VALUE;

uint8 ERROR_COUNT=0;
  
uint16 myApp_ReadBattery( void );

extern void CR2_DATA_NODETECT_MSG(void);
/*********************************************************************
 * LOCAL FUNCTIONS
 */
//void SampleApp_HandleKeys( uint8 shift, uint8 keys );//本用户应用按键函数
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );//本用户应用信息回调函数
void SampleApp_SendPeriodicMessage( void );         //本用户应用发送周期消息函数
void SampleApp_ProcessCommandMessage(afIncomingMSGPacket_t *pkt);
void SampleApp_SendInMessage(void);
void SampleApp_ProcessBINDINGMessage(afIncomingMSGPacket_t *pkt);
//void  Delay_1u(uint32 microSecs);




extern    	uint8 CmdType;  //命令类型： '1' 学习，'2' 使用
	extern uint16 IRCodeIndex;   //
//extern uint8 longPollRateFlag;
extern void changePollRate(void);
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

  
  SampleApp_epDesc.endPoint = SAMPLEAPP_ENDPOINT;//SampleApp EP描述符的EP号：20
  SampleApp_epDesc.task_id = &SampleApp_TaskID;//SampleApp EP描述符的任务ID：0
  SampleApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&SampleApp_SimpleDesc;//SampleApp EP简单描述符
  SampleApp_epDesc.latencyReq = noLatencyReqs;//延时策略

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
 
  
    afDataConfirm_t *afDataConfirm;
   uint8 sentStatus;
  
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

          
          case AF_DATA_CONFIRM_CMD:
          afDataConfirm = (afDataConfirm_t *)MSGpkt;
          sentStatus = afDataConfirm->hdr.status;
          if ( sentStatus != ZSuccess )
          {
              ZDO_ACK_DEAL_WITH(); 
          }
          break; 
          
        case ZDO_STATE_CHANGE:  
          SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (SampleApp_NwkState == DEV_ZB_COORD)
              || (SampleApp_NwkState == DEV_ROUTER)
              || (SampleApp_NwkState == DEV_END_DEVICE) )
          {
           APP_JOIN_DEAL_WITH();
           USER_APP_JOIN_MSG(); // 处理入网后应用的内容 
          }
          else
          {
   
          }
          break;

        default:
          break;
      }

      // Release the memory

      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next - if one is available
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    }//end： while ( MSGpkt )

    return (events ^ SYS_EVENT_MSG);
  }
  
  if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT )//发送周期消息事件
  {
      USER_MAYBE_BUTTERY_MSG();
      SampleApp_SendPeriodicMessage();

      osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
                              (55000+ 30*(osal_rand() & 0x00FF)) );
   
    return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT);
  }
  
   
  
  

  if ( events & SAMPLEAPP_SEND_NWKADDR_EVT )
  {
     SampleApp_SendNwkaddrMessage(JOIN_ROLE,DEVICE_ID);

    return ( events ^ SAMPLEAPP_SEND_NWKADDR_EVT);
  }
  
   
  

    if ( events & SAMPLEAPP_SEND_RS_EVT )
  {

    USER_SET_RS_MSG();//设置RS数据
    sampleAPP_sendRS_periodMessage(JOIN_ROLE,DEVICE_ID,RS_LEN,RS_STR);
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

       #if defined (CR2_DATA_NODETECT)
     // if(DEEP_POLL_FLAG==0)//非深度POLL的时候取消报警 2014 11 29 所有时刻接受命令都禁止中断一下
      { 
       CR2_DATA_NODETECT_MSG();
      }     
     #endif
      
      SampleApp_ProcessCommandMessage(pkt);
      break;  
      
      
      case SAMPLEAPP_FLASH_CLUSTERID:

       OS_FLAG=APP_ENDDEVICE_ACK(pkt->cmd.Data[0],pkt->cmd.Data[1],pkt->cmd.Data[2]);
      
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


  uint8 active_count=0;
  uint8 SET_str[10];

  for(uint8 i=0;i<4;i++)
  {
   str[i] = SHORT_ADRESS[i];
  }
  str[4]='/';
  str[5]='C';
  str[6]='/';
  

   str[7]=pkt->cmd.Data[6];//处理设备发送类型
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
   
     //处理设备命令类  窗帘控制器
   if(pkt->cmd.Data[8]==DEFINED_CONTROL_CHAR)
   {
  
    active_count = USER_APP_COMMAND(&pkt->cmd.Data[13],&str[9]);
    str[8]='U';//单次返回
    SampleApp_SPI_SendCommand_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
    SampleApp_SPI_SendCommand_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
    SampleApp_SPI_SendCommand_DstAddr.addr.shortAddr = pkt->srcAddr.addr.shortAddr;//发送给来的设备

    if ( AF_DataRequest( &SampleApp_SPI_SendCommand_DstAddr,
                         (endPointDesc_t *)&SampleApp_epDesc,
                          SAMPLEAPP_COMMAND_CLUSTERID, //终端传过去，协调器处理了
                          active_count, str,
                          &SampleApp_TransID, 
                          AF_DISCV_ROUTE, 
                          AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
    {
      
     }
    else
     {
       
     }
     
       /* 
     
    str[9]='/';

   CmdType = pkt->cmd.Data[13];//窗帘设备命令类型 学习 使用
   IRCodeIndex = (pkt->cmd.Data[14]- '0')*100 +
                 (pkt->cmd.Data[15]- '0')*10 +
                 (pkt->cmd.Data[16]- '0');      //编码序号
   if('1' == CmdType)		//'1' 表示学习
   {
     irCodeRx(IRCodeIndex);
  //   changePollRate();
   }
   else if('2' == CmdType)	//'2' 表示使用
   {   
     irCodeTx(IRCodeIndex);
   //  changePollRate();
   }
  
   str[10] = CmdType - '0';//数据回填
   str[12] = IRCodeIndex;//数据回填
   str[11] = IRCodeIndex >> 8;


    if ( AF_DataRequest( &SampleApp_SPI_SendCommand_DstAddr,
                         (endPointDesc_t *)&SampleApp_epDesc,
                          SAMPLEAPP_COMMAND_CLUSTERID, //终端传过去，协调器处理了
                          13, str,
                          &SampleApp_TransID, 
                          AF_DISCV_ROUTE, 
                          AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
    {
      
     }
    else
     {
       
     }
     */
     
   }
 
 
    else if(pkt->cmd.Data[8]=='L')//v:118
 {
  uint8 blink_count=0;
  blink_count = (pkt->cmd.Data[13]-0x30)*100 + (pkt->cmd.Data[14]-0x30)*10 + (pkt->cmd.Data[15]-0x30);
  HalLedBlink ( HAL_LED_1, blink_count, 50, 800 );

 }
  else if(pkt->cmd.Data[8]=='O')
  {
    
       uint8 *SET_str;
       uint8 sent_flag = 0;
       
       HalLedBlink ( HAL_LED_1, 1, 50, 1000 );
       if((pkt->cmd.Data[13]-'0') ==  1)
       {
         pollrate_time = 100*(pkt->cmd.Data[14]-'0')+10*(pkt->cmd.Data[15]-'0')+(pkt->cmd.Data[16]-'0');
         if(pollrate_time > 120 )
         {
           pollrate_time = 120;
         }
         else if(pollrate_time == 0)
         {
           pollrate_time = 1;
         }
         
         if(POLLRATE_TIME_VALUE!=pollrate_time)
         { 
           POLLRATE_TIME_VALUE = (uint8)pollrate_time;
           osal_nv_write( ZCD_NV_POLLRATE_TIME, 0, 1, &POLLRATE_TIME_VALUE );  
           //pollrate_time = pollrate_time * 500;
           NLME_SetPollRate( pollrate_time * 500 ); 
         }
         //else
         //pollrate_time = pollrate_time * 500;
         sent_flag = 1;
       }
       else if((pkt->cmd.Data[13]-'0') ==  2)
       {
         //osal_nv_item_init( ZCD_NV_POLLRATE_TIME, 1, &POLLRATE_TIME_VALUE );  
         //osal_nv_read( ZCD_NV_POLLRATE_TIME, 0, 1, &POLLRATE_TIME_VALUE );
         //if((POLLRATE_TIME_VALUE!=0)&&(POLLRATE_TIME_VALUE!=255))
         //pollrate_time = POLLRATE_TIME_VALUE * 500; 
         //else
         //pollrate_time = 2000; // default 2S
         sent_flag = 1;
       }
       
       if(sent_flag)
       {
         
         pollrate_time = POLLRATE_TIME_VALUE * 500;
         
         SET_str=osal_mem_alloc(13);
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
         SET_str[10]=0x22;
         SET_str[11]=pollrate_time/256;//get the rssi
         SET_str[12]=pollrate_time%256;//get the rssi
         
         SampleApp_SPI_SendCommand_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
         SampleApp_SPI_SendCommand_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
         // SampleApp_SPI_SendCommand_DstAddr.addr.shortAddr = 0X0000;
         SampleApp_SPI_SendCommand_DstAddr.addr.shortAddr = pkt->srcAddr.addr.shortAddr;//发送给来的设备
         
         if ( AF_DataRequest( &SampleApp_SPI_SendCommand_DstAddr,
                             (endPointDesc_t *)&SampleApp_epDesc,
                             SAMPLEAPP_COMMAND_CLUSTERID, //终端传过去，协调器处理了
                             13, SET_str,
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
 else if(pkt->cmd.Data[8]==82)//R:86 刷新设备列表 特殊处理，直接按一定格式上报
 {

   osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_RS_EVT,100);
   
 }
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


void SampleApp_PermitJoin(uint8 permit_time);
void SampleApp_PermitJoin(uint8 permit_time)
{
    zAddrType_t dstAddr;
 if( (devState==DEV_END_DEVICE) || (devState==DEV_ROUTER) )
      {
        if(permit_time>0)
        HalLedBlink ( HAL_LED_1, 1, 50, 600 );//应许
        else
         HalLedBlink ( HAL_LED_1, 2, 50, 600 );//应许  
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
                          SAMPLEAPP_COMMAND_CLUSTERID, //终端传过去，协调器处理了
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
	key_hold_flag = 1;//长按有效。  key_double_flag 表示按键按下的次数
	if (keys && (pHalKeyProcessFunction))
	{
	  (pHalKeyProcessFunction) (keys, HAL_KEY_STATE_NORMAL);
	  // key_ISR_FLAG=0;
	  KEY_NET_ISR_FLAG=0;
	}
  }
  key_holdtime_waiting_flag = 0;	//清除等待长按时间标志
  KEY_NET_ISR_FLAG=0;
}

void HalKeyMultiClickCheck(void);
void HalKeyMultiClickCheck(void)
{
  uint8 keys = 0;
  
  keys |= HAL_KEY_SW_8;
  key_double_flag=KEY_NET_ISR_FLAG;//将几次按键值赋给状态
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
	 	osal_start_timerEx (Hal_TaskID, HAL_KEY_HOLD_EVENT, HAL_KEY_HOLD_VALUE);//长按时间（10s）达到以后进入相应的处理事件
		key_holdtime_waiting_flag = 1;
	}
  }
  else
  {
	
	key_hold_flag = 0;//长按无效。  key_double_flag 表示按键按下的次数
	if (keys && (pHalKeyProcessFunction))
	{
	  (pHalKeyProcessFunction) (keys, HAL_KEY_STATE_NORMAL);
	  // key_ISR_FLAG=0;
	  KEY_NET_ISR_FLAG=0;
	}
  }
  
}




// ADC definitions for CC2430 from the hal_adc.c file
#define HAL_ADC_REF_125V    0x00    /* Internal 1.25V Reference */
#define HAL_ADC_DEC_064     0x00    /* Decimate by 64 : 8-bit resolution */
#define HAL_ADC_DEC_128     0x10    /* Decimate by 128 : 10-bit resolution */
#define HAL_ADC_DEC_512     0x30    /* Decimate by 512 : 14-bit resolution */
#define HAL_ADC_CHN_VDD3    0x0f    /* Input channel: VDD/3 */
#define HAL_ADC_CHN_TEMP    0x0e    /* Temperature sensor */

uint16 myApp_ReadBattery( void )
{

#if defined (HAL_MCU_CC2430) || defined (HAL_MCU_CC2530)

  uint32 value;
  //uint32 value;

  /* Clear ADC interrupt flag */
  ADCIF = 0;

  ADCCON3 = (HAL_ADC_REF_125V | HAL_ADC_DEC_128 | HAL_ADC_CHN_VDD3);
// ADCCON3 = (HAL_ADC_REF_125V | HAL_ADC_DEC_512 | HAL_ADC_CHN_VDD3);
  
  /* Wait for the conversion to finish */
  while ( !ADCIF );

  /* Get the result */
  value = ADCL;
  value |= ((uint16) ADCH) << 8;

  /*
   * value now contains measurement of Vdd/3
   * 0 indicates 0V and 32767 indicates 1.25V
   * voltage = (value*3*1.25)/32767 volts
   * we will multiply by this by 10 to allow units of 0.1 volts
   */

  
  value = value >> 6;   // divide first by 2^6
  
 // value = value >> 2;  
  
 // xx *1.15*3>10 
  //value = (uint16)(value * 34.5);//1.15V*30
  
  value = (uint32)(value * 345);//1.15V*300
  

  //value = value >> 13;   
  value = value >> 9;   
  // /512
  // ...and later by 2^9...to prevent overflow during multiplication

  return (uint16)value;

#endif    // CC2430 or CC2530

#if defined HAL_MCU_MSP430

  uint16 value;

/*
  There are more than MSP430 board now. Idealy, ADC read should be called
*/
#if defined (HAL_BOARD_F5438)

  value = HalAdcRead (HAL_ADC_CHANNEL_VDD, HAL_ADC_RESOLUTION_14);
  value = value * 50;
  value = value / 4096;

#else

  ADC12CTL0 = ADC12ON+SHT0_2+REFON;             // Turn on and set up ADC12
  ADC12CTL1 = SHP;                              // Use sampling timer
  ADC12MCTL0 = SREF_1+INCH_11;                  // Vr+=Vref+

  ADC12CTL0 |= ENC | ADC12SC;                   // Start conversion
  while ((ADC12IFG & BIT0)==0);

  value = ADC12MEM0;

  /*
   * value now contains measurement of AVcc/2
   * value is in range 0 to 4095 indicating voltage from 0 to 1.5V
   * voltage = (value*2*1.5)/4095 volts
   * we will multiply by this by 10 to allow units of 0.1 volts
   */

  value = value >> 1;     // value is now in range of 0 to 2048
  value = value * 30;
  value = value >> 11;

#endif

  return ( value );

#endif // MSP430



}

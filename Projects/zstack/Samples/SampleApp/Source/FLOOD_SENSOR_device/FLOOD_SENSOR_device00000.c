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
//#include "PIR_device.h"

#include "device.h"

#include "OSAL.h"
#include "ZGlobals.h"
#include "AF.h"
#include "aps_groups.h"
#include "ZDApp.h"

#include "OnBoard.h"
#include "MT_UART.h"//2430 是 SPIMGR  2530叫 MT_UART  换了个名字
#include "mac_rx.h"

/* HAL */
#include "hal_led.h"
#include "hal_key.h"
#include "OSAL_Nv.h"
#include "nwk_util.h"
#include "ZDProfile.h"
#include "ZDObject.h"

#include "AddrMgr.h"
#include "nwk_util.h"
//#include "irr.h"
//#include "m25p16.h"
//#include "APP_LIB.h"
#include "ZDO_LIB.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
  //  uint8  rxlen;//接收数据长度
   // uint8 *databuf;//接收数据块指针
  //  uint8 *shortddr_mem;  
   // uint8 *extendaddr_mem;
    uint8 Command_t,Sent_t;
 //   uint16 Communication_saddr; 
    
    	uint8 CmdType;  //命令类型： '1' 学习，'2' 使用
	uint16 IRCodeIndex;   //
	uint8 SAddr,PAddr,PNum;
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES—全局变量
 */

const cId_t SampleApp_ClusterList[SAMPLEAPP_MAX_CLUSTERS] =
{
  SAMPLEAPP_FLASH_CLUSTERID,    //闪烁信息簇ID=2
  SAMPLEAPP_ADDR_CLUSTERID,
  SAMPLEAPP_COMMAND_CLUSTERID,
  SAMPLEAPP_PERIODIC_CLUSTERID,
  SAMPLEAPP_ANNCE_REQ_CLUSTERID,
  SAMPLEAPP_PERIODIC_COORDINATOR_CLUSTERID,
  SAMPLEAPP_ACTIVE_COMMAND_CLUSTERID,
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

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
//uint8 SampleApp_TaskID;   
devStates_t SampleApp_NwkState;

//uint8 SampleApp_TransID;  



afAddrType_t SampleApp_Periodic_DstAddr;       //周期事件寻址模式
afAddrType_t SampleApp_SPI_SendData_DstAddr;   //串口数据信息
afAddrType_t SampleApp_SPI_SendCommand_DstAddr;//串口命令信息
afAddrType_t SampleApp_Addr_SendData_DstAddr;  //入网报告
afAddrType_t SampleApp_annce_req_DstAddr;    //annce 关联表对比清楚信息
afAddrType_t SampleApp_BIND_DstAddr;

uint8 SampleAppPeriodicCounter = 0;
uint8 SampleAppFlashCounter = 0;
uint16 dst_short_ddr=0;

uint8 logicalType1=0;
uint8 sentStatus;
uint8 ANNCE_REQ_FLAG=0;

extern uint8 OS_FLAG;
uint8 BUTTERY_LOW_FLAG=0;//电池欠压告警的标记位

uint8 water_state = 0;

  uint8 ERROR_COUNT=0;
  
  uint8 no_poll_time_count = 0; //无poll时间计数

  uint8 Heart_mesg_count_ACK=0;
  
  extern uint8 HEART_NEED_ACK_FLAG;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void SampleApp_HandleKeys( uint8 shift, uint8 keys );//本用户应用按键函数
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );//本用户应用信息回调函数
void SampleAPP_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
void SampleApp_ProcessBINDINGMessage(afIncomingMSGPacket_t *pkt);
void SampleApp_ProcessCommandMessage(afIncomingMSGPacket_t *pkt);
void SampleApp_SendInMessage(void);


void Delay_device_1u(uint32 microSecs);
void Init_Watchdog(void);
void FeetDog(void);

uint8 zb_ReadConfiguration( uint8 configId, uint8 len, void *pValue );
uint8 zb_WriteConfiguration( uint8 configId, uint8 len, void *pValue );
uint16 myApp_ReadBattery( void );//READ THE buttery


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

  SampleApp_BIND_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  SampleApp_BIND_DstAddr.endPoint = 0;
  SampleApp_BIND_DstAddr.addr.shortAddr = 0;
  
  afRegister( &SampleApp_epDesc );

  RegisterForKeys( SampleApp_TaskID ); 
  
  ZDO_RegisterForZDOMsg( SampleApp_TaskID, End_Device_Bind_rsp );
  
  
 
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
          
       case ZDO_CB_MSG:
          SampleAPP_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
          break; 
          
         case AF_DATA_CONFIRM_CMD:
          afDataConfirm = (afDataConfirm_t *)MSGpkt;
          sentStatus = afDataConfirm->hdr.status;
          if ( sentStatus != ZSuccess )
          {
            //ERROR_COUNT++;
            //if(ERROR_COUNT >2)
            //{
            //  ERROR_COUNT =0;
              ZDO_ACK_DEAL_WITH();
            //}
          }
          break;   
             
        case ZDO_STATE_CHANGE:  

          SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (SampleApp_NwkState == DEV_ZB_COORD)
              || (SampleApp_NwkState == DEV_ROUTER)
              || (SampleApp_NwkState == DEV_END_DEVICE) )
          {
                         
           HalLedBlink ( HAL_LED_1, 1, 99, 2500 );//用BLINK模拟灯亮两秒后熄灭
           NLME_SetPollRate( 5000 );//set the poll rate to quick ,and       
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
      //释放消息占用的内存
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next - if one is available
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    }//end： while ( MSGpkt )

    return (events ^ SYS_EVENT_MSG);
  }

if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT )//发送周期消息事件
  {
 
    if(Heart_mesg_count_ACK%30==0) //暂定半小时使用ACK一次，如果失败了则重新入网  gai 
    {
      //NwkPollReq(0);
      HEART_NEED_ACK_FLAG =1;
      
      if(Heart_mesg_count_ACK%60==0)   //gai
      {   
        uint16 active_battery_voltage=0;
        Heart_mesg_count_ACK = 0;
        //增加电池检测的功能
        active_battery_voltage=myApp_ReadBattery();
        
        if(active_battery_voltage<=270)//2.7V以下则上报欠压  gai
        {
          if( (devState==DEV_END_DEVICE) || (devState==DEV_ROUTER) )
          {
            BUTTERY_LOW_FLAG =1;
            SampleApp_SendInMessage();
          }
        }
        // 低于2.5V 则不应许报警 ,检测电压值比实际值小0.1V左右。所以当电池跌到2.5V时则不让报警
        
      }
    }
    
        SampleApp_SendPeriodicMessage();

    
      osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
                              (55000+ 30*(osal_rand() & 0x00FF)) );
    
      Heart_mesg_count_ACK++;
    

 

    return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT);
  }

  
    if ( events & NEW_JOIN_ROLE_EVT )
  {
    APP_JOIN_START_EVT();
     return ( events ^ NEW_JOIN_ROLE_EVT);
  }

/*向COMMUNICTION报告入网信息*/
  if ( events & SAMPLEAPP_SEND_NWKADDR_EVT )
  {
    // 路由用0，ID  终端用1，ID
    SampleApp_SendNwkaddrMessage(1,WATER_SENSOR_ID);
    osal_start_timerEx( SampleApp_TaskID, OS_TEST_EVT,20000);
    return ( events ^ SAMPLEAPP_SEND_NWKADDR_EVT);
  }

     if ( events & OS_TEST_EVT )
  {
        APP_OS_DEAL_WITH(18000);
      //osal_start_timerEx( SampleApp_TaskID, HALF_HOUR_NO_POLL_EVT,60000);//1分钟
       return ( events ^ OS_TEST_EVT);
  }
  
  
   if ( events & HALF_HOUR_NO_POLL_EVT )
  {
       no_poll_time_count++;
      if(no_poll_time_count>5) //5分钟后
      {
        no_poll_time_count=0;
       NLME_SetPollRate(0); //直接不POLL了，靠心跳的POLL来做
//       DEEP_POLL_FLAG =1;
      }
      else
      {
        osal_start_timerEx( SampleApp_TaskID, HALF_HOUR_NO_POLL_EVT,60000);
      }
       
       return ( events ^ HALF_HOUR_NO_POLL_EVT);
  }
  
   if ( events & SAMPLEAPP_RESTORE_EVT )
  {
       //  SystemResetSoft();
        HAL_SYSTEM_RESET();
  }
  


  

  return 0;
}

void SampleAPP_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg )
{
 
  switch ( inMsg->clusterID )
  {
    case End_Device_Bind_rsp:
      if ( ZDO_ParseBindRsp( inMsg ) == ZSuccess )
      {
        // Light LED
       HalLedBlink ( HAL_LED_2, 3, 50, 1000 );
      }
#if defined(BLINK_LEDS)
      else
      {
        HalLedBlink ( HAL_LED_2, 6, 50, 500 );
      }
#endif
      break;
  }

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
//  uint8 logicalType;
  zAddrType_t dstAddr;
  
  //初定，2次按键为申请加入网络，3次按键为软复位，4次按键为恢复出厂设置（暂时还没想好如何实现）
  if ( keys &  HAL_KEY_SW_8 )
  {     
    
      if(key_double_flag==1)
    {
     
    if(water_state == 1)
    {
        water_state = 0;
        SampleApp_SendInMessage();
    }
    }
    
   
     else if(key_double_flag==4)
    {
       APPLY_TO_JOIN_OF_KEY();
    }

     if(key_hold_flag==1) 
    {   
      key_hold_flag=0;
       HalLedBlink ( HAL_LED_1, 4, 50, 600 );
      RESTORE_TO_FACTORY();
      osal_start_timerEx( SampleApp_TaskID,SAMPLEAPP_RESTORE_EVT,5000);
     }
  }
    if ( (keys &  HAL_KEY_SW_9) || (keys &  HAL_KEY_SW_6) )
  { 
    
   if( (devState==DEV_END_DEVICE) || (devState==DEV_ROUTER) )
   {
     if(water_state == 0)
     {
     water_state = 1;
    SampleApp_SendInMessage();
     }
    if(bindNumReflections(SAMPLEAPP_ENDPOINT,SAMPLEAPP_ONOFF_CLUSTERID)>0)//有绑定项目
    {
      uint8 ONOROFF =0;
       ONOROFF = PUSH4_SBIT;  //开         
     if ( AF_DataRequest( &SampleApp_BIND_DstAddr, &SampleApp_epDesc,
                           SAMPLEAPP_ONOFF_CLUSTERID,
                           1,  //数据长度
                           &ONOROFF,//###添加的发送数据        
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
   }
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
//  uint16 flashTime;
  switch ( pkt->clusterId )
  {

    case SAMPLEAPP_COMMAND_CLUSTERID:
      
      SampleApp_ProcessCommandMessage(pkt);

      break;     

   case SAMPLEAPP_FLASH_CLUSTERID:
      
      if( (pkt->cmd.Data[0]==1) && (pkt->cmd.Data[1]==2) && (pkt->cmd.Data[2]==3))
      {
       OS_FLAG=1;
      }
      
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
/*0 1 2 3 4 5 6  7  8   9  10  11  12   13   14 ...*/
/*_ _ _ / C / U  U  X   X   X   X   X   D ATA...*/

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
    
 if(pkt->cmd.Data[8]=='E') // /C/UUEXXXX10 上位机手动消除警告
 {
     str[9]='/';
     if(pkt->cmd.Data[13]=='0')
   {
     str[10]=0;
     water_state = 0;
     data_sucess_flag=1;
     active_count = 11;
   }
 
 }
 
  else if(pkt->cmd.Data[8]=='L')//v:118
 {
  uint8 blink_count=0;
  blink_count = (pkt->cmd.Data[13]-0x30)*100 + (pkt->cmd.Data[14]-0x30)*10 + (pkt->cmd.Data[15]-0x30);
  HalLedBlink ( HAL_LED_2, blink_count, 50, 800 );

 }

 else if(pkt->cmd.Data[8]==82)//R:86 刷新设备列表 特殊处理，直接按一定格式上报
 {
   data_sucess_flag=1;
   str[9]='R';
   Command_t=str[9];

   uint8 i;
#if defined(RTR_NWK)
  str[10]=Send_Router;
#else
  str[10]=Send_Endvice;
#endif  
  

  str[11]=WATER_SENSOR_ID;
 
  
  Sample_extendaddr=SampleApp_GetExtendAddr();
  for(i=0;i<16;i++)
  {
    str[i+12]=*Sample_extendaddr++;
  }
   osal_mem_free( extendaddr_mem );
  
   str[29]=water_state;
   
   active_count = 30;
 }
  
  osal_mem_free( shortddr_mem );
  
//---------------------------------    
    if((pkt->cmd.Data[7]==85) && (data_sucess_flag==1) )
  {
    str[8]='U';
    SampleApp_SPI_SendCommand_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
    SampleApp_SPI_SendCommand_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
    //SampleApp_SPI_SendCommand_DstAddr.addr.shortAddr = 0X0000;
   SampleApp_SPI_SendCommand_DstAddr.addr.shortAddr = pkt->srcAddr.addr.shortAddr;

    
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
  }
}

void SampleApp_ProcessBINDINGMessage(afIncomingMSGPacket_t *pkt)
{

 
}



/*
  告警输入处理函数，如红外和门磁或者报警器和开关等 包括红外的放拆报警 欠压 等
*/
void SampleApp_SendInMessage(void)
{

  uint8 active_str[15];
  uint8 active_count = 0;
  uint8 *Sample_Cmd_shorAddr;

  Sample_Cmd_shorAddr=SampleApp_GetShortAddr();
  active_str[0]=*Sample_Cmd_shorAddr++;
  active_str[1]=*Sample_Cmd_shorAddr++;
  active_str[2]=*Sample_Cmd_shorAddr++;
  active_str[3]=*Sample_Cmd_shorAddr;
  osal_mem_free( shortddr_mem );

  active_str[4]='/';
  active_str[5]='/';
  active_str[6]='/';
  active_str[7]='/';
  active_str[8]='/';

  
  active_str[9]=0xA1;//报警类数据  电池 放拆等
   
  if(BUTTERY_LOW_FLAG==1)
   {  
    active_str[9]=0xA1;//报警类数据  电池 放拆等
    active_str[10]=1;//1代表 电池告警标记位
    active_str[11]=1; //代表 电池告警事件
    active_count = 12;
    BUTTERY_LOW_FLAG = 0;
   }
   
 else
 {
   active_str[9]='/';
   active_str[10]= water_state;
   HalLedBlink ( HAL_LED_2, 1, 50, 600);
      

    active_count = 11;
  }
  
  SampleApp_SPI_SendCommand_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  SampleApp_SPI_SendCommand_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_SPI_SendCommand_DstAddr.addr.shortAddr = 0X0000;
    
    if ( AF_DataRequest( &SampleApp_SPI_SendCommand_DstAddr,
                         (endPointDesc_t *)&SampleApp_epDesc,
                          SAMPLEAPP_COMMAND_CLUSTERID, //终端传过去，协调器处理了
                          active_count, active_str,
                          &SampleApp_TransID, 
                          AF_DISCV_ROUTE, 
                          AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
    {
        
    }
    else
    {
      
    }   
}


void Delay_device_1u(uint32 microSecs) 
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
*看门狗初始化
*************************************************************/
void Init_Watchdog(void)
{
	WDCTL = 0x00;
	//时间间隔一秒，看门狗模式
	WDCTL |= 0x08;
	//启动看门狗
}


/************************************************************
*喂狗函数
*************************************************************/
void FeetDog(void)
{
	WDCTL = 0xa0;
	WDCTL = 0x50;
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

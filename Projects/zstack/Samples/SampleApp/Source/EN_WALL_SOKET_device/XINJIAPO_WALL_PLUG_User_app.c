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
/*******************************************************************************
文件名称：SR_SOCKET_User_app.C
文件功能：用户主文件
设计人员：
设计时间：20141014
修改记录：无
备注说明：包含用户需要填充的所有回调函数，用户只需填充回调函数即可
*******************************************************************************/
/*******************************************************************************
文件包含
*******************************************************************************/
#include "AF.h"
#include "ZDApp.h"
#include "device.h"
#include "OnBoard.h"
#include "MT_UART.h"

/* HAL */
#include "hal_led.h"
#include "hal_key.h"
#include "OSAL_Nv.h"

#include "AddrMgr.h"

#include "ZDObject.h"
#include "ZDO_LIB.h"
#include "common_device.h"
#include "CS5463_spi.h" 
#include "IO_config.h"
/*******************************************************************************
全局变量
*******************************************************************************/  
uint8 UserApp_TaskID;//用户注册的任务的编号，系统自动分配
uint8 RS_LEN=0;
uint8 RS_STR[10];

uint8 DEVICE_ID;
uint8 DEFINED_CONTROL_CHAR;
uint8 JOIN_ROLE;

uint8 SHORT_ADRESS[4];
uint8 USER_DATA_LEN=0;
uint8 USER_DATA_TYPE=0;
uint8 *USER_DATA_STR;
uint32 P_Value;
extern uint8 KEY_NUMBER_FLAG_A ;//按键标志

// 节点相关的全局变量-----------------------------------------------------------

cId_t  USER_SampleApp_ClusterList[SAMPLEAPP_MAX_CLUSTERS]=
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

endPointDesc_t SampleApp_epDesc1;
endPointDesc_t SampleApp_epDesc2;

SimpleDescriptionFormat_t SampleApp_SimpleDesc1 =
{
    SAMPLEAPP_ENDPOINT1,              //  int Endpoint;
    SAMPLEAPP_PROFID,                //  uint16 AppProfId[2];
    SAMPLEAPP_DEVICEID,              //  uint16 AppDeviceId[2];
    SAMPLEAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
    SAMPLEAPP_FLAGS,                 //  int   AppFlags:4;
    SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
    (cId_t *)USER_SampleApp_ClusterList,  //  uint8 *pAppInClusterList;
    SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
    (cId_t *)USER_SampleApp_ClusterList   //  uint8 *pAppInClusterList;
};

SimpleDescriptionFormat_t SampleApp_SimpleDesc2 =
{
    SAMPLEAPP_ENDPOINT2,              //  int Endpoint;
    SAMPLEAPP_PROFID,                //  uint16 AppProfId[2];
    SAMPLEAPP_DEVICEID,              //  uint16 AppDeviceId[2];
    SAMPLEAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
    SAMPLEAPP_FLAGS,                 //  int   AppFlags:4;
    SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
    (cId_t *)USER_SampleApp_ClusterList,  //  uint8 *pAppInClusterList;
    SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
    (cId_t *)USER_SampleApp_ClusterList   //  uint8 *pAppInClusterList;
};
//绑定命令
cId_t USER_bindingINClusters[1]=
{
    SAMPLEAPP_ONOFF_CLUSTERID
};

//计量的全局变量--------------------
uint32 	P_Value_Pre, P_Value_Now, P_Value_energy = 0;//功率的当前值和前一个功率值
uint32 	P_Protect_Value = 10;//功率保护值
uint16  P_Value_Distance = 100;//功率上报的差值

uint16 	P_Value_jiaozhun_Temp = 0, I_Value_jiaozhun_Temp = 0;//功率校准值
float  P_Value_Bili_Temp = 1.0, I_Value_Bili_Temp = 1.0;//功率的比例校准
uint16 	P_Value_Bili , I_Value_Bili ;//功率的比例校准,电流比例校准
uint32 Total_P_Value = 0,Total_P_Value_Temp = 0;//总功率值，使用WH为单位；
uint32 I_Value_Now;//当前电流值
uint8  CS5463_STR[10];//0-已校准的标记 ； 1、2- 功率偏移；  3、4-功率的比例；  5、6-电流的偏移； 7、8-电流的比例
/*******************************************************************************
函数声明
*******************************************************************************/
uint8 USER_APP_COMMAND(uint8 *user_app_data,uint8 *str); 
void USER_APP_JOIN_MSG(void);
void SampleApp_ProcessBINDINGMessage(afIncomingMSGPacket_t *pkt);

void User_APP_Drive_init(void);
void SampleApp_HandleKeys( uint8 shift, uint8 keys );//本用户应用按键函数
void USER_SET_RS_MSG(void);//设置RS数据

void USER_ZDO_JOINING_MSG(void); //ZDO层入网的时候处理函数 就是闪烁灯
uint8 READ_NETKEY(void); //读取入网状态按键的状态传递到应用层
uint8 LED_CHANGE(uint8 Position,uint8 Mode);// 设置LED函数
void SampleAPP_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );

uint8 CS5463_Send_PWER(void);
void P_JiaoZhun_Process(void);
void USER_AFTER_RS_MSG(void);

void USER_AFTER_RS_MSG(void)
{

}
/*******************************************************************************
* 函数名称: UserApp_Init
* 函数功能: 用户的初始化函数
* 入口参数: task_id：用户的任务的编号，是系统分配的，用户不用操心    
* 出口参数: 无
* 备注说明: 无
******************************************************************************/
void UserApp_Init( uint8 task_id )
{
    
    UserApp_TaskID = task_id;
    
    RegisterForKeys(task_id);
    
    SampleApp_epDesc1.endPoint = SAMPLEAPP_ENDPOINT1;//SampleApp EP描述符的EP号：20
    SampleApp_epDesc1.task_id = &SampleApp_TaskID;//SampleApp EP描述符的任务ID：0
    SampleApp_epDesc1.simpleDesc = (SimpleDescriptionFormat_t *)&SampleApp_SimpleDesc1;//SampleApp EP简单描述符
    SampleApp_epDesc1.latencyReq = noLatencyReqs;//延时策略
    afRegister( &SampleApp_epDesc1 );
    
    SampleApp_epDesc2.endPoint = SAMPLEAPP_ENDPOINT2;//SampleApp EP描述符的EP号：20
    SampleApp_epDesc2.task_id = &SampleApp_TaskID;//SampleApp EP描述符的任务ID：0
    SampleApp_epDesc2.simpleDesc  = (SimpleDescriptionFormat_t *)&SampleApp_SimpleDesc2;//SampleApp EP简单描述符
    SampleApp_epDesc2.latencyReq = noLatencyReqs;//延时策略
    afRegister( &SampleApp_epDesc2 );
    
    ZDO_RegisterForZDOMsg( UserApp_TaskID, End_Device_Bind_rsp );//注册终端节点绑定消息接收事件，用于截取绑定类的消息  
    
    User_APP_Drive_init();
    
}
/*******************************************************************************
* 函数名称: UserApp_ProcessEvent
* 函数功能: 事件处理函数，此函数会被系统轮询
* 入口参数: task_id：用户的任务的编号，是系统分配的，用户不用操心  
events：事件mark表 
* 出口参数: 无
* 备注说明: 此函数的函数头固定不变，用户不必操心
******************************************************************************/
UINT16 UserApp_ProcessEvent( uint8 task_id, uint16 events )
{
    afIncomingMSGPacket_t *MSGpkt;
    
    //系统事件，在这里主要处理按键----------------------------------------------
    if ( events & SYS_EVENT_MSG )
    {
        MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( UserApp_TaskID );
        while ( MSGpkt )                                           
        {                                                                     
            switch ( MSGpkt->hdr.event )
            {
                case KEY_CHANGE:      
                SampleApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
                break;                                             
                
                case ZDO_CB_MSG:
                SampleAPP_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
                break;
                
                default:
                break;
            }
            
            // Release the memory
            //释放消息占用的内存
            osal_msg_deallocate( (uint8 *)MSGpkt );
            
            // Next - if one is available
            MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( UserApp_TaskID );
        }//end： while ( MSGpkt )
        
        return (events ^ SYS_EVENT_MSG);
    }
    
    
    //用户自行添加的事件处理----------------------------------------------------
    if ( events & USER_SEND_MESSAGE_EVT )//每两秒读取一次CS5463操作
    {

        
        
        osal_nv_item_init(CS5463_JIAOZHUN_NV_FLAG, 10, CS5463_STR);  
        osal_nv_read(CS5463_JIAOZHUN_NV_FLAG, 0, 10,   CS5463_STR);
        
        if(CS5463_STR[0] == 11)//如果已经保存了校准值，则直接读取已经保存的校准值
        {
            
            P_Value_Bili_Temp = (float)((uint16)CS5463_STR[3] << 8 | (uint16)CS5463_STR[4]) / 1000;//存储的时候*1000
            P_Value_jiaozhun_Temp = (uint16)CS5463_STR[1] << 8 | (uint16)CS5463_STR[2];
            I_Value_Bili_Temp = (float)((uint16)CS5463_STR[7] << 8 | (uint16)CS5463_STR[8]) / 1000;//存储的时候*1000
            I_Value_jiaozhun_Temp = (uint16)CS5463_STR[5] << 8 | (uint16)CS5463_STR[6];
            
        }       
        
        CS5463_Send_PWER();
        osal_start_timerEx( UserApp_TaskID, USER_SEND_MESSAGE_EVT,2000);//事件延时        
        return (events ^ USER_SEND_MESSAGE_EVT );
    } 
    else if(events & USER_SEND_DS_MESSAGE_EVT)//这是一个一次性的事件
    {
        SampleApp_SendInMessage(); 
        osal_start_timerEx( UserApp_TaskID, USER_SEND_MESSAGE_EVT,2000);//事件延时 
        return (events ^ USER_SEND_DS_MESSAGE_EVT );		
    }
    else if(events & USER_SEND_MESSAGE_5min_EVT)//5分钟发送一次数据
    {
        SampleApp_SendInMessage(); 
        osal_start_timerEx( UserApp_TaskID, USER_SEND_MESSAGE_5min_EVT,60000 * 5);//事件延时        
        return (events ^ USER_SEND_MESSAGE_5min_EVT );		
    }
    
    return 0;
}


/*******************************************************************************
* 函数名称: SampleApp_HandleKeys
* 函数功能: 按键命令传输到USER层，用户自行添加删除，处理按键命令
* 入口参数: shift：状态码 
keys：按键的值
* 出口参数: 无
* 备注说明: 此函数的函数头固定不变，用户不必操心
******************************************************************************/ 
void SampleApp_HandleKeys( uint8 shift, uint8 keys )
{
    //P1.3  
    if ( KEY_NUMBER_FLAG_A &  HAL_KEY_SW_9 )
    { 
        if(P1_6 == 1)  
        {   
            P1_6 =0;//打开继电器
            HAL_TURN_ON_LED2();
        }
        
        else if(P1_6 == 0)  
        {
            P1_6 =1;//关闭继电器
            HAL_TURN_OFF_LED2();
        }
	
        if(key_double_flag == 3)
        {
            SampleApp_send_in_binding_req(SAMPLEAPP_ENDPOINT1,USER_bindingINClusters,1);//绑定1号端点
	}
        
        if( (devState==DEV_END_DEVICE) || (devState==DEV_ROUTER) )
        {
            SampleApp_SendInMessage();
        }
        
    }
    
    else if ( KEY_NUMBER_FLAG_A &  HAL_KEY_SW_7 )
    {      
        
        
        if(IO_CTLA_SBIT == 1)  
        {   
            IO_CTLA_SBIT =0;//打开继电器
            P1_7 = 0;//led
        }
        
        else if(IO_CTLA_SBIT == 0)  
        {
            IO_CTLA_SBIT =1;//关闭继电器
            P1_7 = 1;//led
        }
        
        
        if(key_double_flag == 3)
        {
            SampleApp_send_in_binding_req(SAMPLEAPP_ENDPOINT2,USER_bindingINClusters,1);//绑定2号端点
	}
	
        if( (devState==DEV_END_DEVICE) || (devState==DEV_ROUTER) )
        {
            SampleApp_SendInMessage();
        }
    }   
    
    else if ( keys&  HAL_KEY_SW_8  )
    {   
        
        
        if(key_double_flag==4)//快速按了4次按键
        {
            APPLY_TO_JOIN_OF_KEY();//函数中带有闪灯的代码
        }
        
        else if( (key_double_flag == 3) )
        {
            SampleApp_send_in_binding_req(SAMPLEAPP_ENDPOINT,USER_bindingINClusters,1);
        }
        
        else if(key_double_flag==6)
        {
            SampleApp_PermitJoin(240);
        }
        
        else if(key_double_flag==8)
        {
            SampleApp_PermitJoin(0);
        }
        
        
        
        if(key_hold_flag==1) //一个长按键，则恢复出厂设置
        {   
            HalLedBlink ( HAL_LED_1, 4, 50, 1000 );
            RESTORE_TO_FACTORY();
            osal_start_timerEx( SampleApp_TaskID,SAMPLEAPP_RESTORE_EVT,5000 );
        }        
    } 
    
    KEY_NUMBER_FLAG_A = 0;//清除按键标志
}


/*******************************************************************************
* 函数名称: SampleApp_ProcessBINDINGMessage
* 函数功能: 与绑定有关的数据的处理
* 入口参数: pkt：指向接收到的绑定请求信息的指针
* 出口参数: 无
* 备注说明: 此函数的函数头固定不变，用户不必操心
******************************************************************************/ 
void SampleApp_ProcessBINDINGMessage(afIncomingMSGPacket_t *pkt)
{
    
    if ((pkt->clusterId == SAMPLEAPP_ONOFF_CLUSTERID )&& (pkt->endPoint == SAMPLEAPP_ENDPOINT1))
    {
        if(pkt->cmd.Data[0]==1)//开命令
        {  
            P1_6 =0;//打开继电器
            HAL_TURN_ON_LED2();
        }
        
        else if(pkt->cmd.Data[0]==0 )//关
        {
            P1_6 =1;//关闭继电器
            HAL_TURN_OFF_LED2();
        }
        
        else if(pkt->cmd.Data[0]==2 )//2012 7 24 增加2 查询当前状态
        {
            
            P1_6 = !P1_6;
            LED2_SBIT = P1_6;
        }
    }
    else if(((pkt->clusterId == SAMPLEAPP_ONOFF_CLUSTERID )&& (pkt->endPoint == SAMPLEAPP_ENDPOINT2)))
    {
        if(pkt->cmd.Data[0]==1)//开命令
        {  
            IO_CTLA_SBIT =0;//打开继电器
            P1_7 = 0;//led             
        }
        
        else if(pkt->cmd.Data[0]==0)//关
        {
            IO_CTLA_SBIT =1;//关闭继电器
            P1_7 = 1;//led
        }
        
        else if(pkt->cmd.Data[0]==2 )//2012 7 24 增加2 查询当前状态
        {
            IO_CTLA_SBIT = !IO_CTLA_SBIT;
            P1_7 = IO_CTLA_SBIT;
        }
    }    			
    
    SampleApp_SendInMessage( );
    
    
}


/*******************************************************************************
* 函数名称: USER_APP_JOIN_MSG
* 函数功能: 入网后的回调函数，用于处理用户的加入网络后需要处理的操作代码
* 入口参数: 无
* 出口参数: 无
* 备注说明: 无
******************************************************************************/ 
void USER_APP_JOIN_MSG(void)
{
    uint8 *Sample_Cmd_shorAddr;
    
    HalLedBlink ( HAL_LED_1, 1, 99, 2500 );//用BLINK模拟灯亮两秒后熄灭  
    
    Sample_Cmd_shorAddr=SampleApp_GetShortAddr();
    SHORT_ADRESS[0]=*Sample_Cmd_shorAddr++;
    SHORT_ADRESS[1]=*Sample_Cmd_shorAddr++;
    SHORT_ADRESS[2]=*Sample_Cmd_shorAddr++;
    SHORT_ADRESS[3]=*Sample_Cmd_shorAddr;
    osal_mem_free( shortddr_mem );
    
    // 保证在发送上线消息RS的后面紧跟一条DS的初始数据，5S时间--------
    osal_start_timerEx( UserApp_TaskID, USER_SEND_DS_MESSAGE_EVT,5000); 
    
    //用户自己的事件,每隔五分钟发送一次数据--------------------------
    osal_start_timerEx( UserApp_TaskID, USER_SEND_MESSAGE_5min_EVT,60000 * 5); 
    
}



/*******************************************************************************
* 函数名称: USER_APP_COMMAND
* 函数功能: 收到命令后的处理函数
* 入口参数: user_app_data 上位机发过来的有效数据 字母后面开始
str：需要填入的返回数据数组
* 出口参数: 无
* 备注说明: 无
******************************************************************************/ 
uint8 USER_APP_COMMAND(uint8 *user_app_data,uint8 *str)
{
    uint8 active_len=0;
    
    str[0] = '/';	
    if(user_app_data[0]=='1')
    {
        if(user_app_data[1]=='1' )//开命令
        {  
            P1_6 =0;//打开继电器
            HAL_TURN_ON_LED2();
            
            str[1]=1;//第1个插座
            str[2]=1;
            active_len=12;
            return active_len;
        }
        
        else if(user_app_data[1]=='0' )//关
        {
            P1_6 =1;//关闭继电器
            HAL_TURN_OFF_LED2();
            
            str[1]=1;//第1个插座
            str[2]=0;
            active_len=12;
            return active_len;
        }
        
        else if(user_app_data[1]=='2' )//2012 7 24 增加2 查询当前状态
        {
            P1_6 = !P1_6;
            LED2_SBIT = P1_6;
            active_len=12;
            return active_len;
        }
        else if(user_app_data[1]=='3' )//2012 7 24 增加3 切换当前状态
        {
            IO_CTLA_SBIT = !IO_CTLA_SBIT;
            LED2_SBIT = IO_CTLA_SBIT;
            str[1]=1;//第1个插座
            str[2]=!IO_CTLA_SBIT;
            active_len=12;
            return active_len;;
        }
    }
    else if(user_app_data[0]=='2')
    {
	
        if(user_app_data[1]=='1' )//开命令
        {  
            IO_CTLA_SBIT =0;//打开继电器
            P1_7 = 0;//led    
            
            str[1]=2;//第2个插座
            str[2]=1;
            active_len=12;
            return active_len;
        }
        
        else if(user_app_data[1]=='0' )//关
        {
            IO_CTLA_SBIT =1;//关闭继电器
            P1_7 = 1;//led
            
            str[1]=2;//第2个插座
            str[2]=0;
            active_len=12;
            return active_len;
        }
        
        else if(user_app_data[1]=='2' )//2012 7 24 增加2 查询当前状态
        {
            str[1]=2;//第2个插座
            str[2]=!IO_CTLB_SBIT;
            active_len=12;
            return active_len;
        }
        else if(user_app_data[1]=='3' )//2012 7 24 增加3 切换当前状态
        {
            str[1]=2;//第2个插座
            str[2]=IO_CTLB_SBIT;
            
            IO_CTLA_SBIT = !IO_CTLA_SBIT;
            P1_7 = IO_CTLA_SBIT;
            
            active_len=12;
            return active_len;;
        }
    }    
    else if(user_app_data[0]=='5')
    {
        SampleApp_SendInMessage( );
        active_len=11;
        return active_len;;
    }
    else if(user_app_data[0]=='6')
    {
        P_Value_Distance = (user_app_data[1]-'0')*1000 +(user_app_data[2]-'0')*100 + (user_app_data[3]-'0')*10 + (user_app_data[4]-'0');
        str[1] = 6;
        str[2] = P_Value_Distance/256;
        str[3] = P_Value_Distance%256;			
        active_len=13;
        return active_len;;	
    }
    else if(user_app_data[0]=='7')	
    {
        P_Value_jiaozhun_Temp = (user_app_data[1]-'0')*1000 +(user_app_data[2]-'0')*100 + (user_app_data[3]-'0')*10 + (user_app_data[4]-'0');
        str[1] = 7;
        str[2] = P_Value_jiaozhun_Temp/256;
        str[3] = P_Value_jiaozhun_Temp%256;			
        active_len=13;
        return active_len;;	
    }
    else if(user_app_data[0]=='8')	
    {
        P_Protect_Value = (user_app_data[1]-'0')*1000 +(user_app_data[2]-'0')*100 + (user_app_data[3]-'0')*10 + (user_app_data[4]-'0');
        str[1] = 8;
        str[2] = P_Protect_Value/256;
        str[3] = P_Protect_Value%256;			
        active_len=13;
        return active_len;;	
    }
    else if(user_app_data[0]=='9')
    {
        
        P_Value = GetPactive(1.0, 10000, 0);//获取有功功率
        P_Value_Now = P_Value*1.2;//*12000/10000得到最终的有效的功率值	
        I_Value_Now = GetCurrentIrms(1,10000,0);//读取电流值
        
        if(user_app_data[1] == '1')//差值校准,0.1A电流，220V电压
        {
            if(I_Value_Now > 1000)
            {
                I_Value_jiaozhun_Temp = I_Value_Now - 1000;//电流校准的差值
            }
            
            P_Value_Now *= P_Value_Bili_Temp;
            
            if(P_Value_Now > 22)
            {
                P_Value_jiaozhun_Temp = P_Value_Now - 22;//22W的校准
            }
            
            str[2]=1;
        }
        else if(user_app_data[1] == '2')//比例校准，8A电流，220V电压
        {
            if(I_Value_Now > 80000)//8A电流
            {
                I_Value_Bili_Temp = 80000 / I_Value_Now;//电流校准的比例值
            }
            else
            {
                I_Value_Bili_Temp =  I_Value_Now / 80000;//电流校准的比例值
            }
            
            P_Value_Now -= P_Value_jiaozhun_Temp;
            
            if(P_Value_Now > 1760)
            {
                P_Value_Bili_Temp = 1760.0 / (float)P_Value_Now;
            }
            else 
            {
                P_Value_Bili_Temp = (float)P_Value_Now / 1760.0 ;
            }
            
            str[2]=2;
        }
        else if(user_app_data[1] == '3')//确认校准值
        {
            
            CS5463_STR[0] = 11;//已经校准的标志
            P_Value_Bili = (uint16)(P_Value_Bili_Temp * 1000);//保存校准值
            CS5463_STR[3] = P_Value_Bili >> 8;
            CS5463_STR[4] = P_Value_Bili ;
            
            CS5463_STR[1] = P_Value_jiaozhun_Temp>>8;
            CS5463_STR[2] = P_Value_jiaozhun_Temp;
            
            CS5463_STR[5] = I_Value_jiaozhun_Temp>>8;
            CS5463_STR[6] = I_Value_jiaozhun_Temp;	
            
            I_Value_Bili = (uint16)(I_Value_Bili_Temp * 1000);
            CS5463_STR[7] = I_Value_Bili >> 8;
            CS5463_STR[8] = I_Value_Bili ;	
            
            osal_nv_item_init(CS5463_JIAOZHUN_NV_FLAG, 10, CS5463_STR);  //b保存到FLASH
            osal_nv_write(CS5463_JIAOZHUN_NV_FLAG, 0, 10, CS5463_STR);					
            
            str[2]=3;
            
            
        }
        else if(user_app_data[1] == '0')
        {
            P_Value_Bili_Temp = 1.0;
            P_Value_jiaozhun_Temp = 0;
            I_Value_jiaozhun_Temp = 0;
            I_Value_Bili_Temp = 1.0;
            
            CS5463_STR[0] = 0;
            CS5463_STR[1] = 0;	
            CS5463_STR[2] = 0;
            CS5463_STR[3] = 0;	
            CS5463_STR[4] = 0;
            CS5463_STR[5] = 0;	
            CS5463_STR[6] = 0;
            CS5463_STR[7] = 0;
            CS5463_STR[8] = 0;
            CS5463_STR[9] = 0;			
            osal_nv_item_init(CS5463_JIAOZHUN_NV_FLAG, 10, CS5463_STR);  
            osal_nv_write(CS5463_JIAOZHUN_NV_FLAG, 0, 10, CS5463_STR);				
            
            str[2]=4;		
        }
        
	str[1]=9;
        active_len=12;
        return active_len;;
    }     
}

/*******************************************************************************
* 函数名称: USER_SET_RS_MSG
* 函数功能: 用户自行设置RS数据
* 入口参数: 无
* 出口参数: 无
* 备注说明: 入网后向协调器发送的自身信息数据，入网后5S会自动发送
******************************************************************************/
void USER_SET_RS_MSG(void)//
{
    
    DEVICE_ID=77;
    DEFINED_CONTROL_CHAR='E';
    JOIN_ROLE=Send_Router;
    RS_LEN = 1;
    RS_STR[0]=IO_CTLA_SBIT;
}

/*******************************************************************************
* 函数名称: SampleApp_SendInMessage
* 函数功能: 用户自行规划主动上报数据的格式。
* 入口参数: 无
* 出口参数: 无
* 备注说明: 无
******************************************************************************/
void SampleApp_SendInMessage()
{
    uint8 *send_str;
    uint8 send_count=0;
    send_count = 10+8;
    send_str = osal_mem_alloc(send_count);//申请数据缓存区
    
    USER_SEND_ADD_STR(send_str);  //\添加固定帧头
    
    send_str[9]='/';
    send_str[10]=9;
    send_str[11]=IO_CTLA_SBIT;//插座1的通断状态
    send_str[12]=IO_CTLB_SBIT;//插座2的通断状态
    send_str[13]=P_Value_Now/256;//当前的功率值
    send_str[14]=P_Value_Now%256;  
    send_str[15]=(uint8)P_Value_energy>>16;
    send_str[16]=(uint8)P_Value_energy>>8;
    send_str[17]=(uint8)P_Value_energy;
    
    
    USER_APP_SEND_IN(send_count,send_str);//发送出数据
    
    osal_mem_free( send_str );//释放数据缓存区
}

/*******************************************************************************
* 函数名称: User_APP_Drive_init
* 函数功能: USER自行添加应用初始化的代码，如初始化外设等
* 入口参数: 无
* 出口参数: 无
* 备注说明: 无
******************************************************************************/
void User_APP_Drive_init(void)
{   
    HAL_IO_CONFIG_INIT();
    InitCS5463();//初始化CS5463
    ConfigCS5463();//配置CS5463
    P_JiaoZhun_Process();
    P1DIR |= 0Xc0;//P1.7 ---led3
    
    //确保继电器和LED均处于关闭状态；
    P1_6 = 1;
    P1_7 = 1;
    P1_5 = 1;
    P0_1 = 1;
}


uint8 READ_NETKEY() //读取入网状态按键的状态传递到应用层
{
    return HAL_PUSH_BUTTON3();
}



/*
mode 0 关
1 开
2 切换
*/
uint8 LED_CHANGE(uint8 Position,uint8 Mode)// 设置LED函数
{
    switch (Position )
    {
        case HAL_LED_1:      
        {
            if(Mode==0 || Mode==1)
            {
                LED1_SBIT = LED1_POLARITY(Mode);
            }
            else if(Mode==2)
            {
                if (LED1_SBIT) 
                { 
                    LED1_SBIT = 0; 
                } 
                else 
                { 
                    LED1_SBIT = 1;
                } 
            }
        }   
        break;     
        
        case HAL_LED_2:      
        {
            if(Mode==0||Mode==1)
            {
                LED2_SBIT = LED2_POLARITY(Mode);
            }
            else if(Mode==2)
            {
                if (LED1_SBIT) 
                { 
                    LED2_SBIT = 0; 
                } 
                else 
                { 
                    LED2_SBIT = 1;
                } 
            }
        }   
        break;     
        
        case HAL_LED_3:      
        {
            if(Mode==0 || Mode==1)
            {
                LED3_SBIT = LED3_POLARITY(Mode);
            }
            else if(Mode==2)
            {
                if (LED3_SBIT) 
                { 
                    LED3_SBIT = 0; 
                } 
                else 
                { 
                    LED3_SBIT = 1;
                } 
            }
        }   
        break;     
        
        default:
        break;
    }
    return 1;
}


void USER_ZDO_JOINING_MSG(void)
{
    HalLedBlink ( HAL_LED_1, 1, 30, 500 );
}


void SampleAPP_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg )
{
    
    switch ( inMsg->clusterID )
    {
        case End_Device_Bind_rsp:
        if ( ZDO_ParseBindRsp( inMsg ) == ZSuccess )
        {
            // Light LED
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






/*****************************************************************************/
/*************************************用户函数********************************/
/*******************************************************************************
* 函数名称: CS5463_Send_PWER
* 函数功能: 每两秒钟读一次功率数据，并判断是否需要上传
* 入口参数: 无
* 出口参数: 无
* 备注说明: 无
******************************************************************************/
uint8 CS5463_Send_PWER(void)
{
    I_Value_Now = GetCurrentIrms(1,10000,0);//读取电流值
    if(I_Value_Now > 100)//0.01A
    {
        P_Value = GetPactive(1.0, 10000, 0);//获取有功功率
        P_Value_Now = (uint32)(P_Value*1.2);//*12000/10000得到最终的有效的功率值
        P_Value_Now = (float)P_Value_Now * P_Value_Bili_Temp;	
        
        if(P_Value_Now > P_Value_jiaozhun_Temp)
        {
            P_Value_Now -= P_Value_jiaozhun_Temp ;//去除校准值
            if((P_Value_Now >0)  &&  (P_Value_Now < 16))
            {
                P_Value_Now -= 1;
            }
        }
        else
        {
            P_Value_Now = 0;//小于校准值，则判断其应为0W
        }
        
    }
    else
    {
        P_Value_Now = 0;
    }  
    
    
    if(P_Value_Now<10)
    {
        P_Value_Distance = 1;
    }
    else if(P_Value_Now<100)
    {
        P_Value_Distance = 10;
    }
    else if(P_Value_Now < 500)
    {
        P_Value_Distance = 50;
    }
    else
    {
        P_Value_Distance = 100;
    }
    
    Total_P_Value_Temp += P_Value_Now * 2;//2s的功率w.s  以W.S为单位
    if(Total_P_Value_Temp >= 3600)//电量累计
    {
        Total_P_Value_Temp -= 3600;//3600W.S = 1 W.H
        P_Value_energy++;
        if(P_Value_energy >= 16000000)P_Value_energy = 16000000;
    }
    
    
    if(P_Value_Now > P_Protect_Value)//超过500W就关闭继电器
    {       	
        SampleApp_SendInMessage();//上报当前功率值  
        return 1;
    }
    else if(P_Value_Now > P_Value_Pre + P_Value_Distance)//超过【上次汇报】过的数据一定的数值就上报
    {        
        P_Value_Pre = P_Value_Now;//保存这一次发送的数值
        SampleApp_SendInMessage();
        return 1;
    }
    
    return 0;
}

/*******************************************************************************
* 函数名称: P_JiaoZhun_Process
* 函数功能: 功率校准值初始化
* 入口参数: 无
* 出口参数: 无
* 备注说明: 无
******************************************************************************/
void P_JiaoZhun_Process(void)
{	
    osal_nv_item_init(CS5463_JIAOZHUN_NV_FLAG, 10, CS5463_STR);  
    osal_nv_read(CS5463_JIAOZHUN_NV_FLAG, 0, 10,   CS5463_STR);
    
    if(CS5463_STR[0] == 11)//如果已经保存了校准值，则直接读取已经保存的校准值
    {
        
        P_Value_Bili_Temp = (float)((uint16)CS5463_STR[3] << 8 | (uint16)CS5463_STR[4]) / 1000;//存储的时候*1000
        P_Value_jiaozhun_Temp = (uint16)CS5463_STR[1] << 8 | (uint16)CS5463_STR[2];
        I_Value_Bili_Temp = (float)((uint16)CS5463_STR[7] << 8 | (uint16)CS5463_STR[8]) / 1000;//存储的时候*1000
        I_Value_jiaozhun_Temp = (uint16)CS5463_STR[5] << 8 | (uint16)CS5463_STR[6];
        
    }
    else//如果没有保存校准值，则直接使用默认校准值
    {
        P_Value_Bili_Temp = 1.0;
        P_Value_jiaozhun_Temp = 0;	
        I_Value_Bili_Temp = 0;
        I_Value_jiaozhun_Temp = 0;
    }
}



/**********************************END OF FILE********************************/
/*****************************************************************************/












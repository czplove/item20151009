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
/*******************************************************************************
�ļ����ƣ�SR_SOCKET_User_app.C
�ļ����ܣ��û����ļ�
�����Ա��
���ʱ�䣺20141014
�޸ļ�¼����
��ע˵���������û���Ҫ�������лص��������û�ֻ�����ص���������
*******************************************************************************/
/*******************************************************************************
�ļ�����
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
ȫ�ֱ���
*******************************************************************************/  
uint8 UserApp_TaskID;//�û�ע�������ı�ţ�ϵͳ�Զ�����
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
extern uint8 KEY_NUMBER_FLAG_A ;//������־

// �ڵ���ص�ȫ�ֱ���-----------------------------------------------------------

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
//������
cId_t USER_bindingINClusters[1]=
{
    SAMPLEAPP_ONOFF_CLUSTERID
};

//������ȫ�ֱ���--------------------
uint32 	P_Value_Pre, P_Value_Now, P_Value_energy = 0;//���ʵĵ�ǰֵ��ǰһ������ֵ
uint32 	P_Protect_Value = 10;//���ʱ���ֵ
uint16  P_Value_Distance = 100;//�����ϱ��Ĳ�ֵ

uint16 	P_Value_jiaozhun_Temp = 0, I_Value_jiaozhun_Temp = 0;//����У׼ֵ
float  P_Value_Bili_Temp = 1.0, I_Value_Bili_Temp = 1.0;//���ʵı���У׼
uint16 	P_Value_Bili , I_Value_Bili ;//���ʵı���У׼,��������У׼
uint32 Total_P_Value = 0,Total_P_Value_Temp = 0;//�ܹ���ֵ��ʹ��WHΪ��λ��
uint32 I_Value_Now;//��ǰ����ֵ
uint8  CS5463_STR[10];//0-��У׼�ı�� �� 1��2- ����ƫ�ƣ�  3��4-���ʵı�����  5��6-������ƫ�ƣ� 7��8-�����ı���
/*******************************************************************************
��������
*******************************************************************************/
uint8 USER_APP_COMMAND(uint8 *user_app_data,uint8 *str); 
void USER_APP_JOIN_MSG(void);
void SampleApp_ProcessBINDINGMessage(afIncomingMSGPacket_t *pkt);

void User_APP_Drive_init(void);
void SampleApp_HandleKeys( uint8 shift, uint8 keys );//���û�Ӧ�ð�������
void USER_SET_RS_MSG(void);//����RS����

void USER_ZDO_JOINING_MSG(void); //ZDO��������ʱ������ ������˸��
uint8 READ_NETKEY(void); //��ȡ����״̬������״̬���ݵ�Ӧ�ò�
uint8 LED_CHANGE(uint8 Position,uint8 Mode);// ����LED����
void SampleAPP_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );

uint8 CS5463_Send_PWER(void);
void P_JiaoZhun_Process(void);
void USER_AFTER_RS_MSG(void);

void USER_AFTER_RS_MSG(void)
{

}
/*******************************************************************************
* ��������: UserApp_Init
* ��������: �û��ĳ�ʼ������
* ��ڲ���: task_id���û�������ı�ţ���ϵͳ����ģ��û����ò���    
* ���ڲ���: ��
* ��ע˵��: ��
******************************************************************************/
void UserApp_Init( uint8 task_id )
{
    
    UserApp_TaskID = task_id;
    
    RegisterForKeys(task_id);
    
    SampleApp_epDesc1.endPoint = SAMPLEAPP_ENDPOINT1;//SampleApp EP��������EP�ţ�20
    SampleApp_epDesc1.task_id = &SampleApp_TaskID;//SampleApp EP������������ID��0
    SampleApp_epDesc1.simpleDesc = (SimpleDescriptionFormat_t *)&SampleApp_SimpleDesc1;//SampleApp EP��������
    SampleApp_epDesc1.latencyReq = noLatencyReqs;//��ʱ����
    afRegister( &SampleApp_epDesc1 );
    
    SampleApp_epDesc2.endPoint = SAMPLEAPP_ENDPOINT2;//SampleApp EP��������EP�ţ�20
    SampleApp_epDesc2.task_id = &SampleApp_TaskID;//SampleApp EP������������ID��0
    SampleApp_epDesc2.simpleDesc  = (SimpleDescriptionFormat_t *)&SampleApp_SimpleDesc2;//SampleApp EP��������
    SampleApp_epDesc2.latencyReq = noLatencyReqs;//��ʱ����
    afRegister( &SampleApp_epDesc2 );
    
    ZDO_RegisterForZDOMsg( UserApp_TaskID, End_Device_Bind_rsp );//ע���ն˽ڵ����Ϣ�����¼������ڽ�ȡ�������Ϣ  
    
    User_APP_Drive_init();
    
}
/*******************************************************************************
* ��������: UserApp_ProcessEvent
* ��������: �¼����������˺����ᱻϵͳ��ѯ
* ��ڲ���: task_id���û�������ı�ţ���ϵͳ����ģ��û����ò���  
events���¼�mark�� 
* ���ڲ���: ��
* ��ע˵��: �˺����ĺ���ͷ�̶����䣬�û����ز���
******************************************************************************/
UINT16 UserApp_ProcessEvent( uint8 task_id, uint16 events )
{
    afIncomingMSGPacket_t *MSGpkt;
    
    //ϵͳ�¼�����������Ҫ������----------------------------------------------
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
            //�ͷ���Ϣռ�õ��ڴ�
            osal_msg_deallocate( (uint8 *)MSGpkt );
            
            // Next - if one is available
            MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( UserApp_TaskID );
        }//end�� while ( MSGpkt )
        
        return (events ^ SYS_EVENT_MSG);
    }
    
    
    //�û�������ӵ��¼�����----------------------------------------------------
    if ( events & USER_SEND_MESSAGE_EVT )//ÿ�����ȡһ��CS5463����
    {

        
        
        osal_nv_item_init(CS5463_JIAOZHUN_NV_FLAG, 10, CS5463_STR);  
        osal_nv_read(CS5463_JIAOZHUN_NV_FLAG, 0, 10,   CS5463_STR);
        
        if(CS5463_STR[0] == 11)//����Ѿ�������У׼ֵ����ֱ�Ӷ�ȡ�Ѿ������У׼ֵ
        {
            
            P_Value_Bili_Temp = (float)((uint16)CS5463_STR[3] << 8 | (uint16)CS5463_STR[4]) / 1000;//�洢��ʱ��*1000
            P_Value_jiaozhun_Temp = (uint16)CS5463_STR[1] << 8 | (uint16)CS5463_STR[2];
            I_Value_Bili_Temp = (float)((uint16)CS5463_STR[7] << 8 | (uint16)CS5463_STR[8]) / 1000;//�洢��ʱ��*1000
            I_Value_jiaozhun_Temp = (uint16)CS5463_STR[5] << 8 | (uint16)CS5463_STR[6];
            
        }       
        
        CS5463_Send_PWER();
        osal_start_timerEx( UserApp_TaskID, USER_SEND_MESSAGE_EVT,2000);//�¼���ʱ        
        return (events ^ USER_SEND_MESSAGE_EVT );
    } 
    else if(events & USER_SEND_DS_MESSAGE_EVT)//����һ��һ���Ե��¼�
    {
        SampleApp_SendInMessage(); 
        osal_start_timerEx( UserApp_TaskID, USER_SEND_MESSAGE_EVT,2000);//�¼���ʱ 
        return (events ^ USER_SEND_DS_MESSAGE_EVT );		
    }
    else if(events & USER_SEND_MESSAGE_5min_EVT)//5���ӷ���һ������
    {
        SampleApp_SendInMessage(); 
        osal_start_timerEx( UserApp_TaskID, USER_SEND_MESSAGE_5min_EVT,60000 * 5);//�¼���ʱ        
        return (events ^ USER_SEND_MESSAGE_5min_EVT );		
    }
    
    return 0;
}


/*******************************************************************************
* ��������: SampleApp_HandleKeys
* ��������: ��������䵽USER�㣬�û��������ɾ��������������
* ��ڲ���: shift��״̬�� 
keys��������ֵ
* ���ڲ���: ��
* ��ע˵��: �˺����ĺ���ͷ�̶����䣬�û����ز���
******************************************************************************/ 
void SampleApp_HandleKeys( uint8 shift, uint8 keys )
{
    //P1.3  
    if ( KEY_NUMBER_FLAG_A &  HAL_KEY_SW_9 )
    { 
        if(P1_6 == 1)  
        {   
            P1_6 =0;//�򿪼̵���
            HAL_TURN_ON_LED2();
        }
        
        else if(P1_6 == 0)  
        {
            P1_6 =1;//�رռ̵���
            HAL_TURN_OFF_LED2();
        }
	
        if(key_double_flag == 3)
        {
            SampleApp_send_in_binding_req(SAMPLEAPP_ENDPOINT1,USER_bindingINClusters,1);//��1�Ŷ˵�
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
            IO_CTLA_SBIT =0;//�򿪼̵���
            P1_7 = 0;//led
        }
        
        else if(IO_CTLA_SBIT == 0)  
        {
            IO_CTLA_SBIT =1;//�رռ̵���
            P1_7 = 1;//led
        }
        
        
        if(key_double_flag == 3)
        {
            SampleApp_send_in_binding_req(SAMPLEAPP_ENDPOINT2,USER_bindingINClusters,1);//��2�Ŷ˵�
	}
	
        if( (devState==DEV_END_DEVICE) || (devState==DEV_ROUTER) )
        {
            SampleApp_SendInMessage();
        }
    }   
    
    else if ( keys&  HAL_KEY_SW_8  )
    {   
        
        
        if(key_double_flag==4)//���ٰ���4�ΰ���
        {
            APPLY_TO_JOIN_OF_KEY();//�����д������ƵĴ���
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
        
        
        
        if(key_hold_flag==1) //һ������������ָ���������
        {   
            HalLedBlink ( HAL_LED_1, 4, 50, 1000 );
            RESTORE_TO_FACTORY();
            osal_start_timerEx( SampleApp_TaskID,SAMPLEAPP_RESTORE_EVT,5000 );
        }        
    } 
    
    KEY_NUMBER_FLAG_A = 0;//���������־
}


/*******************************************************************************
* ��������: SampleApp_ProcessBINDINGMessage
* ��������: ����йص����ݵĴ���
* ��ڲ���: pkt��ָ����յ��İ�������Ϣ��ָ��
* ���ڲ���: ��
* ��ע˵��: �˺����ĺ���ͷ�̶����䣬�û����ز���
******************************************************************************/ 
void SampleApp_ProcessBINDINGMessage(afIncomingMSGPacket_t *pkt)
{
    
    if ((pkt->clusterId == SAMPLEAPP_ONOFF_CLUSTERID )&& (pkt->endPoint == SAMPLEAPP_ENDPOINT1))
    {
        if(pkt->cmd.Data[0]==1)//������
        {  
            P1_6 =0;//�򿪼̵���
            HAL_TURN_ON_LED2();
        }
        
        else if(pkt->cmd.Data[0]==0 )//��
        {
            P1_6 =1;//�رռ̵���
            HAL_TURN_OFF_LED2();
        }
        
        else if(pkt->cmd.Data[0]==2 )//2012 7 24 ����2 ��ѯ��ǰ״̬
        {
            
            P1_6 = !P1_6;
            LED2_SBIT = P1_6;
        }
    }
    else if(((pkt->clusterId == SAMPLEAPP_ONOFF_CLUSTERID )&& (pkt->endPoint == SAMPLEAPP_ENDPOINT2)))
    {
        if(pkt->cmd.Data[0]==1)//������
        {  
            IO_CTLA_SBIT =0;//�򿪼̵���
            P1_7 = 0;//led             
        }
        
        else if(pkt->cmd.Data[0]==0)//��
        {
            IO_CTLA_SBIT =1;//�رռ̵���
            P1_7 = 1;//led
        }
        
        else if(pkt->cmd.Data[0]==2 )//2012 7 24 ����2 ��ѯ��ǰ״̬
        {
            IO_CTLA_SBIT = !IO_CTLA_SBIT;
            P1_7 = IO_CTLA_SBIT;
        }
    }    			
    
    SampleApp_SendInMessage( );
    
    
}


/*******************************************************************************
* ��������: USER_APP_JOIN_MSG
* ��������: ������Ļص����������ڴ����û��ļ����������Ҫ����Ĳ�������
* ��ڲ���: ��
* ���ڲ���: ��
* ��ע˵��: ��
******************************************************************************/ 
void USER_APP_JOIN_MSG(void)
{
    uint8 *Sample_Cmd_shorAddr;
    
    HalLedBlink ( HAL_LED_1, 1, 99, 2500 );//��BLINKģ����������Ϩ��  
    
    Sample_Cmd_shorAddr=SampleApp_GetShortAddr();
    SHORT_ADRESS[0]=*Sample_Cmd_shorAddr++;
    SHORT_ADRESS[1]=*Sample_Cmd_shorAddr++;
    SHORT_ADRESS[2]=*Sample_Cmd_shorAddr++;
    SHORT_ADRESS[3]=*Sample_Cmd_shorAddr;
    osal_mem_free( shortddr_mem );
    
    // ��֤�ڷ���������ϢRS�ĺ������һ��DS�ĳ�ʼ���ݣ�5Sʱ��--------
    osal_start_timerEx( UserApp_TaskID, USER_SEND_DS_MESSAGE_EVT,5000); 
    
    //�û��Լ����¼�,ÿ������ӷ���һ������--------------------------
    osal_start_timerEx( UserApp_TaskID, USER_SEND_MESSAGE_5min_EVT,60000 * 5); 
    
}



/*******************************************************************************
* ��������: USER_APP_COMMAND
* ��������: �յ������Ĵ�����
* ��ڲ���: user_app_data ��λ������������Ч���� ��ĸ���濪ʼ
str����Ҫ����ķ�����������
* ���ڲ���: ��
* ��ע˵��: ��
******************************************************************************/ 
uint8 USER_APP_COMMAND(uint8 *user_app_data,uint8 *str)
{
    uint8 active_len=0;
    
    str[0] = '/';	
    if(user_app_data[0]=='1')
    {
        if(user_app_data[1]=='1' )//������
        {  
            P1_6 =0;//�򿪼̵���
            HAL_TURN_ON_LED2();
            
            str[1]=1;//��1������
            str[2]=1;
            active_len=12;
            return active_len;
        }
        
        else if(user_app_data[1]=='0' )//��
        {
            P1_6 =1;//�رռ̵���
            HAL_TURN_OFF_LED2();
            
            str[1]=1;//��1������
            str[2]=0;
            active_len=12;
            return active_len;
        }
        
        else if(user_app_data[1]=='2' )//2012 7 24 ����2 ��ѯ��ǰ״̬
        {
            P1_6 = !P1_6;
            LED2_SBIT = P1_6;
            active_len=12;
            return active_len;
        }
        else if(user_app_data[1]=='3' )//2012 7 24 ����3 �л���ǰ״̬
        {
            IO_CTLA_SBIT = !IO_CTLA_SBIT;
            LED2_SBIT = IO_CTLA_SBIT;
            str[1]=1;//��1������
            str[2]=!IO_CTLA_SBIT;
            active_len=12;
            return active_len;;
        }
    }
    else if(user_app_data[0]=='2')
    {
	
        if(user_app_data[1]=='1' )//������
        {  
            IO_CTLA_SBIT =0;//�򿪼̵���
            P1_7 = 0;//led    
            
            str[1]=2;//��2������
            str[2]=1;
            active_len=12;
            return active_len;
        }
        
        else if(user_app_data[1]=='0' )//��
        {
            IO_CTLA_SBIT =1;//�رռ̵���
            P1_7 = 1;//led
            
            str[1]=2;//��2������
            str[2]=0;
            active_len=12;
            return active_len;
        }
        
        else if(user_app_data[1]=='2' )//2012 7 24 ����2 ��ѯ��ǰ״̬
        {
            str[1]=2;//��2������
            str[2]=!IO_CTLB_SBIT;
            active_len=12;
            return active_len;
        }
        else if(user_app_data[1]=='3' )//2012 7 24 ����3 �л���ǰ״̬
        {
            str[1]=2;//��2������
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
        
        P_Value = GetPactive(1.0, 10000, 0);//��ȡ�й�����
        P_Value_Now = P_Value*1.2;//*12000/10000�õ����յ���Ч�Ĺ���ֵ	
        I_Value_Now = GetCurrentIrms(1,10000,0);//��ȡ����ֵ
        
        if(user_app_data[1] == '1')//��ֵУ׼,0.1A������220V��ѹ
        {
            if(I_Value_Now > 1000)
            {
                I_Value_jiaozhun_Temp = I_Value_Now - 1000;//����У׼�Ĳ�ֵ
            }
            
            P_Value_Now *= P_Value_Bili_Temp;
            
            if(P_Value_Now > 22)
            {
                P_Value_jiaozhun_Temp = P_Value_Now - 22;//22W��У׼
            }
            
            str[2]=1;
        }
        else if(user_app_data[1] == '2')//����У׼��8A������220V��ѹ
        {
            if(I_Value_Now > 80000)//8A����
            {
                I_Value_Bili_Temp = 80000 / I_Value_Now;//����У׼�ı���ֵ
            }
            else
            {
                I_Value_Bili_Temp =  I_Value_Now / 80000;//����У׼�ı���ֵ
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
        else if(user_app_data[1] == '3')//ȷ��У׼ֵ
        {
            
            CS5463_STR[0] = 11;//�Ѿ�У׼�ı�־
            P_Value_Bili = (uint16)(P_Value_Bili_Temp * 1000);//����У׼ֵ
            CS5463_STR[3] = P_Value_Bili >> 8;
            CS5463_STR[4] = P_Value_Bili ;
            
            CS5463_STR[1] = P_Value_jiaozhun_Temp>>8;
            CS5463_STR[2] = P_Value_jiaozhun_Temp;
            
            CS5463_STR[5] = I_Value_jiaozhun_Temp>>8;
            CS5463_STR[6] = I_Value_jiaozhun_Temp;	
            
            I_Value_Bili = (uint16)(I_Value_Bili_Temp * 1000);
            CS5463_STR[7] = I_Value_Bili >> 8;
            CS5463_STR[8] = I_Value_Bili ;	
            
            osal_nv_item_init(CS5463_JIAOZHUN_NV_FLAG, 10, CS5463_STR);  //b���浽FLASH
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
* ��������: USER_SET_RS_MSG
* ��������: �û���������RS����
* ��ڲ���: ��
* ���ڲ���: ��
* ��ע˵��: ��������Э�������͵�������Ϣ���ݣ�������5S���Զ�����
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
* ��������: SampleApp_SendInMessage
* ��������: �û����й滮�����ϱ����ݵĸ�ʽ��
* ��ڲ���: ��
* ���ڲ���: ��
* ��ע˵��: ��
******************************************************************************/
void SampleApp_SendInMessage()
{
    uint8 *send_str;
    uint8 send_count=0;
    send_count = 10+8;
    send_str = osal_mem_alloc(send_count);//�������ݻ�����
    
    USER_SEND_ADD_STR(send_str);  //\��ӹ̶�֡ͷ
    
    send_str[9]='/';
    send_str[10]=9;
    send_str[11]=IO_CTLA_SBIT;//����1��ͨ��״̬
    send_str[12]=IO_CTLB_SBIT;//����2��ͨ��״̬
    send_str[13]=P_Value_Now/256;//��ǰ�Ĺ���ֵ
    send_str[14]=P_Value_Now%256;  
    send_str[15]=(uint8)P_Value_energy>>16;
    send_str[16]=(uint8)P_Value_energy>>8;
    send_str[17]=(uint8)P_Value_energy;
    
    
    USER_APP_SEND_IN(send_count,send_str);//���ͳ�����
    
    osal_mem_free( send_str );//�ͷ����ݻ�����
}

/*******************************************************************************
* ��������: User_APP_Drive_init
* ��������: USER�������Ӧ�ó�ʼ���Ĵ��룬���ʼ�������
* ��ڲ���: ��
* ���ڲ���: ��
* ��ע˵��: ��
******************************************************************************/
void User_APP_Drive_init(void)
{   
    HAL_IO_CONFIG_INIT();
    InitCS5463();//��ʼ��CS5463
    ConfigCS5463();//����CS5463
    P_JiaoZhun_Process();
    P1DIR |= 0Xc0;//P1.7 ---led3
    
    //ȷ���̵�����LED�����ڹر�״̬��
    P1_6 = 1;
    P1_7 = 1;
    P1_5 = 1;
    P0_1 = 1;
}


uint8 READ_NETKEY() //��ȡ����״̬������״̬���ݵ�Ӧ�ò�
{
    return HAL_PUSH_BUTTON3();
}



/*
mode 0 ��
1 ��
2 �л�
*/
uint8 LED_CHANGE(uint8 Position,uint8 Mode)// ����LED����
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
/*************************************�û�����********************************/
/*******************************************************************************
* ��������: CS5463_Send_PWER
* ��������: ÿ�����Ӷ�һ�ι������ݣ����ж��Ƿ���Ҫ�ϴ�
* ��ڲ���: ��
* ���ڲ���: ��
* ��ע˵��: ��
******************************************************************************/
uint8 CS5463_Send_PWER(void)
{
    I_Value_Now = GetCurrentIrms(1,10000,0);//��ȡ����ֵ
    if(I_Value_Now > 100)//0.01A
    {
        P_Value = GetPactive(1.0, 10000, 0);//��ȡ�й�����
        P_Value_Now = (uint32)(P_Value*1.2);//*12000/10000�õ����յ���Ч�Ĺ���ֵ
        P_Value_Now = (float)P_Value_Now * P_Value_Bili_Temp;	
        
        if(P_Value_Now > P_Value_jiaozhun_Temp)
        {
            P_Value_Now -= P_Value_jiaozhun_Temp ;//ȥ��У׼ֵ
            if((P_Value_Now >0)  &&  (P_Value_Now < 16))
            {
                P_Value_Now -= 1;
            }
        }
        else
        {
            P_Value_Now = 0;//С��У׼ֵ�����ж���ӦΪ0W
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
    
    Total_P_Value_Temp += P_Value_Now * 2;//2s�Ĺ���w.s  ��W.SΪ��λ
    if(Total_P_Value_Temp >= 3600)//�����ۼ�
    {
        Total_P_Value_Temp -= 3600;//3600W.S = 1 W.H
        P_Value_energy++;
        if(P_Value_energy >= 16000000)P_Value_energy = 16000000;
    }
    
    
    if(P_Value_Now > P_Protect_Value)//����500W�͹رռ̵���
    {       	
        SampleApp_SendInMessage();//�ϱ���ǰ����ֵ  
        return 1;
    }
    else if(P_Value_Now > P_Value_Pre + P_Value_Distance)//�������ϴλ㱨����������һ������ֵ���ϱ�
    {        
        P_Value_Pre = P_Value_Now;//������һ�η��͵���ֵ
        SampleApp_SendInMessage();
        return 1;
    }
    
    return 0;
}

/*******************************************************************************
* ��������: P_JiaoZhun_Process
* ��������: ����У׼ֵ��ʼ��
* ��ڲ���: ��
* ���ڲ���: ��
* ��ע˵��: ��
******************************************************************************/
void P_JiaoZhun_Process(void)
{	
    osal_nv_item_init(CS5463_JIAOZHUN_NV_FLAG, 10, CS5463_STR);  
    osal_nv_read(CS5463_JIAOZHUN_NV_FLAG, 0, 10,   CS5463_STR);
    
    if(CS5463_STR[0] == 11)//����Ѿ�������У׼ֵ����ֱ�Ӷ�ȡ�Ѿ������У׼ֵ
    {
        
        P_Value_Bili_Temp = (float)((uint16)CS5463_STR[3] << 8 | (uint16)CS5463_STR[4]) / 1000;//�洢��ʱ��*1000
        P_Value_jiaozhun_Temp = (uint16)CS5463_STR[1] << 8 | (uint16)CS5463_STR[2];
        I_Value_Bili_Temp = (float)((uint16)CS5463_STR[7] << 8 | (uint16)CS5463_STR[8]) / 1000;//�洢��ʱ��*1000
        I_Value_jiaozhun_Temp = (uint16)CS5463_STR[5] << 8 | (uint16)CS5463_STR[6];
        
    }
    else//���û�б���У׼ֵ����ֱ��ʹ��Ĭ��У׼ֵ
    {
        P_Value_Bili_Temp = 1.0;
        P_Value_jiaozhun_Temp = 0;	
        I_Value_Bili_Temp = 0;
        I_Value_jiaozhun_Temp = 0;
    }
}



/**********************************END OF FILE********************************/
/*****************************************************************************/












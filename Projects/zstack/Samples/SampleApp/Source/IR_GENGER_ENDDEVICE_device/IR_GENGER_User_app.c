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




#include "OSAL.h"
#include "ZGlobals.h"
#include "AF.h"
#include "aps_groups.h"
#include "ZDApp.h"
#include "device.h"
#include "OnBoard.h"
#include "MT_UART.h"
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
#include "irr.h"
#include "m25p16.h"
   #include "IO_config.h"


uint8 UserApp_TaskID;
uint8 RS_LEN=0;
uint8 RS_STR[10];
uint8 SHORT_ADRESS[4];
uint8 USER_DATA_LEN=0;
uint8 USER_DATA_TYPE=0;
uint8 *USER_DATA_STR;

uint8 CmdType;  //�������ͣ� '1' ѧϰ��'2' ʹ��
uint16 IRCodeIndex;   //
uint8 SAddr,PAddr,PNum;

uint8 DEVICE_ID;
uint8 DEFINED_CONTROL_CHAR;
uint8 JOIN_ROLE;
uint16 NORMAL_POLL_TIME=7000;


uint8 longPollRateFlag=0;

extern cId_t bindingINClusters[1];


uint8 USER_APP_COMMAND(uint8 *user_app_data,uint8 *str); 
void USER_APP_JOIN_MSG(void);
void SampleApp_ProcessBINDINGMessage(afIncomingMSGPacket_t *pkt);

void User_APP_Drive_init(void);
void SampleApp_HandleKeys( uint8 shift, uint8 keys );//���û�Ӧ�ð�������
void USER_SET_RS_MSG(void);//����RS����

void USER_AFTER_RS_MSG(void);

void USER_ZDO_JOINING_MSG(void); //ZDO��������ʱ������ ������˸��
uint8 READ_NETKEY(void); //��ȡ����״̬������״̬���ݵ�Ӧ�ò�
void LED_CHANGE(uint8 Position,uint8 Mode);// ����LED����


void changePollRate(void);

void UserApp_Init( uint8 task_id )
{

  UserApp_TaskID = task_id;
  
 
  RegisterForKeys(task_id);
  
  User_APP_Drive_init();

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
uint16 UserApp_ProcessEvent( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
 
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
  
  
 //�û���������¼�
 //test
     if ( events & USER_TEST_EVT )
  {
    HAL_TOGGLE_LED1();
      osal_start_timerEx( UserApp_TaskID, USER_TEST_EVT,10000);
    
     return (events ^ USER_TEST_EVT );
  } 
  
  
   if( events & RESUME_POLL_RATE )
  {
	  longPollRateFlag = 1;
	  //setDC6688sleep();
	  NLME_SetPollRate( 2000 );//set the poll rate to quick ,and   
	  return ( events ^ RESUME_POLL_RATE);
  }
  

  return 0;
}



// ��������䵽USER�㣬�û��������ɾ��
void SampleApp_HandleKeys( uint8 shift, uint8 keys )
{
  uint8 startOptions;
 // uint8 logicalType;
  zAddrType_t dstAddr;
  

   if ( keys &  HAL_KEY_SW_8 )
  {   
    
    
       if(key_double_flag==4)
    {
         APPLY_TO_JOIN_OF_KEY();
    }
    
   
   else if(key_double_flag==6)
    {
        SampleApp_PermitJoin(240);
    }

    else if(key_double_flag==8)
    {
        SampleApp_PermitJoin(0);
    }
    
   

     if(key_hold_flag==1) 
    {   
      HalLedBlink ( HAL_LED_1, 4, 50, 1000 );
      RESTORE_TO_FACTORY();
     osal_start_timerEx( SampleApp_TaskID,SAMPLEAPP_RESTORE_EVT,5000 );
     }
  
  }
 
}



void SampleApp_ProcessBINDINGMessage(afIncomingMSGPacket_t *pkt)
{

  
}


// ��������Ӻ���
void USER_APP_JOIN_MSG(void)
{
    HalLedBlink ( HAL_LED_1, 1, 99, 2500 );//��BLINKģ����������Ϩ��
    NLME_SetPollRate( 1000 );//set the poll rate to quick ,and   

    uint8 *Sample_Cmd_shorAddr;
    Sample_Cmd_shorAddr=SampleApp_GetShortAddr();
    SHORT_ADRESS[0]=*Sample_Cmd_shorAddr++;
    SHORT_ADRESS[1]=*Sample_Cmd_shorAddr++;
    SHORT_ADRESS[2]=*Sample_Cmd_shorAddr++;
    SHORT_ADRESS[3]=*Sample_Cmd_shorAddr;
    osal_mem_free( shortddr_mem );
  // USER�������������Ĳ�������
 
}


// �յ��������Ӻ���
// user_app_data ��λ������������Ч���� ��ĸ���濪ʼ 13��ʼ
//user_rf_data ��Ҫ����ķ�����������
uint8 USER_APP_COMMAND(uint8 *user_app_data,uint8 *str)
{
  uint8 active_len=0;

    str[0] = '/';
   
      CmdType = user_app_data[0];//�����豸�������� ѧϰ ʹ��
      IRCodeIndex = 	(user_app_data[1]- '0')*100 +
					(user_app_data[2]- '0')*10 +
					(user_app_data[3]- '0');      //�������

    if('1' == CmdType)		//'1' ��ʾѧϰ
    {
      irCodeRx(IRCodeIndex);
      str[1] = CmdType - '0';//���ݻ���
      str[3] = IRCodeIndex;//���ݻ���
      str[2] = IRCodeIndex >> 8;
      active_len=13;
      changePollRate();
      //HAL_TURN_OFF_LED2();//off led
      return active_len;
    }
    else if('2' == CmdType)	//'2' ��ʾʹ��
    {
    //  HAL_TURN_ON_LED2();//on led
      
     HalLedBlink ( HAL_LED_2, 1, 30, 200 );
    
     irCodeTx(IRCodeIndex);
  
     str[1] = CmdType - '0';//���ݻ���
      str[3] = IRCodeIndex;//���ݻ���
      str[2] = IRCodeIndex >> 8;
         changePollRate();      
      active_len=13;
      return active_len;
    }
    else 	
	{
           return 0;
	}
     

}


void USER_SET_RS_MSG(void)//�û���������RS����
{
    DEVICE_ID=22;
    DEFINED_CONTROL_CHAR='T';
    JOIN_ROLE=Send_Endvice;
    RS_LEN = 1;
    RS_STR[0]=0;
}


/*
 �û����й滮�����ϱ����ݵĸ�ʽ��
*/
void SampleApp_SendInMessage(void)
{
 
  uint8 *send_str;
  uint8 send_count=0;
  send_count = 10+2;
  send_str = osal_mem_alloc(send_count);
  
  USER_SEND_ADD_STR(send_str);  //\��ӹ̶�֡ͷ

  send_str[9]=0xA1;
  
   if(BUTTERY_LOW_FLAG==1)
   {  
    send_str[10]=1;//1���� ��ظ澯���λ
    send_str[11]=1; //���� ��ظ澯�¼�
    BUTTERY_LOW_FLAG = 0;
   }
   else
   send_str[11]=0;

  USER_APP_SEND_IN(send_count,send_str);//���ͳ�����
  
  osal_mem_free( send_str );
}

void User_APP_Drive_init(void)
{
  HAL_IO_CONFIG_INIT();
     
        irrInit();
  
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
void LED_CHANGE(uint8 Position,uint8 Mode)// ����LED����
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
        
          case HAL_LED_4:      
        {
         if(Mode==0 || Mode==1)
         {
          LED4_SBIT = LED4_POLARITY(Mode);
         }
         else if(Mode==2)
         {
          if (LED4_SBIT) 
          { 
            LED4_SBIT = 0; 
          } 
          else 
          { 
            LED4_SBIT = 1;
          } 
         }
        }   
        break;
        
          case HAL_LED_5:      
        {
         if(Mode==0 || Mode==1)
         {
          LED5_SBIT = LED5_POLARITY(Mode);
         }
         else if(Mode==2)
         {
          if (LED5_SBIT) 
          { 
            LED5_SBIT = 0; 
          } 
          else 
          { 
            LED5_SBIT = 1;
          } 
         }
        }   
        break;

    default:
      break;
    }
}


void USER_ZDO_JOINING_MSG(void)
{
 HalLedBlink ( HAL_LED_1, 1, 30, 500 );
}
//RS�����Ƿ�Ҫ��������¼�����Ӧ�ã��ڴ˽��д���
void USER_AFTER_RS_MSG(void)
{


}


//��������1����һ�Σ�������������ؼ�⣬�����Ҫ�Ļ�
void USER_MAYBE_BUTTERY_MSG(void)
{
      //���ӵ�ؼ��Ĺ���
    uint16 active_battery_voltage=0;
    active_battery_voltage=myApp_ReadBattery();

    if(active_battery_voltage<=25)
    {
     if( (devState==DEV_END_DEVICE) || (devState==DEV_ROUTER) )
     {
       BUTTERY_LOW_FLAG =1;
       SampleApp_SendInMessage();
       
     }
    }
 
}

/*******************************************************************************
�������ܣ��ı�poll��ʱ������������Ϊ���������ĳɶ̼����������60s������
		  ���ָ��¼�
*/
void changePollRate(void)
{
	//osal_stop_timerEx( SampleApp_TaskID, RESUME_POLL_RATE);
	if(longPollRateFlag)
	{
		//�ı�pollrate��100ms
		longPollRateFlag = 0;
		NLME_SetPollRate( 300 );//set the poll rate to quick ,and   
	}
	osal_start_timerEx( UserApp_TaskID, RESUME_POLL_RATE,20000);
}



// �����ɹ��󣬽�������ģʽ�������豸��POLL RATEʱ��
void USER_SET_NOMAL_POLL_TIME(void)
{
 NORMAL_POLL_TIME=2000;
}
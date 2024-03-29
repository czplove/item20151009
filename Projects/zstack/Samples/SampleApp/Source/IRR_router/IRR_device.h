/**************************************************************************************************
  Filename:       SampleApp.h
  Revised:        $Date: 2007-10-27 17:16:54 -0700 (Sat, 27 Oct 2007) $
  Revision:       $Revision: 15793 $

  Description:    This file contains the Sample Application definitions.


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

#ifndef IRR_DEVICE_H
#define IRR_DEVICE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"

/***********************************************
Application head files
*/
 
  
#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"


/***********************************************
Application head files
*/
 
#define IRR_ID  22

typedef struct
{
  
  uint8 isActive;
  uint8 heart_count;
} Child_device_t;    
  
#define APP_INIT                           0    // Initial state
#define APP_START                          1    // Sensor has joined network  
  
/*********************************************************************
 * CONSTANTS
 */

// These constants are only for example and should be changed to the
// device's needs
#define SAMPLEAPP_ENDPOINT           20

#define SAMPLEAPP_PROFID             0x0F08
#define SAMPLEAPP_DEVICEID           0x0001
#define SAMPLEAPP_DEVICE_VERSION     0
#define SAMPLEAPP_FLAGS              0

#define SAMPLEAPP_MAX_CLUSTERS        9
#define SAMPLEAPP_ONOFF_CLUSTERID   1
#define SAMPLEAPP_FLASH_CLUSTERID     2
#define SAMPLEAPP_ADDR_CLUSTERID      3
#define SAMPLEAPP_COMMAND_CLUSTERID   4
#define SAMPLEAPP_PERIODIC_CLUSTERID  5  //心跳信息
#define SAMPLEAPP_ANNCE_REQ_CLUSTERID 6  //ANNCE REQ
#define SAMPLEAPP_PERIODIC_COORDINATOR_CLUSTERID  7  //心跳信息给协调器
#define SAMPLEAPP_ACTIVE_COMMAND_CLUSTERID       8//主动上报命令
#define SAMPLEAPP_IR_CLUSTERID  12
// Send Message Timeout
#define SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT   60000     // Every 30 seconds

// Application Events (OSAL) - These are bit weighted definitions.
#define SAMPLEAPP_SEND_PERIODIC_MSG_EVT       0x0001
#define ASSOCIA_MENTAIN_EVT                   0x0002
#define SAMPLEAPP_SEND_NWKADDR_EVT            0x0004
#define OS_TEST_EVT			      0x0008 //20110721
#define SAMPLEAPP_RESTORE_EVT                 0x0010
#define NEW_JOIN_ROLE_EVT                     0x0020
#define SAMPLEAPP_SEND_RS_EVT                 0x4000

#define RESUME_POLL_RATE                      0x0040




// Group ID for Flash Command
//#ifndef ZDO_COORDINATOR
#define SAMPLEAPP_GROUP1                       0x0001
#define SAMPLEAPP_GROUP2                       0x0002  
#define SAMPLEAPP_GROUP3                       0x0003
//#define COMMUNICATION_GROUP9                   0x0009  
//#endif
  
// Flash Command Duration - in milliseconds
#define SAMPLEAPP_FLASH_DURATION               1000

#define BatteryVoltageMin_Threshold  26

#define ZB_SUCCESS                        ZSuccess
#define ZB_FAILURE                        ZFailure
#define ZB_INVALID_PARAMETER              ZInvalidParameter

#define SAPICB_START_CNF  0xE2  
  
#define Send_Router 0
#define Send_Endvice 1
  
//#define COMMUNICATION
/*********************************************************************
 * MACROS
 */


/*********************************************************************
 * FUNCTIONS
 */


/*
 * peripheral Initialization for the Generic Application,
 * 在此函数里作出了系统必须的初始化以外的初始化，如二氧化碳外设，红外外设等
 * 的初始化，根据不同的应用，作相应的初始化。
 */
extern void Apps_Init(void);


/*
 * Task Initialization for the Generic Application
 */
extern void SampleApp_Init( uint8 task_id );

/*
 * Task Event Processor for the Generic Application
 */
extern UINT16 SampleApp_ProcessEvent( uint8 task_id, uint16 events );

extern uint8 zb_ReadConfiguration( uint8 configId, uint8 len, void *pValue );
extern uint8 zb_WriteConfiguration( uint8 configId, uint8 len, void *pValue );
extern void zb_StartRequest ( void );

extern void SampleApp_SendPeriodicMessage(void);
extern void Delay_device_1u(uint32 microSecs);



extern bool returnState;


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SAMPLEAPP_H */

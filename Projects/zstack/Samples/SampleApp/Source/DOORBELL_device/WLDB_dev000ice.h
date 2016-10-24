
/******************** (C) COPYRIGHT 2011 Jami***********************************
* File Name		: WLDB_device.h
* Author		: JamiLiang At Gmail.com
* Date			: 2014/01/14
* Description	: This file provides all the xxx Module functions.
* Version		: V0.1
* ChangeLog	:
* Version		Name			Date			Description
  0.1			JamiLiang		2014/01/14		Initial Version

*******************************************************************************/
#ifndef _WLDB_DEVICE_H_
#define _WLDB_DEVICE_H_

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "ZComDef.h"

#define DEVICE_TYPE_ID			176
typedef struct
{
	uint8 isActive;
	uint8 heart_count;
} Child_device_t;

#define APP_INIT							0	// Initial state
#define APP_START							1	// Sensor has joined network

// CONSTANTS
// These constants are only for example and should be changed to the
// device's needs
#define SAMPLEAPP_ENDPOINT							20

#define SAMPLEAPP_PROFID							0x0F08

#define SAMPLEAPP_DEVICEID							0x0001
#define SAMPLEAPP_DEVICE_VERSION					0
#define SAMPLEAPP_FLAGS 							0

#define SAMPLEAPP_MAX_CLUSTERS						8
#define SAMPLEAPP_ONOFF_CLUSTERID					1
#define SAMPLEAPP_FLASH_CLUSTERID					2
#define SAMPLEAPP_ADDR_CLUSTERID					3
#define SAMPLEAPP_COMMAND_CLUSTERID					4
#define SAMPLEAPP_PERIODIC_CLUSTERID				5
#define SAMPLEAPP_ANNCE_REQ_CLUSTERID				6
#define SAMPLEAPP_PERIODIC_COORDINATOR_CLUSTERID	7
#define SAMPLEAPP_ACTIVE_COMMAND_CLUSTERID			8
// Send Message Timeout
#define SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT			60000 	// Every 30 seconds

// Application Events (OSAL) - These are bit weighted definitions.
#define SAMPLEAPP_SEND_PERIODIC_MSG_EVT 			0x0001
#define ASSOCIA_MENTAIN_EVT 						0x0002
#define SAMPLEAPP_SEND_NWKADDR_EVT					0x0004
#define OS_TEST_EVT									0x0008
#define SAMPLEAPP_RESTORE_EVT						0x0010
#define NEW_JOIN_ROLE_EVT							0x0020
#define WLDB_CLRIO_EVENT							0x0040
#define WLDB_CLRDB_EVENT							0x0080
//#define WLDB_CLRDBOP_EVENT							0x0100
// Group ID for Flash Command
//#ifndef ZDO_COORDINATOR
#define SAMPLEAPP_GROUP1							0x0001
#define SAMPLEAPP_GROUP2							0x0002
#define SAMPLEAPP_GROUP3							0x0003
//#define COMMUNICATION_GROUP9						0x0009
//#endif

// Flash Command Duration - in milliseconds
#define SAMPLEAPP_FLASH_DURATION					1000

#define ZB_SUCCESS									ZSuccess
#define ZB_FAILURE									ZFailure
#define ZB_INVALID_PARAMETER						ZInvalidParameter

#define SAPICB_START_CNF							0xE2

#define Send_Router				0
#define Send_Endvice			1

//#define COMMUNICATION
extern void Apps_Init(void);
extern uint8 SampleApp_TaskID;

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
extern bool returnState;


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /*_WLDB_DEVICE_H_*/


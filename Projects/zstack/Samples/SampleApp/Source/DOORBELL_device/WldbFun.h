
/******************** (C) COPYRIGHT 2011 Jami***********************************
* File Name		: SrmoFun.h
* Author		: JamiLiang At Gmail.com
* Date			: 2012/06/28
* Description	: This file provides all the xxx Module functions.
* Version		: V0.1
* ChangeLog	:
* Version		Name       		Date			Description
  0.1			JamiLiang		2012/06/28		Initial Version

*******************************************************************************/
#ifndef _SRMOFUN_H_
#define _SRMOFUN_H_

/* Includes ------------------------------------------------------------------*/
//#include "Commdef.h"
#include "OSAL_Nv.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#ifdef USE_TI_ZIGBEE_FRAMWORK
//
#define IO_CTLA					P1_3
#define IO_CTLB					P1_6
#define IO_CTLC					P1_4
#define IO_CTLD					P2_0

#define CTLA_H					IO_CTLA=1;
#define CTLA_L					IO_CTLA=0;
#define CTLB_H					IO_CTLB=1;
#define CTLB_L					IO_CTLB=0;
#define CTLC_H					IO_CTLC=1;
#define CTLC_L					IO_CTLC=0;
#define CTLD_H					IO_CTLD=1;
#define CTLD_L					IO_CTLD=0;
#else

#endif



/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* External variables --------------------------------------------------------*/
/* External functions --------------------------------------------------------*/

/* Public typedef ------------------------------------------------------------*/
/* Public define -------------------------------------------------------------*/
/* Public macro --------------------------------------------------------------*/
/* Public variables ----------------------------------------------------------*/
/* Public function prototypes ------------------------------------------------*/

/* Public functions ----------------------------------------------------------*/

int NWK_interface(uint8 *cmdBuf, uint8 len);

void initWldb(void);

#endif /*_SRMOFUN_H_*/



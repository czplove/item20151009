/***************************************************************************************************
  Filename:       CO2_UART.h
  Revised:        $Date: 2009-07-02 15:24:39 -0700 (Thu, 02 Jul 2009) $
  Revision:       $Revision: 20269 $

  Description:    This header describes the functions that handle the serial port.

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

***************************************************************************************************/
#ifndef BGM_UART_H
#define BGM_UART_H

#ifdef __cplusplus
extern "C"
{
#endif

/***************************************************************************************************
 *                                               INCLUDES
 ***************************************************************************************************/
//#include "Onboard.h"
//#include "OSAL.h"


#include "hal_mcu.h"
//#include "hal_board_cfg.h"
//#include "ZComDef.h"
//#include "OSAL_Nv.h"

	
	
	
/***************************************************************************************************
 *                                             CONSTANTS
 ***************************************************************************************************/

#if defined(BGM_ROUTER)

#define NAME_TYPE   0x01
#define STATE_TYPE  0x02

  
extern uint8 paraGetFlag;
extern uint8 uart0RecDataType;
extern uint8 BGM_nameUTF8[100];
extern uint8 nameUTF8Length;
extern uint8 index4nameUTF8_L;
extern uint8 index4nameUTF8_H;

extern uint8 RT_nwkLocState;//bb
extern uint8 RT_voiceValue;//cc
extern uint8 RT_playState;
extern uint8 RT_indexSong_H;
extern uint8 RT_indexSong_L;
extern uint8 RT_totalSong_H;
extern uint8 RT_totalSong_L;//ii
extern uint8 RT_cycleMode;
extern uint8 RT_playMode;
extern uint8 RT_download;   
extern uint16 RT_totalSong;   
extern uint16 RT_totalSongPre;   
extern uint8 RT_listUpdateFlag;

/*****************************************
获取当前播放器的状态
*****************************************/
extern uint8 serial_sxbuf_state[8];   

/*****************************************
获取当前播放歌曲的名字
*****************************************/
extern uint8 serial_sxbuf_NAME[9];

/*****************************************
播放控制命令
*****************************************/
extern uint8 serial_sxbuf_play[8]; //play
extern uint8 serial_sxbuf_stop[8]; //stop
extern uint8 serial_sxbuf_break[8]; //break

/*****************************************
本地模式
*****************************************/
extern uint8 serial_sxbuf_BNED_STOP[8];//BEND STOP
extern uint8 serial_sxbuf_BNED_PLAY[8];//BEND PLAY

/*****************************************
切歌命令
*****************************************/
extern uint8 serial_sxbuf_LAST[10];//切歌命令 上一首
extern uint8 serial_sxbuf_NEXT[10];//切歌命令 下一首
//uint8 serial_sxbuf_INDEX[9]={0xFF,0x55,0x05,0x80,0x06,0x00,0x00,0x00,0xFF,0xAA};//切歌命令 按索引 YY YY为索引 默认为第一首

/*****************************************
歌曲循环状态控制，该命令只在本地模式下使用
*****************************************/
extern uint8 serial_sxbuf_CYCLE_LB[8]; //列表循环
extern uint8 serial_sxbuf_CYCLE_DQ[8];//单曲循环
extern uint8 serial_sxbuf_CYCLE_SJ[8];//随机循环

/*****************************************
切换网络模式命令
*****************************************/
extern uint8 serial_sxbuf_net_change[8];//网络切换命令
extern uint8 serial_sxbuf_net_AP[8];//AP模式
extern uint8 serial_sxbuf_net_STA[8];//STA模式
extern uint8 serial_sxbuf_net_wps[8];//WPS模式

/*****************************************
保存歌曲命令
*****************************************/
extern uint8 serial_sxbuf_HOLD[8];//保存歌曲
/*****************************************/

extern uint8 serial_sxbuf_on[8];//打开主动返回
extern uint8 serial_sxbuf_off[8];//关闭主动返回

extern uint8 serial_sxbuf_WIFI_IDPW[250];//WIFI SSID password



#endif

	
	
/*
 * Initialization
 */

extern uint16 BGM_UARTRx_datCnt;
extern void hal_USART0Init(void);
extern void hal_USART1Init(void);

extern void hal_USART0Write ( uint8 *datBuf, uint8 DatLen);
extern void hal_USART0Read ( uint8 *datBuf);
extern void hal_USART1Write ( uint8 *datBuf, uint8 DatLen);
extern void hal_USART1Read ( uint8 *datBuf);


extern void hal_USART0Act(void);
extern void hal_USART1Act(void);
extern void hal_USART0Sleep(void);
extern void hal_USART1Sleep(void);
extern void BPM_UART0_ISR_CB(void);

extern void control_volume(uint8 a);
extern void get_list(uint8 index_begin,uint8 index_begin_1);
extern void cut_song_index(uint8 song_L , uint8 song_H);


extern void BGM_uartRXFrameAnalysis(void);

/***************************************************************************************************
***************************************************************************************************/

#endif  /* CO2_UART_H */

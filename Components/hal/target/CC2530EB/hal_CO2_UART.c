/***************************************************************************************************
  Filename:       MT_UART.c
  Revised:        $Date: 2009-03-12 16:25:22 -0700 (Thu, 12 Mar 2009) $
  Revision:       $Revision: 19404 $

  Description:  This module handles anything dealing with the serial port.

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

/***************************************************************************************************
 * INCLUDES
 ***************************************************************************************************/
#include "ZComDef.h"
#include "OSAL.h"
#include "hal_CO2_UART.h"
#include "OSAL_Memory.h"
#include "device.h"
#include "ZDApp.h"
//#include "OLD_CO2_device.h"




uint8 CO2_UART0_DatBuf_Tem[200];
uint8 CO2_UART0_DatBuf[10];

volatile uint8 CO2_UARTRx_datCnt = 0;


/***************************************************************************************************
 *                                         GLOBAL VARIABLES
 ***************************************************************************************************/
/* Used to indentify the application ID for osal task */


void hal_USART0Init(void)
 {
	//P0INP |= 0x04; //P0.2 peripherel
	//P0INP |= 0x08; //P0.3 peripherel
	//P0SEL |= 0x10; //P0.4 peripherel
	//P0SEL |= 0x20; //P0.5 peripherel
	P0SEL |= 0x04; //P0.2 peripherel
	P0SEL |= 0x08; //P0.3 peripherel
	//P0DIR |= 0x08; //P0.3 output
	
	//PERCFG |= 0x01;	
		
	U0BAUD = 0x3B;
	U0UCR = 0x02;
	U0GCR = 0x08;
	U0CSR = 0x82;	//0xC2;
	IEN0 |= 0x04;
	//URX1IE = 1;
	//IEN2 |= 0x08;//UTX0IE = 1;
	EA =1;
	U0CSR |= 0x40;	//0xC2;
	U0CSR &= ~(0x04);//清楚标志位
 }

void hal_UARTWrite ( uint8 *datBuf, uint8 len)
{
	uint8 datCnt;
	//EA = 0;
	for(datCnt = 0; datCnt < len; datCnt++)
	{
		while(U0CSR & 0x01);
		U0DBUF = *datBuf;
		while(0 == (U0CSR &(1<<1)));	//直到发送完成
		U0CSR |= (0x02);				//清楚标志位
		datBuf++;
	}
	//EA = 1;
}


void hal_UARTRead ( uint8 *datBuf)
{
	uint8 Cnt = 0;
	while(1)
	{
		while(!(U1CSR & 0x04));				//清楚标志位
		datBuf[Cnt] = U1DBUF;
		if(0x0A == datBuf[Cnt])// && (0x0D == UARTDatBuf[datCnt - 1]))
		{
			if(0x0D == datBuf[Cnt - 1])
			{
				//receviced
				osal_set_event(SampleApp_TaskID,UART_RX_CMD_CB_EVT);
				return;
			}
		}
		Cnt++;
	}
}




HAL_ISR_FUNCTION( halUSART0RxIsr, URX0_VECTOR )
{
	//HAL_ENTER_ISR();
	//EA = 0;
	TCON &= ~(0x08);//08
	U0CSR &= ~(1<<2);				//清楚标志位
	CO2_UART0_DatBuf_Tem[CO2_UARTRx_datCnt] = U0DBUF;
	if(0x0A == CO2_UART0_DatBuf_Tem[CO2_UARTRx_datCnt])// && (0x0D == UARTDatBuf[datCnt - 1]))
	{
		if(0x0D == CO2_UART0_DatBuf_Tem[CO2_UARTRx_datCnt - 1])
		{
			//receviced
			for(uint8 i = 0; i<10;i++)
				CO2_UART0_DatBuf[i] = CO2_UART0_DatBuf_Tem[CO2_UARTRx_datCnt - 9 + i];
			osal_set_event(SampleApp_TaskID,UART_RX_CMD_CB_EVT);
			CO2_UARTRx_datCnt = 0;
			return;
		}
	}
	
	/*if(0x0A == CO2_UART0_DatBuf[CO2_UARTRx_datCnt])// && (0x0D == UARTDatBuf[datCnt - 1]))
	{
		if(0x0D == CO2_UART0_DatBuf[CO2_UARTRx_datCnt - 1])
		{
			//receviced
			osal_set_event(SampleApp_TaskID,UART_RX_CMD_CB_EVT);
			CO2_UARTRx_datCnt = 0;
			return;
		}
	}*/
	
  	CO2_UARTRx_datCnt++;
	//HAL_EXIT_ISR();
	//EA = 1;
}

HAL_ISR_FUNCTION( halUSART1TxIsr, UTX1_VECTOR )
{
	HAL_ENTER_ISR();
	IRCON2 &= ~(0x04);
	HAL_EXIT_ISR();
}



/***************************************************************************************************
***************************************************************************************************/

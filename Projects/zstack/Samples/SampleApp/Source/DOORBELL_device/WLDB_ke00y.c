
/******************** (C) COPYRIGHT 2011 Jami***********************************
* File Name		: Wldb_KEY.c
* Author		: JamiLiang At Gmail.com
* Date			: 2014/01/14
* Description	: This file provides all the xxx Module functions.
* Version		: V0.1
* ChangeLog	:
* Version		Name       		Date			Description
  0.1			JamiLiang		2014/01/14		Initial Version

*******************************************************************************/


/* Includes ------------------------------------------------------------------*/
#include "hal_mcu.h"
#include "hal_defs.h"
#include "hal_types.h"
#include "hal_board.h"
#include "hal_drivers.h"
#include "hal_adc.h"
#include "hal_key.h"
#include "hal_led.h"
#include "osal.h"
#include "device.h"
#include "ZDApp.h"
#include "WldbFun.h"

#if (defined HAL_KEY) && (HAL_KEY == TRUE)

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define CBIT0			0x01
#define CBIT1			0x02
#define CBIT2			0x04
#define CBIT3			0x08
#define CBIT4			0x10
#define CBIT5			0x20
#define CBIT6			0x40
#define CBIT7			0x80

#define HAL_KEY_DEBOUNCE_VALUE			50
#define HAL_KEY_CLICKED_VALUE			600		//Keep checking Keys Interrput in 600ms,
//and/then send the click times to AppLayer.
#define HAL_KEY_HOLD_VALUE				9400	//9.4s == 10s-0.6s
#define HAL_KEY_DOBULETICK_VALUE		1000


#define HAL_KEY_RISING_EDGE				0	//by default!
#define HAL_KEY_FALLING_EDGE			1

#define HAL_PORT0_EDGE_BIT				CBIT0	//P0.0-p0.7
#define HAL_PORT1L_EDGE_BIT				CBIT1	//P1.0-p1.3
#define HAL_PORT1H_EDGE_BIT				CBIT2	//P1.4-p1.7
#define HAL_PORT2_EDGE_BIT				CBIT3	//P2.0-p2.4

#define HAL_PORT0_IEN					IEN1//cbit5
#define HAL_PORT0_IEN_BIT				CBIT5
#define HAL_PORT1_IEN					IEN2//cbit4
#define HAL_PORT1_IEN_BIT				CBIT4
#define HAL_PORT2_IEN					IEN2//cbit1
#define HAL_PORT2_IEN_BIT				CBIT1

/* CPU port interrupt */
#define HAL_KEY_CPU_PORT_0_IF			P0IF
#define HAL_KEY_CPU_PORT_1_IF			P1IF
#define HAL_KEY_CPU_PORT_2_IF			P2IF





// IO Interrupt Mode configration
// Button Symbol defines
// BYN SEL is at P1.3
#define HAL_EXIT_SW1_PORT				P1
#define HAL_EXIT_SW1_BIT				CBIT3
#define HAL_EXIT_SW1_SEL				P1SEL
#define HAL_EXIT_SW1_DIR				P1DIR
// edge interrupt
#define HAL_EXIT_SW1_EDGEBIT			HAL_PORT1L_EDGE_BIT
#define HAL_EXIT_SW1_EDGE				HAL_KEY_FALLING_EDGE
// PB_1 interrupts
#define HAL_EXIT_SW1_IEN				HAL_PORT1_IEN
#define HAL_EXIT_SW1_IENBIT				HAL_PORT1_IEN_BIT
#define HAL_EXIT_SW1_ICTL				P1IEN
#define HAL_EXIT_SW1_ICTLBIT			CBIT3
#define HAL_EXIT_SW1_PXIFG				P1IFG


// BTN VOL is at P1.5
#define HAL_EXIT_SW2_PORT				P1
#define HAL_EXIT_SW2_BIT				CBIT5
#define HAL_EXIT_SW2_SEL				P1SEL
#define HAL_EXIT_SW2_DIR				P1DIR
// edge interrupt
#define HAL_EXIT_SW2_EDGEBIT			HAL_PORT1H_EDGE_BIT
#define HAL_EXIT_SW2_EDGE				HAL_KEY_FALLING_EDGE
// PB_1 interrupts
#define HAL_EXIT_SW2_IEN				HAL_PORT1_IEN
#define HAL_EXIT_SW2_IENBIT				HAL_PORT1_IEN_BIT
#define HAL_EXIT_SW2_ICTL				P1IEN
#define HAL_EXIT_SW2_ICTLBIT			CBIT5
#define HAL_EXIT_SW2_PXIFG				P1IFG


// BTN_PLAY is at P1.2
#define HAL_EXIT_SW5_PORT				P1
#define HAL_EXIT_SW5_BIT				CBIT2
#define HAL_EXIT_SW5_SEL				P1SEL
#define HAL_EXIT_SW5_DIR				P1DIR
// edge interrupt
#define HAL_EXIT_SW5_EDGEBIT			HAL_PORT1L_EDGE_BIT
#define HAL_EXIT_SW5_EDGE				HAL_KEY_FALLING_EDGE
// PB_1 interrupts
#define HAL_EXIT_SW5_IEN				HAL_PORT1_IEN
#define HAL_EXIT_SW5_IENBIT				HAL_PORT1_IEN_BIT
#define HAL_EXIT_SW5_ICTL				P1IEN
#define HAL_EXIT_SW5_ICTLBIT			CBIT2
#define HAL_EXIT_SW5_PXIFG				P1IFG


#define HAL_KEY_JOY_CHN					HAL_ADC_CHANNEL_6

#define WLDB_BTN_PLAY		(uint8)1
#define WLDB_BTN_SELE		(uint8)2
#define WLDB_BTN_VOLU		(uint8)3
#define WLDB_BTN_BUTTONEN	(uint8)4
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint8 halKeySavedKeys;	// used to store previous key state in polling mode
static uint8 HalKeyConfigured;
static halKeyCBack_t pHalKeyProcessFunction;
bool Hal_KeyIntEnable;

uint8 key_ISR_FLAG=0;
uint8 key_NWK_ISR_FLAG=0;	//Flag : single or double key click.
uint8 key_double_flag = 0;	//Flag : double key click.
uint8 key_holdtime_waiting_flag = 0;	//Flag : 10s waitting.0 nope,1 waitting.
uint8 key_hold_flag = 0;
//uint8 key_TGdouble_flag=0;	//Flag : Key pressed while Dimming.0 no, 1 yes.
uint8 ON_DIMMER_FLAG = 0;	//Flag : Dimming.0 no, 1 yes.
uint8 ButtonEn=1;
/* Private function prototypes -----------------------------------------------*/
void halProcessKeyInterrupt(void);
uint8 halGetJoyKeyInput(void);
void ButtonSW(uint8 btn);
void swIo(uint8 tBtn,uint8 tBtm);
/* Private functions ---------------------------------------------------------*/

/* External variables --------------------------------------------------------*/
/* External functions --------------------------------------------------------*/

/**************************************************************************************************
 * @fn      HalKeyInit
 *
 * @brief   Initilize Key Service
 *
 * @param   none
 *
 * @return  None
 **************************************************************************************************/
void HalKeyInit( void )
{
	/* Initialize previous key to 0 */
	halKeySavedKeys = 0;

	HAL_EXIT_SW1_SEL &= ~(HAL_EXIT_SW1_BIT);    /* Set pin function to GPIO */
	HAL_EXIT_SW1_DIR &= ~(HAL_EXIT_SW1_BIT);    /* Set pin direction to Input */

	HAL_EXIT_SW2_SEL &= ~(HAL_EXIT_SW2_BIT);
	HAL_EXIT_SW2_DIR &= ~(HAL_EXIT_SW2_BIT);

	HAL_EXIT_SW5_SEL &= ~(HAL_EXIT_SW5_BIT);
	HAL_EXIT_SW5_DIR &= ~(HAL_EXIT_SW5_BIT);
	
#ifdef _SYS_POWER_ON_CHK
	HAL_KEY_JOY_MOVE_SEL &= ~(HAL_KEY_JOY_MOVE_BIT);
	HAL_KEY_JOY_MOVE_DIR &= ~(HAL_KEY_JOY_MOVE_BIT);
#endif
	/* Initialize callback function */
	pHalKeyProcessFunction  = NULL;

	/* Start with key is not configured */
	HalKeyConfigured = FALSE;
}


/**************************************************************************************************
 * @fn      HalKeyConfig
 *
 * @brief   Configure the Key serivce
 *
 * @param   interruptEnable - TRUE/FALSE, enable/disable interrupt
 *          cback - pointer to the CallBack function
 *
 * @return  None
 **************************************************************************************************/
void HalKeyConfig (bool interruptEnable, halKeyCBack_t cback)
{
	/* Enable/Disable Interrupt or */
	Hal_KeyIntEnable = interruptEnable;

	/* Register the callback fucntion */
	pHalKeyProcessFunction = cback;

	/* Determine if interrupt is enable or not */
	if (Hal_KeyIntEnable)
	{
		/* Rising/Falling edge configuratinn */
		PICTL &= ~(HAL_EXIT_SW1_EDGEBIT);    //RESET_CBITS(PICTL,HAL_EXIT_SW1_EDGEBIT);
		/* For falling edge, the bit must be set. */
#if (HAL_EXIT_SW1_EDGE == HAL_KEY_FALLING_EDGE)
		PICTL |= HAL_EXIT_SW1_EDGEBIT;		//SET_CBITS(PICTL,HAL_EXIT_SW1_EDGEBIT);
#endif
		/* Interrupt configuration:
		 * - Enable interrupt generation at the port
		 * - Enable CPU interrupt
		 * - Clear any pending interrupt
		 */
		HAL_EXIT_SW1_ICTL |= HAL_EXIT_SW1_ICTLBIT;
		HAL_EXIT_SW1_IEN |= HAL_EXIT_SW1_IENBIT;
		HAL_EXIT_SW1_PXIFG &= ~(HAL_EXIT_SW1_BIT);


		/* Rising/Falling edge configuratinn */
		PICTL &= ~(HAL_EXIT_SW2_EDGEBIT);    //RESET_CBITS(PICTL,HAL_EXIT_SW1_EDGEBIT);
		/* For falling edge, the bit must be set. */
#if (HAL_EXIT_SW2_EDGE == HAL_KEY_FALLING_EDGE)
		PICTL |= HAL_EXIT_SW2_EDGEBIT;		//SET_CBITS(PICTL,HAL_EXIT_SW1_EDGEBIT);
#endif
		/* Interrupt configuration:
		 * - Enable interrupt generation at the port
		 * - Enable CPU interrupt
		 * - Clear any pending interrupt
		 */
		HAL_EXIT_SW2_ICTL |= HAL_EXIT_SW2_ICTLBIT;
		HAL_EXIT_SW2_IEN |= HAL_EXIT_SW2_IENBIT;
		HAL_EXIT_SW2_PXIFG &= ~(HAL_EXIT_SW2_BIT);

		/* Rising/Falling edge configuratinn */
		PICTL &= ~(HAL_EXIT_SW5_EDGEBIT);    //RESET_CBITS(PICTL,HAL_EXIT_SW1_EDGEBIT);
		/* For falling edge, the bit must be set. */
#if (HAL_EXIT_SW5_EDGE == HAL_KEY_FALLING_EDGE)
		PICTL |= HAL_EXIT_SW5_EDGEBIT;		//SET_CBITS(PICTL,HAL_EXIT_SW1_EDGEBIT);
#endif
		/* Interrupt configuration:
		 * - Enable interrupt generation at the port
		 * - Enable CPU interrupt
		 * - Clear any pending interrupt
		 */
		HAL_EXIT_SW5_ICTL |= HAL_EXIT_SW5_ICTLBIT;
		HAL_EXIT_SW5_IEN |= HAL_EXIT_SW5_IENBIT;
		HAL_EXIT_SW5_PXIFG &= ~(HAL_EXIT_SW5_BIT);

		/* Do this only after the hal_key is configured - to work with sleep stuff */
		if (HalKeyConfigured == TRUE)
		{
			osal_stop_timerEx(Hal_TaskID, HAL_KEY_EVENT);	/* Cancel polling if active */
		}
	}
	else    /* Interrupts NOT enabled */
	{
		//HAL_EXIT_SW1_ICTL &= ~(HAL_EXIT_SW1_ICTLBIT);		/* don't generate interrupt */
		//HAL_EXIT_SW1_IEN &= ~(HAL_EXIT_SW1_IENBIT);		/* Clear interrupt enable bit */
		osal_set_event(Hal_TaskID, HAL_KEY_EVENT);
	}
	HalKeyConfigured = TRUE;	/* Key now is configured */
}


/**************************************************************************************************
 * @fn      HalKeyRead
 *
 * @brief   Read the current value of a key
 *
 * @param   None
 *
 * @return  keys - current keys status
 **************************************************************************************************/
uint8 HalKeyRead ( void )
{
	uint8 keys = 0;

	if (HAL_PUSH_BUTTON1())
	{
		keys |= HAL_KEY_SW_1;
	}
	else if (HAL_PUSH_BUTTON2())
	{
		keys |= HAL_KEY_SW_2;
	}
	else if (HAL_PUSH_BUTTON3())
	{
		keys |= HAL_KEY_SW_3;
	}
	else if (HAL_PUSH_BUTTON4())
	{
		keys |= HAL_KEY_SW_4;
	}
	else if (HAL_PUSH_BUTTON5())
	{
		keys |= HAL_KEY_SW_5;
	}


#ifdef _SYS_POWER_ON_CHK
	if ((HAL_KEY_JOY_MOVE_PORT & HAL_KEY_JOY_MOVE_BIT))  /* Key is active low */
	{
		keys |= halGetJoyKeyInput();
	}
#endif

	return keys;
}


/**************************************************************************************************
 * @fn      HalKeyPoll
 *
 * @brief   Called by hal_driver to poll the keys per 100ms
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
extern void SampleApp_SendInMessage(void);
void HalKeyPoll (void)
{
	uint8 keys = 0;
#ifdef _SYS_POWER_ON_CHK
	if ((HAL_KEY_JOY_MOVE_PORT & HAL_KEY_JOY_MOVE_BIT))  /* Key is active HIGH */
	{
		//#if defined( PIR )		//for old vision board
		//keys = halGetJoyKeyInput();
		//#endif
	}
#endif

	/* If interrupts are not enabled, previous key status and current key status
	 * are compared to find out if a key has changed status.
	 */
	if (!Hal_KeyIntEnable)
	{
		if (keys == halKeySavedKeys)
		{
			/* Exit - since no keys have changed */
			return;
		}
		/* Store the current keys for comparation next time */
		halKeySavedKeys = keys;
	}
	else
	{
		if ( (key_ISR_FLAG==HAL_KEY_SW_1) && (!(HAL_PUSH_BUTTON1())) )//select
		{
			//ButtonSW(WLDB_BTN_SELE);
		}
		else if ( (key_ISR_FLAG==HAL_KEY_SW_2) && (!(HAL_PUSH_BUTTON2())) )//volum
		{
			//ButtonSW(WLDB_BTN_VOLU);
		}
		else if ( (key_ISR_FLAG==HAL_KEY_SW_5) && (!(HAL_PUSH_BUTTON5())) )//play
		{
			//ButtonSW(WLDB_BTN_PLAY);
			if(key_holdtime_waiting_flag)
			{
				key_holdtime_waiting_flag = 0;
				osal_stop_timerEx (Hal_TaskID, HAL_KEY_HOLD_EVENT);
				key_NWK_ISR_FLAG = 0;
			}

			key_NWK_ISR_FLAG++;
			osal_stop_timerEx (Hal_TaskID, HAL_KEY_MULTI_CLICK_EVENT);
			osal_start_timerEx (Hal_TaskID, HAL_KEY_MULTI_CLICK_EVENT, HAL_KEY_CLICKED_VALUE);
		}
		else
		{
			//Exceptions
		}
	}
	/* if pHalKeyProcessFunction  != NULL;
	make sure a key registered at least	.
	Invoke Callback if new keys were depressed */
	if (keys && (pHalKeyProcessFunction))
	{
		(pHalKeyProcessFunction) (keys, HAL_KEY_STATE_NORMAL);
		key_ISR_FLAG=0;
	}
}


/**************************************************************************************************
 * @fn      HalKey_double_detect
 *
 * @brief   Called by hal_driver to detect double tick    test
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalKey_double_detect()
{
	uint8 keys = 0;
	//#if (defined HAL_KEY) && (HAL_KEY == TRUE)
	keys |= HAL_KEY_SW_5;
	key_double_flag=key_NWK_ISR_FLAG;

	if (keys && (pHalKeyProcessFunction))
	{
		(pHalKeyProcessFunction) (keys, HAL_KEY_STATE_NORMAL);
		key_NWK_ISR_FLAG=0;
	}
}

/**************************************************************************************************
* @fn      halGetJoyKeyInput
*
* @brief   Map the ADC value to its corresponding key.
*
* @param   None
*
* @return  keys - current joy key status
**************************************************************************************************/
#ifdef _SYS_POWER_ON_CHK
uint8 halGetJoyKeyInput(void)
{
	/* The joystick control is encoded as an analog voltage.
	 * Read the JOY_LEVEL analog value and map it to joy movement.
	 */
	uint8 adc;
	uint8 ksave0 = 0;
	uint8 ksave1;

	/* Keep on reading the ADC until two consecutive key decisions are the same. */
	do
	{
		ksave1 = ksave0;    /* save previouse key reading */

		adc = HalAdcRead (HAL_KEY_JOY_CHN, HAL_ADC_RESOLUTION_8);

		if ((adc >= 2) && (adc <= 38))
		{
			//ksave0 |= HAL_KEY_UP;
		}
		else if ((adc >= 74) && (adc <= 88))
		{
			//ksave0 |= HAL_KEY_RIGHT;
		}
		else if ((adc >= 60) && (adc <= 73))
		{
			//ksave0 |= HAL_KEY_LEFT;
		}
		else if ((adc >= 39) && (adc <= 59))
		{
			///ksave0 |= HAL_KEY_DOWN;
		}
		else if ((adc >= 89) && (adc <= 100))
		{
			//ksave0 |= HAL_KEY_CENTER;
		}
	}
	while (ksave0 != ksave1);

	return ksave0;
}
#endif

/**************************************************************************************************
 * @fn      halProcessKeyInterrupt
 *
 * @brief   Checks to see if it's a valid key interrupt, saves interrupt driven key states for
 *          processing by HalKeyRead(), and debounces keys by scheduling HalKeyRead() 25ms later.
 *
 * @param
 *
 * @return
 **************************************************************************************************/
void halProcessKeyInterrupt (void)
{
	bool valid=FALSE;

	if (HAL_EXIT_SW1_PXIFG & HAL_EXIT_SW1_BIT)  /* Interrupt Flag has been set */
	{
		HAL_EXIT_SW1_PXIFG &= ~(HAL_EXIT_SW1_BIT); /* Clear Interrupt Flag */
		valid = TRUE;
	}
	if (HAL_EXIT_SW2_PXIFG & HAL_EXIT_SW2_BIT)  /* Interrupt Flag has been set */
	{
		HAL_EXIT_SW2_PXIFG &= ~(HAL_EXIT_SW2_BIT); /* Clear Interrupt Flag */
		valid = TRUE;
	}
	if (HAL_EXIT_SW5_PXIFG & HAL_EXIT_SW5_BIT)  /* Interrupt Flag has been set */
	{
		HAL_EXIT_SW5_PXIFG &= ~(HAL_EXIT_SW5_BIT); /* Clear Interrupt Flag */
		valid = TRUE;
	}

	if (valid)
	{
		osal_start_timerEx (Hal_TaskID, HAL_KEY_EVENT, HAL_KEY_DEBOUNCE_VALUE);
	}
}


/**************************************************************************************************
 * @fn      HalKeyEnterSleep
 *
 * @brief  - Get called to enter sleep mode
 *
 * @param
 *
 * @return
 **************************************************************************************************/
void HalKeyEnterSleep ( void )
{
}


/**************************************************************************************************
 * @fn      HalKeyExitSleep
 *
 * @brief   - Get called when sleep is over
 *
 * @param
 *
 * @return  - return saved keys
 **************************************************************************************************/
uint8 HalKeyExitSleep ( void )
{
	/* Wake up and read keys */
	return ( HalKeyRead () );
}


/**************************************************************************************************
 * @fn      halKeyPort0Isr
 *
 * @brief   Port0 ISR
 *
 * @param
 *
 * @return
 **************************************************************************************************/
HAL_ISR_FUNCTION( halKeyPort0Isr, P0INT_VECTOR )
{
	HAL_ENTER_ISR();

	HAL_KEY_CPU_PORT_0_IF = 0;
	CLEAR_SLEEP_MODE();
	HAL_EXIT_ISR();
}


/**************************************************************************************************
 * @fn      halKeyPort1Isr
 *
 * @brief   Port1 ISR
 *
 * @param
 *
 * @return
 **************************************************************************************************/
HAL_ISR_FUNCTION( halKeyPort1Isr, P1INT_VECTOR )
{
	HAL_ENTER_ISR();

	if(HAL_EXIT_SW1_PXIFG & HAL_EXIT_SW1_BIT)
	{
		halProcessKeyInterrupt();
		HAL_EXIT_SW1_PXIFG &= ~(HAL_EXIT_SW1_BIT);
		key_ISR_FLAG = HAL_KEY_SW_1;
	}
	if(HAL_EXIT_SW2_PXIFG & HAL_EXIT_SW2_BIT)
	{
		halProcessKeyInterrupt();
		HAL_EXIT_SW2_PXIFG &= ~(HAL_EXIT_SW2_BIT);
		key_ISR_FLAG = HAL_KEY_SW_2;
	}
	if(HAL_EXIT_SW5_PXIFG & HAL_EXIT_SW5_BIT)
	{
		halProcessKeyInterrupt();
		HAL_EXIT_SW5_PXIFG &= ~(HAL_EXIT_SW5_BIT);
		key_ISR_FLAG = HAL_KEY_SW_5;
	}
	/*
	  Clear the CPU interrupt flag for Port_1
	  PxIFG has to be cleared before PxIF
	*/
	HAL_KEY_CPU_PORT_1_IF = 0;
	// PCON = 0x00;//wakeup
	// CLEAR_SLEEP_MODE();
	HAL_EXIT_ISR();
}


/**************************************************************************************************
 * @fn      halKeyPort2Isr
 *
 * @brief   Port2 ISR
 *
 * @param
 *
 * @return
 **************************************************************************************************/
HAL_ISR_FUNCTION( halKeyPort2Isr, P2INT_VECTOR )
{
	HAL_ENTER_ISR();

	/*
	  Clear the CPU interrupt flag for Port_1
	  PxIFG has to be cleared before PxIF
	*/
	HAL_KEY_CPU_PORT_2_IF = 0;
	// PCON = 0x00;//wakeup
	// CLEAR_SLEEP_MODE();
	HAL_EXIT_ISR();
}


void HalKeyMultiClickCheck(void)
{
	uint8 keys = 0;

	keys |= HAL_KEY_SW_5;//KC4 @ P1.2
	key_double_flag = key_NWK_ISR_FLAG;
	if(1 == key_double_flag)
	{
		if(HAL_PUSH_BUTTON5())
		{
			key_hold_flag = 0;
			if (keys && (pHalKeyProcessFunction))
			{
				(pHalKeyProcessFunction) (keys, HAL_KEY_STATE_NORMAL);
				key_NWK_ISR_FLAG=0;
			}
		}
		else
		{
			//if (keys && (pHalKeyProcessFunction))
			//{
			(pHalKeyProcessFunction) (keys, HAL_KEY_STATE_NORMAL);
			//key_NWK_ISR_FLAG=0;
			//}// todo: remove comment to test!
			osal_start_timerEx (Hal_TaskID, HAL_KEY_HOLD_EVENT, HAL_KEY_HOLD_VALUE);
			key_holdtime_waiting_flag = 1;
		}
	}
	else
	{
		key_hold_flag = 0;
		if (keys && (pHalKeyProcessFunction))
		{
			(pHalKeyProcessFunction) (keys, HAL_KEY_STATE_NORMAL);
			key_NWK_ISR_FLAG=0;
		}
	}
}


void HalKeyHoldCheck(void)
{
	uint8 keys = 0;
	keys |= HAL_KEY_SW_5;
	if( !(HAL_PUSH_BUTTON5()) )
	{
		key_hold_flag = 1;
		if (keys && (pHalKeyProcessFunction))
		{
			(pHalKeyProcessFunction) (keys, HAL_KEY_STATE_NORMAL);
			key_NWK_ISR_FLAG=0;
		}
	}
	key_holdtime_waiting_flag = 0;
	key_NWK_ISR_FLAG=0;
}

#define MELODY_INIT_VOLUM		1
#define MELODY_MAX_DUR			10000
uint8 gPlayVal = 1;
uint8 gVolumVal = MELODY_INIT_VOLUM;//init state
uint8 gMelodyVal = 0;
/*
void ButtonSW(uint8 btn)
{
	swIo(btn);
	SampleApp_SendInMessage();// upload current switch state
}
*/

#define DBOP_MAX_DUR		2000//ms
uint8 gFlagOp = 1;
void swIo(uint8 tBtn,uint8 tBtm)
{
	if(gFlagOp)
	{
		switch(tBtn)
		{
			case WLDB_BTN_PLAY:
			{
				//gPlayVal = 1;
				
                                IO_PLAY_SBIT = 1;
				break;
			}
			case WLDB_BTN_SELE:
			{
				if(++gMelodyVal>37){gMelodyVal = 0;}
				IO_SELE_SBIT = 1;
				break;
			}
			case WLDB_BTN_VOLU:
			{
				if(--gVolumVal<1){gVolumVal = 3;}
				else if(gVolumVal>3){gVolumVal = 3;}
                              
				IO_VOLU_SBIT = 1;
				break;
			}
                        case WLDB_BTN_BUTTONEN:
			{
				
                                ButtonEn = tBtm;
                                if(ButtonEn==0)
                                {
                                 gPlayVal = 0;
                                }
                                else
                                {
                                 gPlayVal = 1;
                                }  
				
				break;
			}
			default:
			{
				break;
			}
		}
		gFlagOp = 0;
		//re-enable gFlagOp and clr after DBOP_MAX_DUR
osal_start_timerEx( SampleApp_TaskID,WLDB_CLRDB_EVENT,DBOP_MAX_DUR);//max , duration 2S
	}
	//clear output after 10ms
	osal_start_timerEx( SampleApp_TaskID, WLDB_CLRIO_EVENT,250);
}

void clrIo(void);
void clrIo(void)
{
	IO_PLAY_SBIT = 0;
	IO_SELE_SBIT = 0;
	IO_VOLU_SBIT = 0;
}

void clrDb(void);
void clrDb(void)
{
	//gPlayVal = 0;
	gFlagOp = 1;
	//SampleApp_SendInMessage();
}

uint8 getCurMelody(uint8 btn);
uint8 getCurMelody(uint8 btn)
{
	if(btn == WLDB_BTN_PLAY){return gPlayVal;}
	else if(btn == WLDB_BTN_SELE){return gMelodyVal;}
	else if(btn == WLDB_BTN_VOLU){return gVolumVal;}
	else return 0;
}
#else
#endif /* HAL_KEY */




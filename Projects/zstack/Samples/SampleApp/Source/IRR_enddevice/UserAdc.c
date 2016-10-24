/****************************************************************

 公司名称：
 模块名称：UserAdc.c
 功    能：CC2530 ADC            
 说    明：
 程序设计：   
 设计时间：        
 版 本 号: 
*********************************************************************/
#include "hal_defs.h"
#include "hal_adc.h"
#include "UserAdc.h"

uint8    BatVeryLow = 0;//严重欠压
uint8    BUTTERY_LOW_FLAG = 0;//电池欠压标志
uint8 Hal_VOC_Init( void );
uint8 Hal_VOC_Init( void )
{
  uint8  k = 0;
  uint8  iDat = 0;
  uint16 tempValue = 0;
  HAL_VOC_SEL |= HAL_VOC_BIT;       /* Set pin function to GPIO */
  HAL_VOC_DIR &= ~(HAL_VOC_BIT);    /* Set pin direction to Input */
//  ADCCFG   |= BV(3);
  ADCCON3  |= Vref; //基准为3.3V 
  
  for(k=0;k<250;k++);
  {
    tempValue = HalAdcRead (HAL_VOC_CHN, HAL_ADC_RESOLUTION_10); 
    if(tempValue<=VoltageVeryLow)
    {
      BUTTERY_LOW_FLAG = 1;
      iDat = 1;
    }
  }
  
  tempValue = HalAdcRead (HAL_VOC_CHN, HAL_ADC_RESOLUTION_10); 
  return iDat;
} 

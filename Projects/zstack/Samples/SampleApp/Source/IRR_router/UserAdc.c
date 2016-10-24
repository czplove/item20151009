/****************************************************************

 ��˾���ƣ�
 ģ�����ƣ�UserAdc.c
 ��    �ܣ�CC2530 ADC            
 ˵    ����
 ������ƣ�   
 ���ʱ�䣺        
 �� �� ��: 
*********************************************************************/
#include "hal_defs.h"
#include "hal_adc.h"
#include "UserAdc.h"

uint8    BatVeryLow = 0;//����Ƿѹ
uint8    BUTTERY_LOW_FLAG = 0;//���Ƿѹ��־
uint8 Hal_VOC_Init( void );
uint8 Hal_VOC_Init( void )
{
  uint8  k = 0;
  uint8  iDat = 0;
  uint16 tempValue = 0;
  HAL_VOC_SEL |= HAL_VOC_BIT;       /* Set pin function to GPIO */
  HAL_VOC_DIR &= ~(HAL_VOC_BIT);    /* Set pin direction to Input */
//  ADCCFG   |= BV(3);
  ADCCON3  |= Vref; //��׼Ϊ3.3V 
  
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

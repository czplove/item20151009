
/**************************************************
  * 函数原型及相关变量
  * 相关文件：UserAdc.h
  * 编写日期：2012.8.2
  * 修改历史:
  

**************************************************/


#ifndef USERADC_H_
#define USERADC_H_
#include "hal_types.h"
#include "ioCC2530.h"
#define HAL_VOC_BIT    BV(3)//BV(6)
#define HAL_VOC_SEL    P0SEL
#define HAL_VOC_DIR    P0DIR
#define HAL_VOC_CHN    HAL_ADC_CHANNEL_3
#define Vref           0x80//    AVDD5
#define VoltageLow      372//1.15V*2 =2.4V 2.4/3.3*512 = 372.36
#define VoltageVeryLow  341//1V*2 = 2.2V   2.2/3.3*512 = 341.33
extern uint8    BatVeryLow ;//严重欠压
extern uint8    BUTTERY_LOW_FLAG;
extern uint8 Hal_VOC_Init( void );

#endif
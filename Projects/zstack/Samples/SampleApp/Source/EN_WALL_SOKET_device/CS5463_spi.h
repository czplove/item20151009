/******************** (C) COPYRIGHT 2014 Owner************** *******************
* File Name		: CS5463_spi.h
* Author		: zhoukh@yuantuo.mobi
* Version		: V1.0.0
* Date			: 06/06/2014
* Description	        : CC2530通过spi对CS5463 操作函数定义及全局参数
*******************************************************************************/

#ifndef __CS5463SPI_H
#define __CS5463SPI_H

//文件包含----------------------------------------------------------------------
#include <ioCC2530.h>
#include <string.h>
#include <stdio.h>
#include <hal_types.h>
//全局类型宏定义 ---------------------------------------------------------------
#define sint8    char
#define sint16   int
#define sint32   long

typedef float(*tfc)();
typedef void (*tcb_fc)(void* para);
typedef enum
{
  Success = 0,
  Fail    = 1
}OperStatus;

typedef enum
{
  Enable  = 0,
  Disable = 1
}FunStatus;

//位操作定义--------------------------------------------------------------------
#define CBIT0   (1<<0)
#define NCBIT0  (~CBIT0)

#define CBIT1   (1<<1)
#define NCBIT1  (~CBIT1)

#define CBIT2   (1<<2)
#define NCBIT2  (~CBIT2)

#define CBIT3   (1<<3)
#define NCBIT3  (~CBIT3)

#define CBIT4   (1<<4)
#define NCBIT4  (~CBIT4)

#define CBIT5   (1<<5)
#define NCBIT5  (~CBIT5)

#define CBIT6   (1<<6)
#define NCBIT6  (~CBIT6)

#define CBIT7   (1<<7)
#define NCBIT7  (~CBIT7)

#define SET_CBITS(reg,bitx)           ((reg) |= (bitx))
#define RESET_CBITS(reg,bitx)         ((reg) &= (~(bitx)))

#define OS_ENTER_CRITICAL()            EA = 0
#define OS_EXIT_CRITICAL()             EA = 1


//函数宏定义--------------------------------------------------------------------

#define _PRAGMA(x) _Pragma(#x)     //定义预处理指令宏  
#define HAL_ISR_FUNC_DECLARATION(f,v)   _PRAGMA(vector=v) __near_func __interrupt void f(void)      //IAR中断函数声明定义格式宏  
#define HAL_ISR_FUNC_PROTOTYPE(f,v)     _PRAGMA(vector=v) __near_func __interrupt void f(void)      //IAR中断函数原型定义格式宏  
#define HAL_ISR_FUNCTION(f,v)           HAL_ISR_FUNC_PROTOTYPE(f,v); HAL_ISR_FUNC_DECLARATION(f,v)      //IAR中断函数定义格式宏 

//高低电平宏定义----------------------------------------------------------------
#define HIGH_LEVEL    1
#define LOW_LEVEL     0

//定义控制LED灯的端口-----------------------------------------------------------
#define LED1 P0_0
#define LED2 P0_1

//定义复位管脚------------------------------------------------------------------
#define RELAYS1 P1_1

//CS5463参数定义----------------------------------------------------------------
/**************************启动转换(Start Conversions) 操作命令*****************
*:    0 = 执行单计算周期   —————— 0xE0
*:    1 = 执行连续计算周期 —————— 0xE8
*******************************************************************************/
#define START_C30_CMD                 0xE0
#define START_C31_CMD                 0xE8

/**************************SYNC1 和 SYNC0 操作命令******************************
*:    0 = 结束串口重新初始化 —————— 0xFE
*:    1 = 开始串口重新初始化 —————— 0xFF
*******************************************************************************/
#define SYNC_0_CMD                    0xFE
#define SYNC_1_CMD                    0xFF

/**************************上电/暂停命令(Power-up/Halt)操作命令*****************
*:    如果芯片进入掉电模式，本命令将使芯片上电。
*:    如果芯片已通电，则此命令将使所有计算暂停。—— 0xA0
*******************************************************************************/
#define PWR_UP_HALT_CMD               0xA0 

/******************掉电/软件复位命令(Power-Down/Software Reset操作命令**********
*:    S[1：0] 掉电模式
*:    00 = 软件复位———— 0x80
*:    01 = 暂停并进入等待模式，这种模式允许快速上电。 ———— 0x88
*:    10 = 暂停并进入睡眠模式，这种模式要求一个较长的上电时间。 ———— 0x90
*:    11 = 保留 ———— 0x98
*******************************************************************************/
#define RESET_S00_CMD                 0x80
#define WAIT_MODE_S01_CMD             0x88
#define SLEEP_MODE_S10_CMD            0x90

/********************校准命令(Calibration)**************************************
*:    在执行校准操作前，用户必须给芯片提供适当的输入。
*:    CAL[4:0] 执行指定校准
*******************************************************************************/
//01001 = 电流通道DC偏移 Idcoff ———— 0xC9
#define CAL_IDCOFF_CMD                0xC9
//01010 = 电流通道DC增益 Idcgn ———— 0xCA
#define CAL_IDCGN_CMD                 0xCA
//01101 = 电流通道AC偏移 Iacoff———— 0xCD
#define CAL_IACOFF_CMD                0xCD
//01110 = 电流通道AC增益 Iacgn———— 0xCE
#define CAL_IACGN_CMD                 0xCE
//10001 = 电压通道DC偏移 Vdcoff———— 0xD1
#define CAL_VDCOFF_CMD                0xD1
//10010 = 电压通道DC增益 Vdcgn———— 0xD2
#define CAL_VDCGN_CMD                 0xD2    
//10101 = 电压通道AC偏移 Vacoff———— 0xD5
#define CAL_VACOFF_CMD                0xD5
//10110 = 电压通道AC增益 Vacgn———— 0xD6
#define CAL_VACGN_CMD                 0xD6
//11001 = 电流和电压通道DC偏移 VIdcoff———— 0xD9
#define CAL_VIDCOFF_CMD               0xD9
//11010 = 电流和电压通道DC增益 VIdcgn———— 0xDA
#define CAL_VIDCGN_CMD                0xDA
//11101 = 电流和电压通道AC偏移 VIacoff———— 0xDD
#define CAL_VIACOFF_CMD               0xDD
//11110 = 电流和电压通道AC增益 VIacgn———— 0xDE
#define CAL_VIACGN_CMD                0xDE

/******************寄存器读/写命令(Register Read/Write)************************/

/*********************************页 操作命令**********************************/
#define OPER_PAGE_CMD                 0x7E
#define OPER_PAGE0_CMD                0x000000
#define OPER_PAGE1_CMD                0x000001
#define OPER_PAGE3_CMD                0x000003

/*********************************寄存器页0 操作命令****************************
*:    地址 RA[4:0] 名称 描述 读 写
*******************************************************************************/
//0 00000 Config 配置0x00 0x40
#define READ_CFG_CMD                  0x00
#define WRITE_CFG_CMD                 0x40
//1 00001 Idcoff 电流通道DC偏移0x02 0x42
#define READ_IDCOFF_CMD               0x02
#define WRITE_IDCOFF_CMD              0x42
//2 00010 Ign 电流通道增益0x04 0x44
#define READ_IGN_CMD                  0x04
#define WRITE_IGN_CMD                 0x44
//3 00011 Vdcoff 电压通道DC偏移0x06 0x46
#define READ_VDCOFF_CMD               0x06
#define WRITE_VDCOFF_CMD              0x46
//4 00100 Vgn 电压通道增益0x08 0x48
#define READ_VGN_CMD                  0x08
#define WRITE_VGN_CMD                 0x48
//5 00101 Cycle Count 一个计算周期的A/D转换数 ———————— 0x0A 0x4A
#define READ_CYCLE_CNT_CMD            0x0A
#define WRITE_CYCLE_CNT_CMD           0x4A
//6 00110 PulseRateE 设置E1、E2、E3的能量-脉冲速率 ————— 0x0C 0x4C
#define READ_PLUSE_RATE_CMD           0x0C
#define WRITE_PLUSE_RATE_CMD          0x4C
//7 00111 I 瞬时电流 ———————————— 0x0E 0x4E
#define READ_INS_I_CMD                0x0E
#define WRITE_INS_I_CMD               0x4E
//8 01000 V 瞬时电压 ———————————— 0x10 0x50
#define READ_INS_V_CMD                0x10
#define WRITE_INS_V_CMD               0x50
//9 01001 P 瞬时功率 ———————————— 0x12 0x52
#define READ_INS_P_CMD                0x12
#define WRITE_INS_P_CMD               0x52
//10 01010 Pactive 有功功率 ———————————— 0x14 0x54
#define READ_PACTIVE_CMD              0x14
#define WRITE_PACTIVE_CMD             0x54
//11 01011 Irms 电流有效值 ———————————— 0x16 0x56
#define READ_IRMS_CMD                 0x16
#define WRITE_IRMS_CMD                0x56
//12 01100 Vrms 电压有效值 ———————————— 0x18 0x58
#define READ_VRMS_CMD                 0x18
#define WRITE_VRMS_CMD                0x58
//13 01101 ∈ 在一个电线周期(50或60Hz)中取样间隔的倒数 0x1A 0x5A
#define READ_SAMP_TIM_CMD             0x1A
#define WRITE_SAMP_TIM_CMD            0x5A
//14 01110 Poff 功率偏移量 ———————————— 0x1C 0x5C
#define READ_POFF_CMD                 0x1C
#define WRITE_POFF_CMD                0x5C
//15 01111 Status 状态 ———————————— 0x1E 0x5E
#define READ_STATUS_CMD               0x1E
#define WRITE_STATUS_CMD              0x5E
//16 10000 Iacoff 电流通道AC偏移 ———————————— 0x20 0x60
#define READ_IACOFF_CMD               0x20
#define WRITE_IACOFF_CMD              0x60
//17 10001 Vacoff 电压通道AC偏移 ———————————— 0x22 0x62
#define READ_VACOFF_CMD               0x22
#define WRITE_VACOFF_CMD              0x62
//18 10010 Mode 操作模式 ———————————— 0x24 0x64
#define READ_MODE_CMD                 0x24
#define WRITE_MODE_CMD                0x64
//19 10011 T 温度 ———————————— 0x26 0x66
#define READ_TEMPER_CMD               0x26
#define WRITE_TEMPER_CMD              0x66       
//20 10100 Qavg 平均无功功率(90°移相) ————————— 0x28 0x68
#define READ_QAVG_CMD                 0x28
#define WRITE_QAVG_CMD                0x68
//21 10101 Q 瞬时无功功率(90°移相) ————————— 0x2A 0x6A
#define READ_INS_Q_CMD                0x2A
#define WRITE_INS_Q_CMD               0x6A
//22 10110 Ipeak 峰值电流 ———————————— 0x2C 0x6C
#define READ_IPEAK_CMD                0x2C
#define WRITE_IPEAK_CMD               0x6C
//23 10111 Vpeak 峰值电压 ———————————— 0x2E 0x6E
#define READ_VPEAK_CMD                0x2E
#define WRITE_VPEAK_CMD               0x6E
//24 11000 Qtrig 用功率三角形计算出的无功功率 —————— 0x30 0x70
#define READ_QTRIG_CMD                0x30
#define WRITE_QTRIG_CMD               0x70
//25 11001 PF 功率因数 ———————————— 0x32 0x72
#define READ_PF_CMD                   0x32
#define WRITE_PF_CMD                  0x72
//26 11010 Mask 中断屏蔽 ———————————— 0x34 0x74
#define READ_INT_MASK_CMD             0x34
#define WRITE_INT_MASK_CMD            0x74
//27 11011 S 视在功率 ———————————— 0x36 0x76
#define READ_S_CMD                    0x36
#define WRITE_S_CMD                   0x76
//28 11100 Ctrl 控制 ———————————— 0x38 0x78
#define READ_CTRL_CMD                 0x38
#define WRITE_CTRL_CMD                0x78
//29 11101 Ph 谐波有功功率 ———————————— 0x3A 0x7A
#define READ_PH_CMD                   0x3A
#define WRITE_PH_CMD                  0x7A
//30 11110 Pf 基波有功功率 ———————————— 0x3C 0x7C
#define READ_PFUND_CMD                0x3C
#define WRITE_PFUND_CMD               0x7C
//31 11111 Qf 基波无功功率 ———————————— 0x3E 0x7E
#define READ_QFUND_CMD                0x3E
#define WRITE_QFUND_CMD               0x7E
/***************************endof 寄存器页0************************************/

/*********************************寄存器页1 操作命令***************************/
//地址 RA[4:0] 名称 描述 读 写
//2 00010 Tgain 温度传感器增益 ———————————— 0x04 0x44
#define READ_TGAIN_CMD                0x04
#define WRITE_TGAIN_CMD               0x44
//3 00011 Toff 温度传感器偏移 ———————————— 0x06 0x46
#define READ_TOFF_CMD                 0x06
#define WRITE_TOFF_CMD                0x46

/*********************************寄存器页3 操作命令***************************/
//地址 RA[4:0] 名称 描述 读 写
//6 00110 VSAGcycle 电压下跌持续时间 —————————— 0x0C 0x4C
#define READ_VSAGCYCLE_CMD            0x0C
#define WRITE_VSAGCYCLE_CMD           0x4C
//7 00111 VSAGlevel 电压下跌电平 —————————— 0x0E 0x4E
#define READ_VSAGLEVEL_CMD            0x0E
#define WRITE_VSAGLEVEL_CMD           0x4E
//10 01010 ISAGcycle 电流下跌持续时间 —————————— 0x14 0x54
#define READ_ISAGCYCLE_CMD            0x14
#define WRITE_ISAGCYCLE_CMD           0x54
//11 01011 ISAGlevel 电流下跌电平 —————————— 0x16 0x56
#define READ_ISAGLEVEL_CMD            0x16
#define WRITE_ISAGLEVEL_CMD           0x56
/*********************************校准值***************************************/
#define VDCOFF_DATA  0x070101
//#define VACOFF_DATA	 0x000000
#define IDCOFF_DATA  0xfe2727
//#define IACOFF_DATA  0xFFFD2C
#define VGN_DATA     0x3C7FFF
#define IGN_DATA     0x000000
/*********************************end of CS5463********************************/
//管脚定义----------------------------------------------------------------------
#define  CS5463_SDI     P0_2   //P0_2// MISO主机
#define  CS5463_SCLK    P0_5   //P0_2//  
#define  CS5463_SDO     P0_3   //P1_0// MOSI
#define  CS5463_CS      P0_6   //P0_3//  
#define  INT     P2_0
#define  CS5463_RESET   P1_3
//其他宏定义--------------------------------------------------------------------
#define  DELAY     50

#define INT_DRDY      (1L<<23) // 数据就绪中断
#define INT_FUP       (1L<<9)  // ∈值更新中断
#define INT_TUP       (1L<<7)  // 温度更新中断
#define INT_TAB_LAB       3
//变量类型定义------------------------------------------------------------------
typedef enum
{
  int_drdy = 0,// 数据就绪中断
  int_int_pup,// ∈值更新中断
  int_tup     // 温度更新中断
}INT_TypeDef;

typedef enum
{
  cali_vdcoff, //电压通道DC偏移
  cali_vacoff, //电压通道AC偏移
  cali_idcoff, //电流通道DC偏移
  cali_iacoff, //电流通道AC偏移
  cali_vacgain,//电压通道AC增益
  cali_iacgain //电流通道AC增益寄
}CALI_TypeDef;

//变量类型定义------------------------------------------------------------------
extern uint32  CS543_CFG_DATA ;


//函数声明----------------------------------------------------------------------
extern void   InitCS5463(void);//初始化 CS5463
extern uint32 GetCalibrationVIoffOrGain(CALI_TypeDef cali_num);//偏移量,增益校准
extern void   ConfigCS5463(void);//配置CS5463uint32* cali_msg

extern void   ClearIntFlagStatus(uint8 int_bit);// 清除对应的中断标志
extern uint8  GetIntPendingBit(INT_TypeDef int_bit);// 检查对应的中断是否发生

extern sint32 GetSamplingPeriod(void);                       //获取采样周期
extern uint32 GetCurrentIrms(float ratioi, uint32 valueapp,uint32 valueapp_discrepancy); //获取电流有效值
extern uint32 GetVoltageVrms(float ratiov, uint32 valueapp,uint32 valueapp_discrepancy); //获取电压有效值
extern sint32 GetPactive(float ratiop, uint32 valueapp,uint32 valueapp_discrepancy);     //获取有功功率
extern uint32 GetApparenPower(float ratios, uint32 valueapp,uint32 valueapp_discrepancy);//获取视在功率
extern sint32 GetPowerFactor(float ratios, uint32 valueapp,uint32 valueapp_discrepancy); //获取功率因数
extern sint32 GetTemperature(uint32 Temper_App_Value);       //获取温度值

extern uint32 Read_CFG_Regester(void);

extern uint32 GetCurrentIvalue(float ratioi, uint32 valueapp, uint32 valueapp_discrepancy);
#endif
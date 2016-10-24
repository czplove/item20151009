/******************** (C) COPYRIGHT 2014 Owner************** *******************
* File Name		: CS5463_spi.h
* Author		: zhoukh@yuantuo.mobi
* Version		: V1.0.0
* Date			: 06/06/2014
* Description	        : CC2530ͨ��spi��CS5463 �����������弰ȫ�ֲ���
*******************************************************************************/

#ifndef __CS5463SPI_H
#define __CS5463SPI_H

//�ļ�����----------------------------------------------------------------------
#include <ioCC2530.h>
#include <string.h>
#include <stdio.h>
#include <hal_types.h>
//ȫ�����ͺ궨�� ---------------------------------------------------------------
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

//λ��������--------------------------------------------------------------------
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


//�����궨��--------------------------------------------------------------------

#define _PRAGMA(x) _Pragma(#x)     //����Ԥ����ָ���  
#define HAL_ISR_FUNC_DECLARATION(f,v)   _PRAGMA(vector=v) __near_func __interrupt void f(void)      //IAR�жϺ������������ʽ��  
#define HAL_ISR_FUNC_PROTOTYPE(f,v)     _PRAGMA(vector=v) __near_func __interrupt void f(void)      //IAR�жϺ���ԭ�Ͷ����ʽ��  
#define HAL_ISR_FUNCTION(f,v)           HAL_ISR_FUNC_PROTOTYPE(f,v); HAL_ISR_FUNC_DECLARATION(f,v)      //IAR�жϺ��������ʽ�� 

//�ߵ͵�ƽ�궨��----------------------------------------------------------------
#define HIGH_LEVEL    1
#define LOW_LEVEL     0

//�������LED�ƵĶ˿�-----------------------------------------------------------
#define LED1 P0_0
#define LED2 P0_1

//���帴λ�ܽ�------------------------------------------------------------------
#define RELAYS1 P1_1

//CS5463��������----------------------------------------------------------------
/**************************����ת��(Start Conversions) ��������*****************
*:    0 = ִ�е���������   ������������ 0xE0
*:    1 = ִ�������������� ������������ 0xE8
*******************************************************************************/
#define START_C30_CMD                 0xE0
#define START_C31_CMD                 0xE8

/**************************SYNC1 �� SYNC0 ��������******************************
*:    0 = �����������³�ʼ�� ������������ 0xFE
*:    1 = ��ʼ�������³�ʼ�� ������������ 0xFF
*******************************************************************************/
#define SYNC_0_CMD                    0xFE
#define SYNC_1_CMD                    0xFF

/**************************�ϵ�/��ͣ����(Power-up/Halt)��������*****************
*:    ���оƬ�������ģʽ�������ʹоƬ�ϵ硣
*:    ���оƬ��ͨ�磬������ʹ���м�����ͣ������ 0xA0
*******************************************************************************/
#define PWR_UP_HALT_CMD               0xA0 

/******************����/�����λ����(Power-Down/Software Reset��������**********
*:    S[1��0] ����ģʽ
*:    00 = �����λ�������� 0x80
*:    01 = ��ͣ������ȴ�ģʽ������ģʽ��������ϵ硣 �������� 0x88
*:    10 = ��ͣ������˯��ģʽ������ģʽҪ��һ���ϳ����ϵ�ʱ�䡣 �������� 0x90
*:    11 = ���� �������� 0x98
*******************************************************************************/
#define RESET_S00_CMD                 0x80
#define WAIT_MODE_S01_CMD             0x88
#define SLEEP_MODE_S10_CMD            0x90

/********************У׼����(Calibration)**************************************
*:    ��ִ��У׼����ǰ���û������оƬ�ṩ�ʵ������롣
*:    CAL[4:0] ִ��ָ��У׼
*******************************************************************************/
//01001 = ����ͨ��DCƫ�� Idcoff �������� 0xC9
#define CAL_IDCOFF_CMD                0xC9
//01010 = ����ͨ��DC���� Idcgn �������� 0xCA
#define CAL_IDCGN_CMD                 0xCA
//01101 = ����ͨ��ACƫ�� Iacoff�������� 0xCD
#define CAL_IACOFF_CMD                0xCD
//01110 = ����ͨ��AC���� Iacgn�������� 0xCE
#define CAL_IACGN_CMD                 0xCE
//10001 = ��ѹͨ��DCƫ�� Vdcoff�������� 0xD1
#define CAL_VDCOFF_CMD                0xD1
//10010 = ��ѹͨ��DC���� Vdcgn�������� 0xD2
#define CAL_VDCGN_CMD                 0xD2    
//10101 = ��ѹͨ��ACƫ�� Vacoff�������� 0xD5
#define CAL_VACOFF_CMD                0xD5
//10110 = ��ѹͨ��AC���� Vacgn�������� 0xD6
#define CAL_VACGN_CMD                 0xD6
//11001 = �����͵�ѹͨ��DCƫ�� VIdcoff�������� 0xD9
#define CAL_VIDCOFF_CMD               0xD9
//11010 = �����͵�ѹͨ��DC���� VIdcgn�������� 0xDA
#define CAL_VIDCGN_CMD                0xDA
//11101 = �����͵�ѹͨ��ACƫ�� VIacoff�������� 0xDD
#define CAL_VIACOFF_CMD               0xDD
//11110 = �����͵�ѹͨ��AC���� VIacgn�������� 0xDE
#define CAL_VIACGN_CMD                0xDE

/******************�Ĵ�����/д����(Register Read/Write)************************/

/*********************************ҳ ��������**********************************/
#define OPER_PAGE_CMD                 0x7E
#define OPER_PAGE0_CMD                0x000000
#define OPER_PAGE1_CMD                0x000001
#define OPER_PAGE3_CMD                0x000003

/*********************************�Ĵ���ҳ0 ��������****************************
*:    ��ַ RA[4:0] ���� ���� �� д
*******************************************************************************/
//0 00000 Config ����0x00 0x40
#define READ_CFG_CMD                  0x00
#define WRITE_CFG_CMD                 0x40
//1 00001 Idcoff ����ͨ��DCƫ��0x02 0x42
#define READ_IDCOFF_CMD               0x02
#define WRITE_IDCOFF_CMD              0x42
//2 00010 Ign ����ͨ������0x04 0x44
#define READ_IGN_CMD                  0x04
#define WRITE_IGN_CMD                 0x44
//3 00011 Vdcoff ��ѹͨ��DCƫ��0x06 0x46
#define READ_VDCOFF_CMD               0x06
#define WRITE_VDCOFF_CMD              0x46
//4 00100 Vgn ��ѹͨ������0x08 0x48
#define READ_VGN_CMD                  0x08
#define WRITE_VGN_CMD                 0x48
//5 00101 Cycle Count һ���������ڵ�A/Dת���� ���������������� 0x0A 0x4A
#define READ_CYCLE_CNT_CMD            0x0A
#define WRITE_CYCLE_CNT_CMD           0x4A
//6 00110 PulseRateE ����E1��E2��E3������-�������� ���������� 0x0C 0x4C
#define READ_PLUSE_RATE_CMD           0x0C
#define WRITE_PLUSE_RATE_CMD          0x4C
//7 00111 I ˲ʱ���� ������������������������ 0x0E 0x4E
#define READ_INS_I_CMD                0x0E
#define WRITE_INS_I_CMD               0x4E
//8 01000 V ˲ʱ��ѹ ������������������������ 0x10 0x50
#define READ_INS_V_CMD                0x10
#define WRITE_INS_V_CMD               0x50
//9 01001 P ˲ʱ���� ������������������������ 0x12 0x52
#define READ_INS_P_CMD                0x12
#define WRITE_INS_P_CMD               0x52
//10 01010 Pactive �й����� ������������������������ 0x14 0x54
#define READ_PACTIVE_CMD              0x14
#define WRITE_PACTIVE_CMD             0x54
//11 01011 Irms ������Чֵ ������������������������ 0x16 0x56
#define READ_IRMS_CMD                 0x16
#define WRITE_IRMS_CMD                0x56
//12 01100 Vrms ��ѹ��Чֵ ������������������������ 0x18 0x58
#define READ_VRMS_CMD                 0x18
#define WRITE_VRMS_CMD                0x58
//13 01101 �� ��һ����������(50��60Hz)��ȡ������ĵ��� 0x1A 0x5A
#define READ_SAMP_TIM_CMD             0x1A
#define WRITE_SAMP_TIM_CMD            0x5A
//14 01110 Poff ����ƫ���� ������������������������ 0x1C 0x5C
#define READ_POFF_CMD                 0x1C
#define WRITE_POFF_CMD                0x5C
//15 01111 Status ״̬ ������������������������ 0x1E 0x5E
#define READ_STATUS_CMD               0x1E
#define WRITE_STATUS_CMD              0x5E
//16 10000 Iacoff ����ͨ��ACƫ�� ������������������������ 0x20 0x60
#define READ_IACOFF_CMD               0x20
#define WRITE_IACOFF_CMD              0x60
//17 10001 Vacoff ��ѹͨ��ACƫ�� ������������������������ 0x22 0x62
#define READ_VACOFF_CMD               0x22
#define WRITE_VACOFF_CMD              0x62
//18 10010 Mode ����ģʽ ������������������������ 0x24 0x64
#define READ_MODE_CMD                 0x24
#define WRITE_MODE_CMD                0x64
//19 10011 T �¶� ������������������������ 0x26 0x66
#define READ_TEMPER_CMD               0x26
#define WRITE_TEMPER_CMD              0x66       
//20 10100 Qavg ƽ���޹�����(90������) ������������������ 0x28 0x68
#define READ_QAVG_CMD                 0x28
#define WRITE_QAVG_CMD                0x68
//21 10101 Q ˲ʱ�޹�����(90������) ������������������ 0x2A 0x6A
#define READ_INS_Q_CMD                0x2A
#define WRITE_INS_Q_CMD               0x6A
//22 10110 Ipeak ��ֵ���� ������������������������ 0x2C 0x6C
#define READ_IPEAK_CMD                0x2C
#define WRITE_IPEAK_CMD               0x6C
//23 10111 Vpeak ��ֵ��ѹ ������������������������ 0x2E 0x6E
#define READ_VPEAK_CMD                0x2E
#define WRITE_VPEAK_CMD               0x6E
//24 11000 Qtrig �ù��������μ�������޹����� ������������ 0x30 0x70
#define READ_QTRIG_CMD                0x30
#define WRITE_QTRIG_CMD               0x70
//25 11001 PF �������� ������������������������ 0x32 0x72
#define READ_PF_CMD                   0x32
#define WRITE_PF_CMD                  0x72
//26 11010 Mask �ж����� ������������������������ 0x34 0x74
#define READ_INT_MASK_CMD             0x34
#define WRITE_INT_MASK_CMD            0x74
//27 11011 S ���ڹ��� ������������������������ 0x36 0x76
#define READ_S_CMD                    0x36
#define WRITE_S_CMD                   0x76
//28 11100 Ctrl ���� ������������������������ 0x38 0x78
#define READ_CTRL_CMD                 0x38
#define WRITE_CTRL_CMD                0x78
//29 11101 Ph г���й����� ������������������������ 0x3A 0x7A
#define READ_PH_CMD                   0x3A
#define WRITE_PH_CMD                  0x7A
//30 11110 Pf �����й����� ������������������������ 0x3C 0x7C
#define READ_PFUND_CMD                0x3C
#define WRITE_PFUND_CMD               0x7C
//31 11111 Qf �����޹����� ������������������������ 0x3E 0x7E
#define READ_QFUND_CMD                0x3E
#define WRITE_QFUND_CMD               0x7E
/***************************endof �Ĵ���ҳ0************************************/

/*********************************�Ĵ���ҳ1 ��������***************************/
//��ַ RA[4:0] ���� ���� �� д
//2 00010 Tgain �¶ȴ��������� ������������������������ 0x04 0x44
#define READ_TGAIN_CMD                0x04
#define WRITE_TGAIN_CMD               0x44
//3 00011 Toff �¶ȴ�����ƫ�� ������������������������ 0x06 0x46
#define READ_TOFF_CMD                 0x06
#define WRITE_TOFF_CMD                0x46

/*********************************�Ĵ���ҳ3 ��������***************************/
//��ַ RA[4:0] ���� ���� �� д
//6 00110 VSAGcycle ��ѹ�µ�����ʱ�� �������������������� 0x0C 0x4C
#define READ_VSAGCYCLE_CMD            0x0C
#define WRITE_VSAGCYCLE_CMD           0x4C
//7 00111 VSAGlevel ��ѹ�µ���ƽ �������������������� 0x0E 0x4E
#define READ_VSAGLEVEL_CMD            0x0E
#define WRITE_VSAGLEVEL_CMD           0x4E
//10 01010 ISAGcycle �����µ�����ʱ�� �������������������� 0x14 0x54
#define READ_ISAGCYCLE_CMD            0x14
#define WRITE_ISAGCYCLE_CMD           0x54
//11 01011 ISAGlevel �����µ���ƽ �������������������� 0x16 0x56
#define READ_ISAGLEVEL_CMD            0x16
#define WRITE_ISAGLEVEL_CMD           0x56
/*********************************У׼ֵ***************************************/
#define VDCOFF_DATA  0x070101
//#define VACOFF_DATA	 0x000000
#define IDCOFF_DATA  0xfe2727
//#define IACOFF_DATA  0xFFFD2C
#define VGN_DATA     0x3C7FFF
#define IGN_DATA     0x000000
/*********************************end of CS5463********************************/
//�ܽŶ���----------------------------------------------------------------------
#define  CS5463_SDI     P0_2   //P0_2// MISO����
#define  CS5463_SCLK    P0_5   //P0_2//  
#define  CS5463_SDO     P0_3   //P1_0// MOSI
#define  CS5463_CS      P0_6   //P0_3//  
#define  INT     P2_0
#define  CS5463_RESET   P1_3
//�����궨��--------------------------------------------------------------------
#define  DELAY     50

#define INT_DRDY      (1L<<23) // ���ݾ����ж�
#define INT_FUP       (1L<<9)  // ��ֵ�����ж�
#define INT_TUP       (1L<<7)  // �¶ȸ����ж�
#define INT_TAB_LAB       3
//�������Ͷ���------------------------------------------------------------------
typedef enum
{
  int_drdy = 0,// ���ݾ����ж�
  int_int_pup,// ��ֵ�����ж�
  int_tup     // �¶ȸ����ж�
}INT_TypeDef;

typedef enum
{
  cali_vdcoff, //��ѹͨ��DCƫ��
  cali_vacoff, //��ѹͨ��ACƫ��
  cali_idcoff, //����ͨ��DCƫ��
  cali_iacoff, //����ͨ��ACƫ��
  cali_vacgain,//��ѹͨ��AC����
  cali_iacgain //����ͨ��AC�����
}CALI_TypeDef;

//�������Ͷ���------------------------------------------------------------------
extern uint32  CS543_CFG_DATA ;


//��������----------------------------------------------------------------------
extern void   InitCS5463(void);//��ʼ�� CS5463
extern uint32 GetCalibrationVIoffOrGain(CALI_TypeDef cali_num);//ƫ����,����У׼
extern void   ConfigCS5463(void);//����CS5463uint32* cali_msg

extern void   ClearIntFlagStatus(uint8 int_bit);// �����Ӧ���жϱ�־
extern uint8  GetIntPendingBit(INT_TypeDef int_bit);// ����Ӧ���ж��Ƿ���

extern sint32 GetSamplingPeriod(void);                       //��ȡ��������
extern uint32 GetCurrentIrms(float ratioi, uint32 valueapp,uint32 valueapp_discrepancy); //��ȡ������Чֵ
extern uint32 GetVoltageVrms(float ratiov, uint32 valueapp,uint32 valueapp_discrepancy); //��ȡ��ѹ��Чֵ
extern sint32 GetPactive(float ratiop, uint32 valueapp,uint32 valueapp_discrepancy);     //��ȡ�й�����
extern uint32 GetApparenPower(float ratios, uint32 valueapp,uint32 valueapp_discrepancy);//��ȡ���ڹ���
extern sint32 GetPowerFactor(float ratios, uint32 valueapp,uint32 valueapp_discrepancy); //��ȡ��������
extern sint32 GetTemperature(uint32 Temper_App_Value);       //��ȡ�¶�ֵ

extern uint32 Read_CFG_Regester(void);

extern uint32 GetCurrentIvalue(float ratioi, uint32 valueapp, uint32 valueapp_discrepancy);
#endif
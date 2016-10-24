//m25p16.h
//�����ṩ (1)��DSP����������д��m25p16��0������3������������д������ȷ����֤
//         (2)��FPGA(ep3c25)����������д��m25p16��4������31������������д������ȷ����֤
//         (3)��m25p16��4������31�������ݶ���������PSģʽ��FPGA(ep3c25)��������
//         (4)���洢��Ӳ��ĳһĿ¼�µ�FPGA��������(.rbf)�ļ����ݶ���������PSģʽ��FPGA(ep3c25)��������(���ڲ���)
//Yingqiang Zheng
//2008-11-23

//ע�⣺m25p16.h m25p16.c �ı�д���� 2008-10-08 Ͷ��ĵװ�� 2008-11-02 Ͷ����Ӱ�
//      (1)DSP ���� McBSP0 �� M25p16 ���ж�д������M25p16 ��Ƭѡ��ʹ�� DSP �� GPIO4
//      (2)DSP ���� GPIO6 GPIO7 �Լ� McBSP2 �� FSX2 CLKR2 �ĸ�������� FPGA �� PS ����
//      (3)�װ���Ӱ������õ� DSP FPGA M25p16 ��ȫһ������ʹ�õ�����Ҳ��ȫһ��
//      (4)T_DCLK������ FPGA �� PS ���ù����� DCLK ʱ�ӵ������������ T_DCLK = 8 ��Ϊ���ʣ�ѡȡ����һЩ��ֵҲ����
//      (5)m25p16.c �ı�д������Ӳ���޹أ��������صľ��� m25p16.h ���������Ա��޸ġ�������ʱ���ϵ��δʵ��
//         �������Ӳ��ƽ̨�����仯��һ��Ҫ�޸� m25p16.h �еĶ��壬���޸� m25p16.c ����Ӳ����صĲ���
//         ��һ������ͨ���궨��ķ��������һ���⣬����Ӳ����صĲ���ȫ���� m25p16.h �ж���
//      (6)m25p16.c �еĺ����� 2008-11-24 �ֱ��ڵװ���Ӱ��ϲ�����ϣ�������.

//2008-12-28 ���Է��֣�ĿǰFPGA�������̽���������M25P16�ķ��ʽ�����ɵģ���DSP��ʱ�����һ����196.608MHz
//                     ͬʱ��T_DCLK��С��1����Ȼ�������鿴DCLK�źţ�����DCLK�źŴ��ڴ����Ŀ���״̬����Щ״̬
//                     �����ڷ���M25P16������(ҳ���)��������������ٶȵĹؼ��������M25P16�ķ����ٶ�

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <stdio.h>
#include "hal_types.h"
#include "hal_mcu.h"
#include "hal_board_cfg.h"


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//#define T_DCLK		1				//��FPGA��������ʱDCLK�ߵ͵�ƽ�Ŀ��(��DSPʱ��Ϊ��׼)
//#define DCLK		GPIO_PIN7		//��FPGA��������ʱDCLK��Ӧ��DSP����
//#define DATA		GPIO_FSX2		//��FPGA��������ʱDATA0��Ӧ��DSP����
//#define nCONFIG		GPIO_CLKR2		//��FPGA��������ʱnCONFIG��Ӧ��DSP����
//#define nSTATUS		GPIO_PIN6		//��FPGA��������ʱnSTATUS��Ӧ��DSP����(��������з��֣���Ҫ����GPIO8-GPIO15����nSTATUS)
//#define nM25P16_SS	GPIO_PIN4		//��m25p16���ж�дʱm25p16��Ƭѡ�ź�(����DSP�����ⲿ����EEPROMͨ��McBSP0��������ʱ
									//                                   Ĭ�ϲ���GPIO4��Ϊ��оƬ��Ƭѡ�ź�)


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//extern uint32 wrDSPBootFile2M25p16(FILE *fp);		//����д������ֽ���
//extern uint32 writeFPGABootFile2M25p16(FILE *fp);	//����д������ֽ���
//extern uint32 bootFPGAFromFile(FILE *fp);			//��������ʧ�ܷ���0x00000000,������������д��FPGA�ֽ���
//extern uint32 bootFPGAFromM25p16(void);				//M25p16�в�������Ч�������ļ�����0xffffffff,��������ʧ�ܷ���0x00000000

extern unsigned char m25p16_ID[4];			//m25p16 ID  �� 00  20  20  15 

extern void m25p16_readST(void);

extern void m25p16_readID(void);
extern void m25p16_writeEnable(void);
extern void m25p16_beWIP(void);
extern void m25p16_readData(unsigned char source_sector_addr,
							unsigned char source_page_addr,
							unsigned char source_byte_addr,
							uint8* rdBuff,
							unsigned int length);
extern void m25p16_pageProgram(unsigned char dest_sector_addr,
							   unsigned char dest_page_addr,
							   unsigned char dest_byte_addr,
							   uint8* wrBuff,
							   unsigned int length);
extern void m25p16_sectorErase(unsigned char erase_sector_addr,
							   unsigned char erase_page_addr,
							   unsigned char erase_byte_addr);
extern void m25p16_pageErase(unsigned char dest_sector_addr,
						unsigned char dest_page_addr);

extern void m25p16_pageModify(unsigned char modify_sector_addr,
					   unsigned char modify_page_start_addr,
					   unsigned char modify_page_end_addr,
					   uint16* wrBuffWord,
					   uint8*  wrBuffByte,
					   unsigned int length);
extern void m25p16_CAPFModify(uint16 IRCodeIndex,uint8 CAPF);
extern void m25p16_BulkErase(void);


extern uint32 writeFPGAConfInfo2M25p16(uint16 *fp, uint16 InfoLen);
extern void m25p16_EraseAllChip(void);
extern void m25p16_deepPowerDown(void);
extern void m25p16_wakeUP(void);

//It's end of the file.

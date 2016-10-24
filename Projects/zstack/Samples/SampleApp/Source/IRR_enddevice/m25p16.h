//m25p16.h
//用于提供 (1)将DSP引导程序烧写至m25p16的0扇区至3扇区，包括烧写过程正确性验证
//         (2)将FPGA(ep3c25)引导程序烧写至m25p16的4扇区至31扇区，包括烧写过程正确性验证
//         (3)将m25p16的4扇区至31扇区内容读出并采用PS模式将FPGA(ep3c25)引导起来
//         (4)将存储于硬盘某一目录下的FPGA引导程序(.rbf)文件内容读出并采用PS模式将FPGA(ep3c25)引导起来(用于测试)
//Yingqiang Zheng
//2008-11-23

//注意：m25p16.h m25p16.c 的编写基于 2008-10-08 投板的底板和 2008-11-02 投板的子板
//      (1)DSP 采用 McBSP0 对 M25p16 进行读写操作，M25p16 的片选线使用 DSP 的 GPIO4
//      (2)DSP 采用 GPIO6 GPIO7 以及 McBSP2 的 FSX2 CLKR2 四个引脚完成 FPGA 的 PS 配置
//      (3)底板和子板所采用的 DSP FPGA M25p16 完全一样，所使用的引脚也完全一样
//      (4)T_DCLK决定了 FPGA 的 PS 配置过程中 DCLK 时钟的脉宽，试验表明 T_DCLK = 8 较为合适，选取更大一些的值也可以
//      (5)m25p16.c 的编写力争与硬件无关，与配件相关的均在 m25p16.h 中声明，以便修改。但由于时间关系暂未实现
//         所以如果硬件平台发生变化，一定要修改 m25p16.h 中的定义，并修改 m25p16.c 中与硬件相关的部分
//         下一步可以通过宏定义的方法解决这一问题，将与硬件相关的部分全部在 m25p16.h 中定义
//      (6)m25p16.c 中的函数于 2008-11-24 分别在底板和子板上测试完毕，均正常.

//2008-12-28 测试发现：目前FPGA引导过程较慢是由于M25P16的访问较慢造成的，将DSP的时钟提高一倍至196.608MHz
//                     同时将T_DCLK减小至1，仍然较慢，查看DCLK信号，发现DCLK信号存在大量的空闲状态，这些状态
//                     是由于访问M25P16产生的(页面读)，所以提高引导速度的关键在于提高M25P16的访问速度

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <stdio.h>
#include "hal_types.h"
#include "hal_mcu.h"
#include "hal_board_cfg.h"


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//#define T_DCLK		1				//对FPGA进行配置时DCLK高低电平的宽度(以DSP时钟为基准)
//#define DCLK		GPIO_PIN7		//对FPGA进行配置时DCLK对应的DSP引脚
//#define DATA		GPIO_FSX2		//对FPGA进行配置时DATA0对应的DSP引脚
//#define nCONFIG		GPIO_CLKR2		//对FPGA进行配置时nCONFIG对应的DSP引脚
//#define nSTATUS		GPIO_PIN6		//对FPGA进行配置时nSTATUS对应的DSP引脚(试验过程中发现：不要采用GPIO8-GPIO15用作nSTATUS)
//#define nM25P16_SS	GPIO_PIN4		//对m25p16进行读写时m25p16的片选信号(由于DSP采用外部串行EEPROM通过McBSP0进行引导时
									//                                   默认采用GPIO4作为该芯片的片选信号)


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//extern uint32 wrDSPBootFile2M25p16(FILE *fp);		//返回写入错误字节数
//extern uint32 writeFPGABootFile2M25p16(FILE *fp);	//返回写入错误字节数
//extern uint32 bootFPGAFromFile(FILE *fp);			//引导过程失败返回0x00000000,正常操作返回写入FPGA字节数
//extern uint32 bootFPGAFromM25p16(void);				//M25p16中不存在有效的引导文件返回0xffffffff,引导过程失败返回0x00000000

extern unsigned char m25p16_ID[4];			//m25p16 ID  ： 00  20  20  15 

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

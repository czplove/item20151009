//m25p16.c
//32扇区*256页*256字节 = 2M字节
//存储器分配，每条红外编码最大为3072个 字节，共12页。21条编码/扇区
// 21*30= 630个编码， 故m25p16可以存放630条 3072个字节红外编码
//m25p16存储区域分配：
//		第0个扇区为编码区域：存放编码0~20
//				该扇区第0~11页存放编码0，第11页的第254、255字节存放该条编码的长度
//				该扇区第12~23页存放编码1，第23页的第254、255字节存放该条编码的长度
//				该扇区第24~35页存放编码2，第35页的第254、255字节存放该条编码的长度
//				......
//				该扇区第240~251页存放编码20，第251页的第254、255字节存放该条编码的长度
//				其它编码区域扇区(1~29) 编码的存放格式同本扇区，编码区域内编码号取模或取余
//
//		第1个扇区为编码区域：存放编码21~41
//		第2个扇区为编码区域：存放编码42~62
//		第2个扇区为编码区域：存放编码63~83
//		......
//		第29个扇区为编码区域：存放编码609~629
//
//		第30个扇区存放各个编码区域的编程标志，		
//				第0页中第0字节表示编码0~7的编程标志，每位对应一个编码，1为未编程，0为已编程
//				第0页中第1字节表示编码8~15的编程标志，每位对应一个编码，1为未编程，0为已编程
//				第0页中第2字节表示编码16~23的编程标志，每位对应一个编码，1为未编程，0为已编程
//				第0页中第3字节表示编码24~31的编程标志，每位对应一个编码，1为未编程，0为已编程
//				......
//				第0页中第77字节表示编码616~623的编程标志，每位对应一个编码，1为未编程，0为已编程
//				第0页中第78字节表示编码624~629的编程标志，每位对应一个编码，1为未编程，0为已编程
//		第31个扇区作为数据中转扇区，用于修改某个扇区中的部分数据


#include "m25p16.h"

#define	m25p16_select	(P0_4 = 0)	//select m25p16
#define	m25p16_release	(P0_4 = 1)	//release m25p16




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//m25p16的数据存取格式：A23 A22 A21 A20 A19 A18 A17 A16 A15 A14 A13 A12 A11 A10 A9 A8 A7 A6 A5 A4 A3 A2 A1 A0
//扇区号 A23 A22 A21 A20  A19 A18 A17 A16 其中A23 A22 A21固定为0，共32个扇区
//页面号 A15 A14 A13 A12  A11 A10 A9  A8  每个扇区256个page
//字节号 A7  A6  A5  A4   A3  A2  A1  A0  每个page 256个字节
static unsigned char m25p16_ST;				//m25p16状态寄存器
static unsigned char m_dummy_data;			//用于m25p16的虚读操作
//static unsigned char m25p16_wrBuff[256];	//m25p16页面写缓冲区
//static unsigned char m25p16_rdBuff[256];	//m25p16页面读缓冲区
 unsigned char m25p16_ID[4];			//m25p16 ID  ： 00  20  20  15 


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                    M25P16的基本读写访问                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//**********************************************************************************
//enable m25p16 write operation
void m25p16_writeEnable(void)
{
	//while(0 == (U0CSR &(1<<1)));	//直到发送完成
	//U0CSR &= ~(1<<1);				//清楚标志位
	m25p16_select;					//select m25p16

	U0DBUF = 0x06;					//0000 0110b (command)
	while(0 == (U0CSR &(1<<1)));	//直到发送完成
	U0CSR &= ~(1<<1);				//清楚标志位
	m_dummy_data = U0DBUF;
	m25p16_release;					//release m25p16
}


//***********************************************************************************
//read m25p16 identification
void m25p16_readID(void)
{
	//while(0 == (U0CSR &(1<<1)));	//直到发送完成
	//U0CSR &= ~(1<<1);				//清楚标志位
	m25p16_select;					//select m25p16

	U0DBUF = 0x9F;					//read m25p16 device ID (command)
	while(0 == (U0CSR &(1<<1)));	//直到发送完成
	U0CSR &= ~(1<<1);				//清楚标志位
	m25p16_ID[0] = U0DBUF;		//dummy read, High Impedance

	U0DBUF = 0x00;					//dummy write
	while(0 == (U0CSR &(1<<1)));	//直到发送完成
	U0CSR &= ~(1<<1);				//清楚标志位
	m25p16_ID[1] = U0DBUF;		//20h

	U0DBUF = 0x00;					//dummy write
	while(0 == (U0CSR &(1<<1)));	//直到发送完成
	U0CSR &= ~(1<<1);				//清楚标志位
	m25p16_ID[2] = U0DBUF;		//20h

	U0DBUF = 0x00;					//dummy write
	while(0 == (U0CSR &(1<<1)));	//直到发送完成
	U0CSR &= ~(1<<1);				//清楚标志位
	m25p16_ID[3] = U0DBUF;		//15h
	
	m25p16_release;					//release m25p16
}



//************************************************************************************
//read m25p16 status register
void m25p16_readST(void)
{
	m25p16_select;					//select m25p16

	U0DBUF = 0x05;					//0000 0101b (command)
	while(0 == (U0CSR &(1<<1)));	//直到发送完成
	U0CSR &= ~(1<<1);				//清楚标志位
	m_dummy_data = U0DBUF;
	
	U0DBUF = 0x00;					//dummy write
	while(0 == (U0CSR &(1<<1)));	//直到发送完成
	U0CSR &= ~(1<<1);				//清楚标志位
	m25p16_ST = U0DBUF;				//m25p16状态寄存器
	
	m25p16_release;					//release m25p16
}

//*************************************************************************************
//judge m25p16 status register's WIP (write in progress) bit
void m25p16_beWIP(void)
{
	do
	{
		m25p16_readST();
	}while((m25p16_ST & 0x01) == 0x01);			//等待WIP位变为0
}

//**************************************************************************************
//read m25p16 data byte
//source_sector_addr: sector no
//source_page_addr: page no
//source_byte_addr: byte no
//length: 准备读取的数据长度，1 --> 256 (注意：char表示不了256，此处一定要用int)
void m25p16_readData(unsigned char source_sector_addr,
					 unsigned char source_page_addr,
					 unsigned char source_byte_addr,
					 uint8* rdBuff,
					 unsigned int length)
{
	unsigned int m_rdCnt;						//读操作计数器

	m25p16_select;					//select m25p16

	U0DBUF = 0x03;					//0000 0011b (read data bytes command)
	while(0 == (U0CSR &(1<<1)));	//直到发送完成
	U0CSR &= ~(1<<1);				//清楚标志位
	m_dummy_data = U0DBUF;

	U0DBUF = source_sector_addr;				//A23 -- A16 (A23 to A21 can be ignored)
	while(0 == (U0CSR &(1<<1)));	//直到发送完成
	U0CSR &= ~(1<<1);				//清楚标志位
	m_dummy_data = U0DBUF;

	U0DBUF = source_page_addr;					//A15 -- A8
	while(0 == (U0CSR &(1<<1)));	//直到发送完成
	U0CSR &= ~(1<<1);				//清楚标志位
	m_dummy_data = U0DBUF;

	U0DBUF = source_byte_addr;					//A7 -- A0
	while(0 == (U0CSR &(1<<1)));	//直到发送完成
	U0CSR &= ~(1<<1);				//清楚标志位
	m_dummy_data = U0DBUF;

	for(m_rdCnt = 0; m_rdCnt < length; m_rdCnt++)
	{
		U0DBUF = 0x00;						//dummy write
		while(0 == (U0CSR &(1<<1)));		//直到发送完成
		U0CSR &= ~(1<<1);					//清楚标志位
		//rdBuff[m_rdCnt] = U0DBUF;	//store read data bytes
		*rdBuff = U0DBUF;	//store read data bytes
		rdBuff++;
	}

	m25p16_release;							//release m25p16
}

//*********************************************************************************************
//write m25p16 data page (PAGE PROGRAM COMMAND)
//dest_sector_addr: sector no
//dest_page_addr: page no
//dest_byte_addr: byte no
//length: 准备写入的数据长度，1 --> 256
void m25p16_pageProgram(unsigned char dest_sector_addr,
						unsigned char dest_page_addr,
						unsigned char dest_byte_addr,
						uint8* wrBuff,
						unsigned int length)
{
	unsigned int m_wrCnt;						//写操作计数器

	m25p16_select;								//select m25p16

	U0DBUF = 0x02;					//0000 0010b (read data bytes command)
	while(0 == (U0CSR &(1<<1)));	//直到发送完成
	U0CSR &= ~(1<<1);				//清楚标志位
	m_dummy_data = U0DBUF;

	U0DBUF = dest_sector_addr;				//A23 -- A16 (A23 to A21 can be ignored)
	while(0 == (U0CSR &(1<<1)));	//直到发送完成
	U0CSR &= ~(1<<1);				//清楚标志位
	m_dummy_data = U0DBUF;

	U0DBUF = dest_page_addr;					//A15 -- A8
	while(0 == (U0CSR &(1<<1)));	//直到发送完成
	U0CSR &= ~(1<<1);				//清楚标志位
	m_dummy_data = U0DBUF;

	U0DBUF = dest_byte_addr;					//A7 -- A0
	while(0 == (U0CSR &(1<<1)));	//直到发送完成
	U0CSR &= ~(1<<1);				//清楚标志位
	m_dummy_data = U0DBUF;

	for(m_wrCnt = 0; m_wrCnt < length; m_wrCnt++)
	{
		//U0DBUF = wrBuff[m_wrCnt];		//
		U0DBUF = *wrBuff;		//
		wrBuff++;
		while(0 == (U0CSR &(1<<1)));	//直到发送完成
		U0CSR &= ~(1<<1);				//清楚标志位
		m_dummy_data = U0DBUF;
	}

	m25p16_release;							//release m25p16
}

//***********************************************************************************************
//erase a sector in m25p16 (SECTOR ERASE COMMAND)
//在对任意一扇区的某页面或某字节进行写之前，该扇区一定应该是擦除过的，否则是写不正确的
//页面号和字节号最好都是0，仅指定扇区号即可
void m25p16_sectorErase(unsigned char erase_sector_addr,
						unsigned char erase_page_addr,
						unsigned char erase_byte_addr)
{
	m25p16_select;								//select m25p16

	U0DBUF = 0xd8;					//1101 1000b (read data bytes command)
	while(0 == (U0CSR &(1<<1)));	//直到发送完成
	U0CSR &= ~(1<<1);				//清楚标志位
	m_dummy_data = U0DBUF;

	U0DBUF = erase_sector_addr;				//A23 -- A16 (A23 to A21 can be ignored)
	while(0 == (U0CSR &(1<<1)));	//直到发送完成
	U0CSR &= ~(1<<1);				//清楚标志位
	m_dummy_data = U0DBUF;

	U0DBUF = erase_page_addr;					//A15 -- A8
	while(0 == (U0CSR &(1<<1)));	//直到发送完成
	U0CSR &= ~(1<<1);				//清楚标志位
	m_dummy_data = U0DBUF;

	U0DBUF = erase_byte_addr;					//A7 -- A0
	while(0 == (U0CSR &(1<<1)));	//直到发送完成
	U0CSR &= ~(1<<1);				//清楚标志位
	m_dummy_data = U0DBUF;

	m25p16_release;							//release m25p16
}



//***********************************************************************************************
//modify  pages in m25p16 
//对0~29扇区的部分扇区中的数据作修改，利用第31个扇区作为中转。0~29扇区为编码扇区
//modify_sector_addr,要修改的页所在的扇区
//modify_page_start_addr,要修改的起始页
//modify_page_end_addr，要修改的结束页
//wrBuff，新的数据
//length，数据块长度，3个字节一块
void m25p16_pageModify(unsigned char modify_sector_addr,
					   unsigned char modify_page_start_addr,
					   unsigned char modify_page_end_addr,
					   uint16* wrBuffWord,
					   uint8*  wrBuffByte,
					   unsigned int length)
{
	uint16 Cnt;
	uint8 SAddr,PAddr,BAddr;//扇区，页，字节地址
	//uint16 IRCodeIndex = 0;//红外编码的索引0~1023 共1024个红外编码。
	
	//擦除中转扇区
	m25p16_beWIP();
	m25p16_writeEnable();
	m25p16_sectorErase(31,0,0);				//擦除第31扇区
	
	//写编码总数至31扇区，
	PAddr = modify_page_start_addr + 11;//modify_page_start_addr=(IRCodeIndex % 21) * 12
	BAddr = 254;
	m25p16_beWIP();
	m25p16_writeEnable();
	m25p16_pageProgram(31,PAddr,BAddr,(uint8 *)&(length),2);
	
	
	//写更新的数据到中转扇区
	for(Cnt = 0; Cnt < length; Cnt++)//Cnt = edgeCnt
	{
		SAddr = 31;//中转扇区
		PAddr = modify_page_start_addr + (Cnt*3) / 255;//modify_page_start_addr=(IRCodeIndex % 21) * 12
		BAddr = (Cnt*3) % 255;
		
		m25p16_beWIP();
		m25p16_writeEnable();
		m25p16_pageProgram(SAddr,PAddr,BAddr,(uint8 *)&(wrBuffWord[Cnt]),2);
		
		PAddr = modify_page_start_addr + (Cnt*3+2) / 255;
		BAddr = (Cnt*3+2) % 255;
		m25p16_beWIP();
		m25p16_writeEnable();
		m25p16_pageProgram(SAddr,PAddr,BAddr,(uint8 *)&(wrBuffByte[Cnt]),1);
	}
	//转移所更新数据的前半部分至中转扇区
	for(Cnt = 0;Cnt < modify_page_start_addr; Cnt++)//Cnt = PageAddr
	{
		m25p16_beWIP();
		m25p16_readData(modify_sector_addr,Cnt,0,wrBuffByte,256);
		m25p16_beWIP();
		m25p16_writeEnable();
		m25p16_pageProgram(31,Cnt,0,wrBuffByte,256);
	}
	//转移所更新数据的后半部分至中转扇区
	for(Cnt = modify_page_end_addr;Cnt < 256; Cnt++)//Cnt = PageAddr
	{
		m25p16_beWIP();
		m25p16_readData(modify_sector_addr,Cnt,0,wrBuffByte,256);
		m25p16_beWIP();
		m25p16_writeEnable();
		m25p16_pageProgram(31,Cnt,0,wrBuffByte,256);
	}
	//将要修改的页所在扇区擦除
	m25p16_beWIP();
	m25p16_writeEnable();
	m25p16_sectorErase(modify_sector_addr,0,0);				//

	//从中转扇区拷贝数据至修改页所在的扇区
	for(Cnt = 0;Cnt < 256; Cnt++)//Cnt = PageAddr
	{
		m25p16_beWIP();
		m25p16_readData(31,Cnt,0,wrBuffByte,256);
		m25p16_beWIP();
		m25p16_writeEnable();
		m25p16_pageProgram(modify_sector_addr,Cnt,0,wrBuffByte,256);
	}
}



//***********************************************************************************************
//编码区域编程标志修改
//对0~29扇区中的编码区域是否被编码的标志作修改，利用第31个扇区作为中转。
//
//		第30个扇区存放各个编码区域的编程标志，		
//				第0页中第0字节表示编码0~7的编程标志，每位对应一个编码，1为未编程，0为已编程
//				第0页中第1字节表示编码8~15的编程标志，每位对应一个编码，1为未编程，0为已编程
//				第0页中第2字节表示编码16~23的编程标志，每位对应一个编码，1为未编程，0为已编程
//				第0页中第3字节表示编码24~31的编程标志，每位对应一个编码，1为未编程，0为已编程
//				......
//				第0页中第77字节表示编码616~623的编程标志，每位对应一个编码，1为未编程，0为已编程
//				第0页中第78字节表示编码624~629的编程标志，每位对应一个编码，1为未编程，0为已编程
//		第31个扇区作为数据中转扇区，用于修改某个扇区中的部分数据
//IRCodeIndex,要修改的编码号
//CAPF，编码的新编程标志
void m25p16_CAPFModify(uint16 IRCodeIndex,uint8 CAPF)
{
	uint16 Cnt;
	uint8 SAddr,PAddr,BAddr,CAPF_BAddr;//扇区，页，字节地址
	//uint16 IRCodeIndex = 0;//红外编码的索引0~1023 共1024个红外编码。
	
	//擦除中转扇区
	m25p16_beWIP();
	m25p16_writeEnable();
	m25p16_sectorErase(31,0,0);				//擦除第31扇区
	
	
	CAPF_BAddr = IRCodeIndex / 8;
	//写IRCodeIndex的标志位（在CAPF中）到中转扇区
	
	SAddr = 31;	//中转扇区
	PAddr = IRCodeIndex / 8 / 256; 	// 一个IRCodeIndex对应一个bit，故一个页可存放8*256个IRCodeIndex 
	//BAddr = Cnt ;
	m25p16_beWIP();
	m25p16_writeEnable();
	m25p16_pageProgram(SAddr,PAddr,CAPF_BAddr,(uint8 *)&CAPF,1);
	
	
	
	//转移所更新数据的前半部分至中转扇区
	for(Cnt = 0; Cnt < CAPF_BAddr; Cnt++)//Cnt = byte
	{
		SAddr = 31;	//中转扇区
		PAddr = IRCodeIndex / 8 / 256; 	// 一个IRCodeIndex对应一个bit，故一个页可存放8*256个IRCodeIndex 
		//BAddr = Cnt ;
		
		m25p16_beWIP();
		m25p16_readData(30,PAddr,Cnt,(uint8 *)&CAPF,1);
		
		m25p16_beWIP();
		m25p16_writeEnable();
		m25p16_pageProgram(SAddr,PAddr,Cnt,(uint8 *)&CAPF,1);
	}

	//转移所更新数据的后半部分至中转扇区
	for(Cnt = CAPF_BAddr + 1;Cnt < 256; Cnt++)//Cnt = byte
	{
		m25p16_beWIP();
		m25p16_readData(30,PAddr,Cnt,(uint8 *)&CAPF,1);
		
		m25p16_beWIP();
		m25p16_writeEnable();
		m25p16_pageProgram(SAddr,PAddr,Cnt,(uint8 *)&CAPF,1);
	}
	
	//将要修改的页所在扇区擦除
	m25p16_beWIP();
	m25p16_writeEnable();
	m25p16_sectorErase(30,0,0);				//
	
	//从中转扇区拷贝数据至修改页所在的扇区
	for(Cnt = 0;Cnt < 256; Cnt++)//Cnt = byte
	{	
		m25p16_beWIP();
		m25p16_readData(SAddr,PAddr,Cnt,(uint8 *)&CAPF,1);
		m25p16_beWIP();
		m25p16_writeEnable();
		m25p16_pageProgram(30,PAddr,Cnt,(uint8 *)&CAPF,1);
	}
}



void m25p16_BulkErase(void)
{
	m25p16_select;								//select m25p16

	U0DBUF = 0xC7;					//11000111b (Bulk Erase command)
	while(0 == (U0CSR &(1<<1)));	//直到发送完成
	U0CSR &= ~(1<<1);				//清楚标志位
	m_dummy_data = U0DBUF;

	m25p16_release;							//release m25p16
}



void m25p16_EraseAllChip(void)
{
	m25p16_beWIP();//等待m25P16空闲，写入块擦除指令
	m25p16_writeEnable();
	m25p16_BulkErase();
	m25p16_beWIP();//等待空闲，即表示块擦除完成
}

void m25p16_deepPowerDown(void)
{
	//while(0 == (U0CSR &(1<<1)));	//直到发送完成
	//U0CSR &= ~(1<<1);				//清楚标志位
	m25p16_select;					//select m25p16

	U0DBUF = 0xB9;					//1011 1001b (command)
	while(0 == (U0CSR &(1<<1)));	//直到发送完成
	U0CSR &= ~(1<<1);				//清楚标志位
	m_dummy_data = U0DBUF;
	m25p16_release;					//release m25p16
}


void m25p16_wakeUP(void)
{
	//while(0 == (U0CSR &(1<<1)));	//直到发送完成
	//U0CSR &= ~(1<<1);				//清楚标志位
	m25p16_select;					//select m25p16

	U0DBUF = 0xAB;					//1010 1011b (command)
	while(0 == (U0CSR &(1<<1)));	//直到发送完成
	U0CSR &= ~(1<<1);				//清楚标志位
	m_dummy_data = U0DBUF;
	m25p16_release;					//release m25p16
}


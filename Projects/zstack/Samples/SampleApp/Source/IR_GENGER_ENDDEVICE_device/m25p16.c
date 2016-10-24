//m25p16.c
//32����*256ҳ*256�ֽ� = 2M�ֽ�
//�洢�����䣬ÿ������������Ϊ3072�� �ֽڣ���12ҳ��21������/����
// 21*30= 630�����룬 ��m25p16���Դ��630�� 3072���ֽں������
//m25p16�洢������䣺
//		��0������Ϊ�������򣺴�ű���0~20
//				��������0~11ҳ��ű���0����11ҳ�ĵ�254��255�ֽڴ�Ÿ�������ĳ���
//				��������12~23ҳ��ű���1����23ҳ�ĵ�254��255�ֽڴ�Ÿ�������ĳ���
//				��������24~35ҳ��ű���2����35ҳ�ĵ�254��255�ֽڴ�Ÿ�������ĳ���
//				......
//				��������240~251ҳ��ű���20����251ҳ�ĵ�254��255�ֽڴ�Ÿ�������ĳ���
//				����������������(1~29) ����Ĵ�Ÿ�ʽͬ�����������������ڱ����ȡģ��ȡ��
//
//		��1������Ϊ�������򣺴�ű���21~41
//		��2������Ϊ�������򣺴�ű���42~62
//		��2������Ϊ�������򣺴�ű���63~83
//		......
//		��29������Ϊ�������򣺴�ű���609~629
//
//		��30��������Ÿ�����������ı�̱�־��		
//				��0ҳ�е�0�ֽڱ�ʾ����0~7�ı�̱�־��ÿλ��Ӧһ�����룬1Ϊδ��̣�0Ϊ�ѱ��
//				��0ҳ�е�1�ֽڱ�ʾ����8~15�ı�̱�־��ÿλ��Ӧһ�����룬1Ϊδ��̣�0Ϊ�ѱ��
//				��0ҳ�е�2�ֽڱ�ʾ����16~23�ı�̱�־��ÿλ��Ӧһ�����룬1Ϊδ��̣�0Ϊ�ѱ��
//				��0ҳ�е�3�ֽڱ�ʾ����24~31�ı�̱�־��ÿλ��Ӧһ�����룬1Ϊδ��̣�0Ϊ�ѱ��
//				......
//				��0ҳ�е�77�ֽڱ�ʾ����616~623�ı�̱�־��ÿλ��Ӧһ�����룬1Ϊδ��̣�0Ϊ�ѱ��
//				��0ҳ�е�78�ֽڱ�ʾ����624~629�ı�̱�־��ÿλ��Ӧһ�����룬1Ϊδ��̣�0Ϊ�ѱ��
//		��31��������Ϊ������ת�����������޸�ĳ�������еĲ�������


#include "m25p16.h"

#define	m25p16_select	(P0_4 = 0)	//select m25p16
#define	m25p16_release	(P0_4 = 1)	//release m25p16




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//m25p16�����ݴ�ȡ��ʽ��A23 A22 A21 A20 A19 A18 A17 A16 A15 A14 A13 A12 A11 A10 A9 A8 A7 A6 A5 A4 A3 A2 A1 A0
//������ A23 A22 A21 A20  A19 A18 A17 A16 ����A23 A22 A21�̶�Ϊ0����32������
//ҳ��� A15 A14 A13 A12  A11 A10 A9  A8  ÿ������256��page
//�ֽں� A7  A6  A5  A4   A3  A2  A1  A0  ÿ��page 256���ֽ�
static unsigned char m25p16_ST;				//m25p16״̬�Ĵ���
static unsigned char m_dummy_data;			//����m25p16���������
//static unsigned char m25p16_wrBuff[256];	//m25p16ҳ��д������
//static unsigned char m25p16_rdBuff[256];	//m25p16ҳ���������
 unsigned char m25p16_ID[4];			//m25p16 ID  �� 00  20  20  15 


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                    M25P16�Ļ�����д����                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//**********************************************************************************
//enable m25p16 write operation
void m25p16_writeEnable(void)
{
	//while(0 == (U0CSR &(1<<1)));	//ֱ���������
	//U0CSR &= ~(1<<1);				//�����־λ
	m25p16_select;					//select m25p16

	U0DBUF = 0x06;					//0000 0110b (command)
	while(0 == (U0CSR &(1<<1)));	//ֱ���������
	U0CSR &= ~(1<<1);				//�����־λ
	m_dummy_data = U0DBUF;
	m25p16_release;					//release m25p16
}


//***********************************************************************************
//read m25p16 identification
void m25p16_readID(void)
{
	//while(0 == (U0CSR &(1<<1)));	//ֱ���������
	//U0CSR &= ~(1<<1);				//�����־λ
	m25p16_select;					//select m25p16

	U0DBUF = 0x9F;					//read m25p16 device ID (command)
	while(0 == (U0CSR &(1<<1)));	//ֱ���������
	U0CSR &= ~(1<<1);				//�����־λ
	m25p16_ID[0] = U0DBUF;		//dummy read, High Impedance

	U0DBUF = 0x00;					//dummy write
	while(0 == (U0CSR &(1<<1)));	//ֱ���������
	U0CSR &= ~(1<<1);				//�����־λ
	m25p16_ID[1] = U0DBUF;		//20h

	U0DBUF = 0x00;					//dummy write
	while(0 == (U0CSR &(1<<1)));	//ֱ���������
	U0CSR &= ~(1<<1);				//�����־λ
	m25p16_ID[2] = U0DBUF;		//20h

	U0DBUF = 0x00;					//dummy write
	while(0 == (U0CSR &(1<<1)));	//ֱ���������
	U0CSR &= ~(1<<1);				//�����־λ
	m25p16_ID[3] = U0DBUF;		//15h
	
	m25p16_release;					//release m25p16
}



//************************************************************************************
//read m25p16 status register
void m25p16_readST(void)
{
	m25p16_select;					//select m25p16

	U0DBUF = 0x05;					//0000 0101b (command)
	while(0 == (U0CSR &(1<<1)));	//ֱ���������
	U0CSR &= ~(1<<1);				//�����־λ
	m_dummy_data = U0DBUF;
	
	U0DBUF = 0x00;					//dummy write
	while(0 == (U0CSR &(1<<1)));	//ֱ���������
	U0CSR &= ~(1<<1);				//�����־λ
	m25p16_ST = U0DBUF;				//m25p16״̬�Ĵ���
	
	m25p16_release;					//release m25p16
}

//*************************************************************************************
//judge m25p16 status register's WIP (write in progress) bit
void m25p16_beWIP(void)
{
	do
	{
		m25p16_readST();
	}while((m25p16_ST & 0x01) == 0x01);			//�ȴ�WIPλ��Ϊ0
}

//**************************************************************************************
//read m25p16 data byte
//source_sector_addr: sector no
//source_page_addr: page no
//source_byte_addr: byte no
//length: ׼����ȡ�����ݳ��ȣ�1 --> 256 (ע�⣺char��ʾ����256���˴�һ��Ҫ��int)
void m25p16_readData(unsigned char source_sector_addr,
					 unsigned char source_page_addr,
					 unsigned char source_byte_addr,
					 uint8* rdBuff,
					 unsigned int length)
{
	unsigned int m_rdCnt;						//������������

	m25p16_select;					//select m25p16

	U0DBUF = 0x03;					//0000 0011b (read data bytes command)
	while(0 == (U0CSR &(1<<1)));	//ֱ���������
	U0CSR &= ~(1<<1);				//�����־λ
	m_dummy_data = U0DBUF;

	U0DBUF = source_sector_addr;				//A23 -- A16 (A23 to A21 can be ignored)
	while(0 == (U0CSR &(1<<1)));	//ֱ���������
	U0CSR &= ~(1<<1);				//�����־λ
	m_dummy_data = U0DBUF;

	U0DBUF = source_page_addr;					//A15 -- A8
	while(0 == (U0CSR &(1<<1)));	//ֱ���������
	U0CSR &= ~(1<<1);				//�����־λ
	m_dummy_data = U0DBUF;

	U0DBUF = source_byte_addr;					//A7 -- A0
	while(0 == (U0CSR &(1<<1)));	//ֱ���������
	U0CSR &= ~(1<<1);				//�����־λ
	m_dummy_data = U0DBUF;

	for(m_rdCnt = 0; m_rdCnt < length; m_rdCnt++)
	{
		U0DBUF = 0x00;						//dummy write
		while(0 == (U0CSR &(1<<1)));		//ֱ���������
		U0CSR &= ~(1<<1);					//�����־λ
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
//length: ׼��д������ݳ��ȣ�1 --> 256
void m25p16_pageProgram(unsigned char dest_sector_addr,
						unsigned char dest_page_addr,
						unsigned char dest_byte_addr,
						uint8* wrBuff,
						unsigned int length)
{
	unsigned int m_wrCnt;						//д����������

	m25p16_select;								//select m25p16

	U0DBUF = 0x02;					//0000 0010b (read data bytes command)
	while(0 == (U0CSR &(1<<1)));	//ֱ���������
	U0CSR &= ~(1<<1);				//�����־λ
	m_dummy_data = U0DBUF;

	U0DBUF = dest_sector_addr;				//A23 -- A16 (A23 to A21 can be ignored)
	while(0 == (U0CSR &(1<<1)));	//ֱ���������
	U0CSR &= ~(1<<1);				//�����־λ
	m_dummy_data = U0DBUF;

	U0DBUF = dest_page_addr;					//A15 -- A8
	while(0 == (U0CSR &(1<<1)));	//ֱ���������
	U0CSR &= ~(1<<1);				//�����־λ
	m_dummy_data = U0DBUF;

	U0DBUF = dest_byte_addr;					//A7 -- A0
	while(0 == (U0CSR &(1<<1)));	//ֱ���������
	U0CSR &= ~(1<<1);				//�����־λ
	m_dummy_data = U0DBUF;

	for(m_wrCnt = 0; m_wrCnt < length; m_wrCnt++)
	{
		//U0DBUF = wrBuff[m_wrCnt];		//
		U0DBUF = *wrBuff;		//
		wrBuff++;
		while(0 == (U0CSR &(1<<1)));	//ֱ���������
		U0CSR &= ~(1<<1);				//�����־λ
		m_dummy_data = U0DBUF;
	}

	m25p16_release;							//release m25p16
}

//***********************************************************************************************
//erase a sector in m25p16 (SECTOR ERASE COMMAND)
//�ڶ�����һ������ĳҳ���ĳ�ֽڽ���д֮ǰ��������һ��Ӧ���ǲ������ģ�������д����ȷ��
//ҳ��ź��ֽں���ö���0����ָ�������ż���
void m25p16_sectorErase(unsigned char erase_sector_addr,
						unsigned char erase_page_addr,
						unsigned char erase_byte_addr)
{
	m25p16_select;								//select m25p16

	U0DBUF = 0xd8;					//1101 1000b (read data bytes command)
	while(0 == (U0CSR &(1<<1)));	//ֱ���������
	U0CSR &= ~(1<<1);				//�����־λ
	m_dummy_data = U0DBUF;

	U0DBUF = erase_sector_addr;				//A23 -- A16 (A23 to A21 can be ignored)
	while(0 == (U0CSR &(1<<1)));	//ֱ���������
	U0CSR &= ~(1<<1);				//�����־λ
	m_dummy_data = U0DBUF;

	U0DBUF = erase_page_addr;					//A15 -- A8
	while(0 == (U0CSR &(1<<1)));	//ֱ���������
	U0CSR &= ~(1<<1);				//�����־λ
	m_dummy_data = U0DBUF;

	U0DBUF = erase_byte_addr;					//A7 -- A0
	while(0 == (U0CSR &(1<<1)));	//ֱ���������
	U0CSR &= ~(1<<1);				//�����־λ
	m_dummy_data = U0DBUF;

	m25p16_release;							//release m25p16
}



//***********************************************************************************************
//modify  pages in m25p16 
//��0~29�����Ĳ��������е��������޸ģ����õ�31��������Ϊ��ת��0~29����Ϊ��������
//modify_sector_addr,Ҫ�޸ĵ�ҳ���ڵ�����
//modify_page_start_addr,Ҫ�޸ĵ���ʼҳ
//modify_page_end_addr��Ҫ�޸ĵĽ���ҳ
//wrBuff���µ�����
//length�����ݿ鳤�ȣ�3���ֽ�һ��
void m25p16_pageModify(unsigned char modify_sector_addr,
					   unsigned char modify_page_start_addr,
					   unsigned char modify_page_end_addr,
					   uint16* wrBuffWord,
					   uint8*  wrBuffByte,
					   unsigned int length)
{
	uint16 Cnt;
	uint8 SAddr,PAddr,BAddr;//������ҳ���ֽڵ�ַ
	//uint16 IRCodeIndex = 0;//������������0~1023 ��1024��������롣
	
	//������ת����
	m25p16_beWIP();
	m25p16_writeEnable();
	m25p16_sectorErase(31,0,0);				//������31����
	
	//д����������31������
	PAddr = modify_page_start_addr + 11;//modify_page_start_addr=(IRCodeIndex % 21) * 12
	BAddr = 254;
	m25p16_beWIP();
	m25p16_writeEnable();
	m25p16_pageProgram(31,PAddr,BAddr,(uint8 *)&(length),2);
	
	
	//д���µ����ݵ���ת����
	for(Cnt = 0; Cnt < length; Cnt++)//Cnt = edgeCnt
	{
		SAddr = 31;//��ת����
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
	//ת�����������ݵ�ǰ�벿������ת����
	for(Cnt = 0;Cnt < modify_page_start_addr; Cnt++)//Cnt = PageAddr
	{
		m25p16_beWIP();
		m25p16_readData(modify_sector_addr,Cnt,0,wrBuffByte,256);
		m25p16_beWIP();
		m25p16_writeEnable();
		m25p16_pageProgram(31,Cnt,0,wrBuffByte,256);
	}
	//ת�����������ݵĺ�벿������ת����
	for(Cnt = modify_page_end_addr;Cnt < 256; Cnt++)//Cnt = PageAddr
	{
		m25p16_beWIP();
		m25p16_readData(modify_sector_addr,Cnt,0,wrBuffByte,256);
		m25p16_beWIP();
		m25p16_writeEnable();
		m25p16_pageProgram(31,Cnt,0,wrBuffByte,256);
	}
	//��Ҫ�޸ĵ�ҳ������������
	m25p16_beWIP();
	m25p16_writeEnable();
	m25p16_sectorErase(modify_sector_addr,0,0);				//

	//����ת���������������޸�ҳ���ڵ�����
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
//���������̱�־�޸�
//��0~29�����еı��������Ƿ񱻱���ı�־���޸ģ����õ�31��������Ϊ��ת��
//
//		��30��������Ÿ�����������ı�̱�־��		
//				��0ҳ�е�0�ֽڱ�ʾ����0~7�ı�̱�־��ÿλ��Ӧһ�����룬1Ϊδ��̣�0Ϊ�ѱ��
//				��0ҳ�е�1�ֽڱ�ʾ����8~15�ı�̱�־��ÿλ��Ӧһ�����룬1Ϊδ��̣�0Ϊ�ѱ��
//				��0ҳ�е�2�ֽڱ�ʾ����16~23�ı�̱�־��ÿλ��Ӧһ�����룬1Ϊδ��̣�0Ϊ�ѱ��
//				��0ҳ�е�3�ֽڱ�ʾ����24~31�ı�̱�־��ÿλ��Ӧһ�����룬1Ϊδ��̣�0Ϊ�ѱ��
//				......
//				��0ҳ�е�77�ֽڱ�ʾ����616~623�ı�̱�־��ÿλ��Ӧһ�����룬1Ϊδ��̣�0Ϊ�ѱ��
//				��0ҳ�е�78�ֽڱ�ʾ����624~629�ı�̱�־��ÿλ��Ӧһ�����룬1Ϊδ��̣�0Ϊ�ѱ��
//		��31��������Ϊ������ת�����������޸�ĳ�������еĲ�������
//IRCodeIndex,Ҫ�޸ĵı����
//CAPF��������±�̱�־
void m25p16_CAPFModify(uint16 IRCodeIndex,uint8 CAPF)
{
	uint16 Cnt;
	uint8 SAddr,PAddr,BAddr,CAPF_BAddr;//������ҳ���ֽڵ�ַ
	//uint16 IRCodeIndex = 0;//������������0~1023 ��1024��������롣
	
	//������ת����
	m25p16_beWIP();
	m25p16_writeEnable();
	m25p16_sectorErase(31,0,0);				//������31����
	
	
	CAPF_BAddr = IRCodeIndex / 8;
	//дIRCodeIndex�ı�־λ����CAPF�У�����ת����
	
	SAddr = 31;	//��ת����
	PAddr = IRCodeIndex / 8 / 256; 	// һ��IRCodeIndex��Ӧһ��bit����һ��ҳ�ɴ��8*256��IRCodeIndex 
	//BAddr = Cnt ;
	m25p16_beWIP();
	m25p16_writeEnable();
	m25p16_pageProgram(SAddr,PAddr,CAPF_BAddr,(uint8 *)&CAPF,1);
	
	
	
	//ת�����������ݵ�ǰ�벿������ת����
	for(Cnt = 0; Cnt < CAPF_BAddr; Cnt++)//Cnt = byte
	{
		SAddr = 31;	//��ת����
		PAddr = IRCodeIndex / 8 / 256; 	// һ��IRCodeIndex��Ӧһ��bit����һ��ҳ�ɴ��8*256��IRCodeIndex 
		//BAddr = Cnt ;
		
		m25p16_beWIP();
		m25p16_readData(30,PAddr,Cnt,(uint8 *)&CAPF,1);
		
		m25p16_beWIP();
		m25p16_writeEnable();
		m25p16_pageProgram(SAddr,PAddr,Cnt,(uint8 *)&CAPF,1);
	}

	//ת�����������ݵĺ�벿������ת����
	for(Cnt = CAPF_BAddr + 1;Cnt < 256; Cnt++)//Cnt = byte
	{
		m25p16_beWIP();
		m25p16_readData(30,PAddr,Cnt,(uint8 *)&CAPF,1);
		
		m25p16_beWIP();
		m25p16_writeEnable();
		m25p16_pageProgram(SAddr,PAddr,Cnt,(uint8 *)&CAPF,1);
	}
	
	//��Ҫ�޸ĵ�ҳ������������
	m25p16_beWIP();
	m25p16_writeEnable();
	m25p16_sectorErase(30,0,0);				//
	
	//����ת���������������޸�ҳ���ڵ�����
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
	while(0 == (U0CSR &(1<<1)));	//ֱ���������
	U0CSR &= ~(1<<1);				//�����־λ
	m_dummy_data = U0DBUF;

	m25p16_release;							//release m25p16
}



void m25p16_EraseAllChip(void)
{
	m25p16_beWIP();//�ȴ�m25P16���У�д������ָ��
	m25p16_writeEnable();
	m25p16_BulkErase();
	m25p16_beWIP();//�ȴ����У�����ʾ��������
}

void m25p16_deepPowerDown(void)
{
	//while(0 == (U0CSR &(1<<1)));	//ֱ���������
	//U0CSR &= ~(1<<1);				//�����־λ
	m25p16_select;					//select m25p16

	U0DBUF = 0xB9;					//1011 1001b (command)
	while(0 == (U0CSR &(1<<1)));	//ֱ���������
	U0CSR &= ~(1<<1);				//�����־λ
	m_dummy_data = U0DBUF;
	m25p16_release;					//release m25p16
}


void m25p16_wakeUP(void)
{
	//while(0 == (U0CSR &(1<<1)));	//ֱ���������
	//U0CSR &= ~(1<<1);				//�����־λ
	m25p16_select;					//select m25p16

	U0DBUF = 0xAB;					//1010 1011b (command)
	while(0 == (U0CSR &(1<<1)));	//ֱ���������
	U0CSR &= ~(1<<1);				//�����־λ
	m_dummy_data = U0DBUF;
	m25p16_release;					//release m25p16
}


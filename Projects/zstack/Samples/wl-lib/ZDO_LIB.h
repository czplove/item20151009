


extern uint8 RTR_NWK_FLAG ;        //��ʾ�궨���ﶨ����RTR_NWK
 extern  uint8 POWER_SAVING_FLAG ;   //��ʾ�궨���ﶨ����POWER_SAVING
 extern uint8 POWER_PA_FLAG ;       //��ʾ�궨���ﶨ����POWER_PA
 extern  uint8 LEAVE_NET_SET_FLAG;       //������������������� ��ֱ��������������,��λ����ֱ�Ӹ�Ԥ����

 extern uint8 OS_FLAG;
 
 
extern uint8 SampleApp_TransID;  // This is the unique message ID (counter)
extern uint8 SampleApp_TaskID;
extern endPointDesc_t SampleApp_epDesc;

extern  uint8 *shortddr_mem;  
extern  uint8 *extendaddr_mem;

extern devStartModes_t devStartMode;
extern uint8 zdoDiscCounter;
extern uint8 SampleApp_TaskID;
extern uint8 Heart_mesg_flag;

extern afAddrType_t SampleApp_Addr_SendData_DstAddr;  //��������
extern afAddrType_t SampleApp_Periodic_DstAddr;       //�����¼�Ѱַģʽ
extern afAddrType_t SampleApp_SPI_SendCommand_DstAddr;
extern afAddrType_t SampleApp_Flash_DstAddr;
 
 
//extern  CONST uint8 defaultKey[SEC_KEY_LEN];
 

 extern void ZDO_DEVICE_DISC(void);
 extern networkDesc_t* ZDApp_NwkDescListProcessing(void);

 

 
 //APP��ʼ����ʱ������ѡ���ӳ���������ֱ��������
 extern void APP_JOIN_INIT(void);
 
 // �������������Ҫ����������ǵ�һ��������Ӧ��3���ӣ���������OS �� ������������
extern void APP_JOIN_DEAL_WITH(void);
// ����NV������������
extern void APP_JOIN_START_EVT(void);

extern void ZDO_REJOIN_ERROR(void);
extern void ZDO_ACK_DEAL_WITH(void);

extern void SampleApp_SendNwkaddrMessage(uint8 type1_id,uint8 type2_id);

extern void APPLY_TO_JOIN_OF_KEY(void);

extern void RESTORE_TO_FACTORY(void);

extern void SampleApp_SendPeriodicMessage( void );

extern uint8 zb_ReadConfiguration( uint8 configId, uint8 len, void *pValue );

extern uint8 zb_WriteConfiguration( uint8 configId, uint8 len, void *pValue );

extern uint8 *SampleApp_GetExtendAddr(void);
extern uint8 *SampleApp_GetShortAddr(void);


extern uint8 hextoword1(uint8 t );

extern uint8 hextoword2(uint8 t);

extern uint8 wordtohex(uint8 x,uint8 y);

extern void APP_NORMAL_COMMAND(uint8 *command_str);

 extern void zb_StartRequest(void);

extern void APP_DEAL_WITH_ASSOCIATED(void);
extern void SampleApp_ProcessNwkaddrMessage(afIncomingMSGPacket_t *pkt);
extern uint8 RemoveStaleNode(uint8 index);
extern uint8 RemoveStaleRouter(uint8 index);
extern void SampleApp_ROUTER_ProcessPeriodicMessage_Data(afIncomingMSGPacket_t *pkt);
extern void APP_OS_DEAL_WITH(uint16 poll_rate_value);
uint8 APP_ENDDEVICE_ACK(uint8 a,uint8 b,uint8 c);
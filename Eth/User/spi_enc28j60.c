/*************************************************************************
 * оǶstm32ʹ��spi����ENC28J60оƬ�������ڣ����ĵ���Ϊ��ײ�����
 * ��Ϊ��������: spi���ó�ʼ����spi��д���Լ�ENC28J60��ʼ��������
 * ���ҷֿ�ѧϰ:spi���֡�ENC28J60����
 * 
 * оǶstm32 @2013-7-3
 *************************************************************************/
#include "stm32f10x_spi.h"
#include "spi_enc28j60.h"

#define ENC28J60_CS_SET()		GPIO_SetBits(GPIOB, GPIO_Pin_10);	//����CS��
#define ENC28J60_CS_CLR()		GPIO_ResetBits(GPIOB, GPIO_Pin_10);	//����CS��
#define ENC28J60_RST_SET() 		GPIO_SetBits(GPIOB, GPIO_Pin_11);	//���߸�λ����
#define ENC28J60_RST_CLR() 		GPIO_ResetBits(GPIOB, GPIO_Pin_11); //���㸴λ����

static unsigned char Enc28j60Bank;
static unsigned int NextPacketPtr;
extern void  Delay (u32 nCount);

//SPI2��Ӧ��IO�����ó�ʼ��
void SPI2_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );	  //SPI2ֻ�õ�GPIOB���IO��

	// PB13 -> SPI2_SCK		PB14 -> SPI2_MISO		PB15 -> SPI2_MOSI
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //����������� 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB,GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);

    // PB10 -> SPI2_NSS  	PB11 -> RST		PB12 -> INT                  						
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   // �������
	GPIO_Init(GPIOB, &GPIO_InitStructure);        				  
    GPIO_SetBits(GPIOB, GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12);                  // �Ȱ�Ƭѡ���ߣ�����ʱ����
}


/*****************************************************************************
 * SPI2������ʼ������
 * �ṹ��SPI_InitTypeDefһ�������³�Ա��ÿ����Ա��Ҫ����
 * uint16_t SPI_Direction������������˫��˫��ȫ˫���������ǵ���˫��һ��ѡ��ǰ�ߣ� 
 * uint16_t SPI_Mode��������SPI������ģʽ��stm32һ��Ϊ����������Ϊ���豸slave              
 * uint16_t SPI_DataSize��������ÿ��������8λ����16λ��          
 * uint16_t SPI_CPOL�������� SPIʱ��CLK�ļ��ԣ�û�����ݴ���ʱʱ�ӵĿ���״̬��ƽ               
 * uint16_t SPI_CPHA�������� SCKʱ�ӵĵ�1/2�����ؽ�������λ�Ĳ����������ڵ�1/2��ʱ�ӱ��ر����档               
 * uint16_t SPI_NSS �������� NSS��Ӳ�������������                 
 * uint16_t SPI_BaudRatePrescaler�������� ������Ԥ��Ƶ��ֵ������SPI����ʱ��������ʱ�ӵĿ��ƣ�   
 * uint16_t SPI_FirstBit �������� �ȴ��͵��Ǹ�λMSB ���ǵ�λLSB            
 * uint16_t SPI_CRCPolynomial �������� CRC����ʽ
       
 * оǶstm32 @2013-7-3
 ************************************************************************/
void SPI2_Config(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
	RCC_APB1PeriphClockCmd(	RCC_APB1Periph_SPI2,  ENABLE );

 	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;	
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;	
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	
	SPI_InitStructure.SPI_CRCPolynomial = 7;	
	SPI_Init(SPI2, &SPI_InitStructure);  
 
	SPI_Cmd(SPI2, ENABLE); 
}

/***************************************************************
 * ����SPI���ߵĻ��νṹ��ֱ�ӵ������д�����ǲ��ֿ��ġ�
 * �����Ƕ�����д������stm32����һ������������ݣ�
 * ���ǻ��ڽ��ն������յ����豸���ص����ݡ�
 * ����Ƕ������������������룬��Ӧ��Ҫ���������ݾͻ���λ�����ڽ��ն�����
 * �����д�������������룬���Ը��ݽ��ջ���Խ��ն����ŵ�����
 *
 * stm32�ڲ�������
 * ����ʱ���ǰ����ݷ������ݼĴ���DR�Ӳ���Զ��������ͳ�ȥ
 * ����ʱ��ֻҪ�����ݼĴ���DR���ȡ���ɣ�����Ӳ���������
 * ע�⣬�Ƿ��ڷ��ͣ���TXE��־����DR�Ĵ���ʱ��RXNE��־�����
 * 
 * оǶstm32 @2013-7-3
 ***************************************************************/
unsigned char SPI2_Read_Write_Byte(unsigned char TxData)			//����SPI2��ʼ���У����õ��շ����ݸ�ʽΪ8λ�������unsigned char
 {
    while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(SPI2, TxData);
    
    /* Wait to receive a byte */
    while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
    return SPI_I2S_ReceiveData(SPI2);
 }


void SPI2_Init(void)
{

	SPI2_GPIO_Init();
	SPI2_Config();	 
//	i =0; 
}


unsigned char enc28j60ReadOp(unsigned char op, unsigned char address)
{
    unsigned char dat = 0;
    
    ENC28J60_CS_CLR();
    
    dat = op | (address & ADDR_MASK);
    SPI2_Read_Write_Byte(dat);
    dat = SPI2_Read_Write_Byte(0xFF);
    // do dummy read if needed (for mac and mii, see datasheet page 29)
    if(address & 0x80)
    {
        dat = SPI2_Read_Write_Byte(0xFF);
    }
    // release CS
    ENC28J60_CS_SET();
    return dat;
}

void enc28j60WriteOp(unsigned char op, unsigned char address, unsigned char data)
{
    unsigned char dat = 0;
    
    ENC28J60_CS_CLR();
    // issue write command
    dat = op | (address & ADDR_MASK);
    SPI2_Read_Write_Byte(dat);
    // write data
    dat = data;
    SPI2_Read_Write_Byte(dat);
    ENC28J60_CS_SET();
}

void enc28j60ReadBuffer(unsigned int len, unsigned char* data)
{
    ENC28J60_CS_CLR();
    // issue read command
    SPI2_Read_Write_Byte(ENC28J60_READ_BUF_MEM);
    while(len)
    {
        len--;
        // read data
        *data = (unsigned char)SPI2_Read_Write_Byte(0);
        data++;
    }
    *data='\0';
    ENC28J60_CS_SET();
}

void enc28j60WriteBuffer(unsigned int len, unsigned char* data)
{
    ENC28J60_CS_CLR();
    // issue write command
    SPI2_Read_Write_Byte(ENC28J60_WRITE_BUF_MEM);
    
    while(len)
    {
        len--;
        SPI2_Read_Write_Byte(*data);
        data++;
    }
    ENC28J60_CS_SET();
}

void enc28j60SetBank(unsigned char address)
{
    // set the bank (if needed)
    if((address & BANK_MASK) != Enc28j60Bank)
    {
        // set the bank
        enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, (ECON1_BSEL1|ECON1_BSEL0));
        enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, (address & BANK_MASK)>>5);
        Enc28j60Bank = (address & BANK_MASK);
    }
}

unsigned char enc28j60Read(unsigned char address)
{
    // set the bank
    enc28j60SetBank(address);
    // do the read
    return enc28j60ReadOp(ENC28J60_READ_CTRL_REG, address);
}

void enc28j60Write(unsigned char address, unsigned char data)
{
    // set the bank
    enc28j60SetBank(address);
    // do the write
    enc28j60WriteOp(ENC28J60_WRITE_CTRL_REG, address, data);
}

void ENC28J60_PHY_Write(unsigned char address, unsigned int data)
{
    // set the PHY register address
    enc28j60Write(MIREGADR, address);
    // write the PHY data
    enc28j60Write(MIWRL, data);
    enc28j60Write(MIWRH, data>>8);
    // wait until the PHY write completes
    while(enc28j60Read(MISTAT) & MISTAT_BUSY)
    {
        //Del_10us(1);
        //_nop_();
    }
}

void enc28j60clkout(unsigned char clk)
{
    //setup clkout: 2 is 12.5MHz:
    enc28j60Write(ECOCON, clk & 0x7);
}

unsigned char ENC28J60_Init(unsigned char* macaddr)
{
	  
    ENC28J60_CS_SET();       
	ENC28J60_RST_CLR();			//��λENC28J60
	Delay(0xfffff);	
//	Delay(0xfffff);
	ENC28J60_RST_SET();			//��λ����				    
	

    // perform system reset
    enc28j60WriteOp(ENC28J60_SOFT_RESET, 0, ENC28J60_SOFT_RESET);
   	enc28j60Write(ECOCON,0x01);//100,���12.5Mʱ��
    // check CLKRDY bit to see if reset is complete
    // The CLKRDY does not work. See Rev. B4 Silicon Errata point. Just wait.
    while(!(enc28j60Read(ESTAT) & ESTAT_CLKRDY));

    // do bank 0 stuff									
    // initialize receive buffer
    // 16-bit transfers, must write low byte first
    // set receive buffer start address    
    NextPacketPtr = RXSTART_INIT;
    // Rx start    
    enc28j60Write(ERXSTL, RXSTART_INIT&0xFF);    
    enc28j60Write(ERXSTH, RXSTART_INIT>>8);
    // set receive pointer address     
    enc28j60Write(ERXRDPTL, RXSTART_INIT&0xFF);
    enc28j60Write(ERXRDPTH, RXSTART_INIT>>8);
    // RX end
    enc28j60Write(ERXNDL, RXSTOP_INIT&0xFF);
    enc28j60Write(ERXNDH, RXSTOP_INIT>>8);
    // TX start   1500
    enc28j60Write(ETXSTL, TXSTART_INIT&0xFF);
    enc28j60Write(ETXSTH, TXSTART_INIT>>8);
    // TX end
    enc28j60Write(ETXNDL, TXSTOP_INIT&0xFF);
    enc28j60Write(ETXNDH, TXSTOP_INIT>>8);
    // do bank 1 stuff, packet filter:
    // For broadcast packets we allow only ARP packtets
    // All other packets should be unicast only for our mac (MAADR)
    //
    // The pattern to match on is therefore
    // Type     ETH.DST
    // ARP      BROADCAST
    // 06 08 -- ff ff ff ff ff ff -> ip checksum for theses bytes=f7f9
    // in binary these poitions are:11 0000 0011 1111
    // This is hex 303F->EPMM0=0x3f,EPMM1=0x30
    
    enc28j60Write(ERXFCON, ERXFCON_UCEN|ERXFCON_CRCEN|ERXFCON_PMEN);
    enc28j60Write(EPMM0, 0x3f);
    enc28j60Write(EPMM1, 0x30);
    enc28j60Write(EPMCSL, 0xf9);
    enc28j60Write(EPMCSH, 0xf7);    
    enc28j60Write(MACON1, MACON1_MARXEN|MACON1_TXPAUS|MACON1_RXPAUS);
    // bring MAC out of reset 
    enc28j60Write(MACON2, 0x00);
    
    enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, MACON3, MACON3_PADCFG0|MACON3_TXCRCEN|MACON3_FRMLNEN|MACON3_FULDPX);
    // set inter-frame gap (non-back-to-back)

    enc28j60Write(MAIPGL, 0x12);
    enc28j60Write(MAIPGH, 0x0C);
    // set inter-frame gap (back-to-back)

    enc28j60Write(MABBIPG, 0x15);
    // Set the maximum packet size which the controller will accept
    // Do not send packets longer than MAX_FRAMELEN:
  
    enc28j60Write(MAMXFLL, MAX_FRAMELEN&0xFF);  
    enc28j60Write(MAMXFLH, MAX_FRAMELEN>>8);
    // do bank 3 stuff
    // write MAC address
    // NOTE: MAC address in ENC28J60 is byte-backward
    enc28j60Write(MAADR5, macaddr[0]);  
    enc28j60Write(MAADR4, macaddr[1]);
    enc28j60Write(MAADR3, macaddr[2]);
    enc28j60Write(MAADR2, macaddr[3]);
    enc28j60Write(MAADR1, macaddr[4]);
    enc28j60Write(MAADR0, macaddr[5]);

    //����PHYΪȫ˫��  LEDBΪ������
    ENC28J60_PHY_Write(PHCON1, PHCON1_PDPXMD);    
    
    // no loopback of transmitted frames
    ENC28J60_PHY_Write(PHCON2, PHCON2_HDLDIS);

    // switch to bank 0    
    enc28j60SetBank(ECON1);

    // enable interrutps
    enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, EIE, EIE_INTIE|EIE_PKTIE);

    // enable packet reception
    enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_RXEN);
	
	return 0;
}

// read the revision of the chip:
unsigned char enc28j60getrev(void)
{
    //��EREVID ��Ҳ�洢�˰汾��Ϣ�� EREVID ��һ��ֻ����
    //�ƼĴ���������һ��5 λ��ʶ����������ʶ�����ض���Ƭ
    //�İ汾��
    return(enc28j60Read(EREVID));
}

void ENC28J60_Packet_Send(unsigned int len, unsigned char* packet)
{
    // Set the write pointer to start of transmit buffer area
    enc28j60Write(EWRPTL, TXSTART_INIT&0xFF);
    enc28j60Write(EWRPTH, TXSTART_INIT>>8);
    
    // Set the TXND pointer to correspond to the packet size given
    enc28j60Write(ETXNDL, (TXSTART_INIT+len)&0xFF);
    enc28j60Write(ETXNDH, (TXSTART_INIT+len)>>8);
    
    // write per-packet control byte (0x00 means use macon3 settings)
    enc28j60WriteOp(ENC28J60_WRITE_BUF_MEM, 0, 0x00);
    
    // copy the packet into the transmit buffer
    enc28j60WriteBuffer(len, packet);
    
    // send the contents of the transmit buffer onto the network
    enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRTS);
    
    // Reset the transmit logic problem. See Rev. B4 Silicon Errata point 12.
    if( (enc28j60Read(EIR) & EIR_TXERIF) )
    {
        enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_TXRTS);
    }
}

// Gets a packet from the network receive buffer, if one is available.
// The packet will by headed by an ethernet header.
//      maxlen  The maximum acceptable length of a retrieved packet.
//      packet  Pointer where packet data should be stored.
// Returns: Packet length in bytes if a packet was retrieved, zero otherwise.
unsigned int ENC28J60_Packet_Receive(unsigned int maxlen, unsigned char* packet)
{
    unsigned int rxstat;
    unsigned int len;
    
    // check if a packet has been received and buffered
    //if( !(enc28j60Read(EIR) & EIR_PKTIF) ){
    // The above does not work. See Rev. B4 Silicon Errata point 6.
    if( enc28j60Read(EPKTCNT) ==0 )  //�յ�����̫�����ݰ�����
    {
        return(0);
    }
    
    // Set the read pointer to the start of the received packet      ��������ָ��
    enc28j60Write(ERDPTL, (NextPacketPtr));
    enc28j60Write(ERDPTH, (NextPacketPtr)>>8);
    
    // read the next packet pointer
    NextPacketPtr  = enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0);
    NextPacketPtr |= enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0)<<8;
    
    // read the packet length (see datasheet page 43)
    len  = enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0);
    len |= enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0)<<8;
    
    len-=4; //remove the CRC count
    // read the receive status (see datasheet page 43)
    rxstat  = enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0);
    rxstat |= enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0)<<8;
    // limit retrieve length
    if (len>maxlen-1)
    {
        len=maxlen-1;
    }
    
    // check CRC and symbol errors (see datasheet page 44, table 7-3):
    // The ERXFCON.CRCEN is set by default. Normally we should not
    // need to check this.
    if ((rxstat & 0x80)==0)
    {
        // invalid
        len=0;
    }
    else
    {
        // copy the packet from the receive buffer
        enc28j60ReadBuffer(len, packet);
    }
    // Move the RX read pointer to the start of the next received packet
    // This frees the memory we just read out
    enc28j60Write(ERXRDPTL, (NextPacketPtr));
    enc28j60Write(ERXRDPTH, (NextPacketPtr)>>8);
    
    // decrement the packet counter indicate we are done with this packet
    enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON2, ECON2_PKTDEC);
    return(len);
}



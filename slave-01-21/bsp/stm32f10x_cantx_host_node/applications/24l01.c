#include <rtthread.h>
#include "stm32f10x.h"
#include "24l01.h"
#include "spi.h"
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//NRF24L01��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/13
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////

const uint8_t TX_ADDRESS[TX_ADR_WIDTH]={0x11,0x11,0x11,0x11,0x11}; //���͵�ַ
const uint8_t RX_ADDRESS[RX_ADR_WIDTH]={0x11,0x11,0x11,0x11,0x11};
uint8_t tmp_buf[33],nrf_flag;	//tmp_bufΪ���ݴ洢����nrf_flagΪnrf24L01״̬�Ĵ��������

/* ������ƿ� */

static struct rt_messagequeue mq;
static char   msg_pool[1024];
static struct rt_semaphore sem_nrf_rx,sem_nrf_tx;
typedef struct nrf_tx_msg
{
	uint8_t buf[33];
	uint8_t len;
}nrf_tx_msg;

typedef struct nrf_rx_msg
{
	uint8_t buf[33];
	uint8_t len;
}nrf_rx_msg;

static nrf_tx_msg tx_msg;
static nrf_rx_msg rx_msg;

void rt_hw_us_delay(int us)
{
    rt_uint32_t delta;
    rt_uint32_t current_delay;
	  /*�ر������ж�*/
    INTX_DISABLE();
    /*�����ʱ������tick*/
    us = us * (SysTick->LOAD/(1000000/RT_TICK_PER_SECOND));
    /*��õ�ǰʱ��*/
    delta = SysTick->VAL;  
    /*ѭ����õ�ǰʱ�䣬ֱ���ﵽʱ����˳�*/
    do
    {
        if ( delta > SysTick->VAL )
            current_delay = delta - SysTick->VAL;
        else
        /*��ʱ��Խ��һ��os tick�ı߽�ʱ*/
            current_delay = SysTick->LOAD + delta - SysTick->VAL;
    } while( current_delay < us );
		/*���������ж�*/
		INTX_DISABLE();
}

//��ʼ��24L01��IO��
void NRF24L01_Init(void)
{ 
	GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//�ⲿ�жϣ���Ҫʹ��AFIOʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_3;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //PA3 �ж�����  
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_ResetBits(GPIOA,GPIO_Pin_1|GPIO_Pin_2);//PA1,2����	
	//nRF24L01 NVIC����
	NVIC_InitStructure.NVIC_IRQChannel=EXTI3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	//��GPIO�ܽ����ⲿ�ж�������
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource3);
	//nRF24L01 EXIT����
	EXTI_InitStructure.EXTI_Line=EXTI_Line3;
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	EXTI_ClearITPendingBit(EXTI_Line3);
		 
  SPI2_Init();    		    //��ʼ��SPI	 
	SPI_Cmd(SPI2, DISABLE); // SPI���費ʹ��
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //SPI����Ϊ˫��˫��ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//SPI����
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//���ͽ���8λ֡�ṹ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		//ʱ�����յ�
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	//���ݲ����ڵ�1��ʱ����
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS�ź����������
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;		//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ16
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRCֵ����Ķ���ʽ
	SPI_Init(SPI2, &SPI_InitStructure);  //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���
	
	SPI_Cmd(SPI2, ENABLE); // ʹ��SPI���� 
	NRF24L01_CE=0; 				 // ʹ��24L01
	NRF24L01_CSN=1;				 // SPIƬѡȡ��  
	 		 	 
}
//���24L01�Ƿ����
//����ֵ:0���ɹ�;1��ʧ��	
uint8_t NRF24L01_Check(void)
{
	uint8_t buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	uint8_t i;
	SPI2_SetSpeed(SPI_BaudRatePrescaler_8); //spi�ٶ�Ϊ9Mhz��24L01�����SPIʱ��Ϊ10Mhz��   	 
	NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,buf,5);//д��5���ֽڵĵ�ַ.	
	NRF24L01_Read_Buf(TX_ADDR,buf,5); //����д��ĵ�ַ  
	for(i=0;i<5;i++)if(buf[i]!=0XA5)break;	 							   
	if(i!=5)return 1;//���24L01����	
	return 0;		 //��⵽24L01
}	 	 
//SPIд�Ĵ���
//reg:ָ���Ĵ�����ַ
//value:д���ֵ
uint8_t NRF24L01_Write_Reg(uint8_t reg,uint8_t value)
{
	uint8_t status;	
	NRF24L01_CSN=0;                 //ʹ��SPI����
	status =SPI2_ReadWriteByte(reg);//���ͼĴ����� 
	SPI2_ReadWriteByte(value);      //д��Ĵ�����ֵ
	NRF24L01_CSN=1;                 //��ֹSPI����	   
	return(status);       			    //����״ֵ̬
}
//��ȡSPI�Ĵ���ֵ
//reg:Ҫ���ļĴ���
uint8_t NRF24L01_Read_Reg(uint8_t reg)
{
	uint8_t reg_val;	    
 	NRF24L01_CSN = 0;          //ʹ��SPI����		
	SPI2_ReadWriteByte(reg);   //���ͼĴ�����
	reg_val=SPI2_ReadWriteByte(0XFF);//��ȡ�Ĵ�������
	NRF24L01_CSN = 1;          //��ֹSPI����		    
	return(reg_val);           //����״ֵ̬
}	
//��ָ��λ�ö���ָ�����ȵ�����
//reg:�Ĵ���(λ��)
//*pBuf:����ָ��
//len:���ݳ���
//����ֵ,�˴ζ�����״̬�Ĵ���ֵ 
uint8_t NRF24L01_Read_Buf(uint8_t reg,uint8_t *pBuf,uint8_t len)
{
	uint8_t status,uint8_t_ctr;	       
	NRF24L01_CSN = 0;           //ʹ��SPI����
	status=SPI2_ReadWriteByte(reg);//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬   	   
	for(uint8_t_ctr=0;uint8_t_ctr<len;uint8_t_ctr++)pBuf[uint8_t_ctr]=SPI2_ReadWriteByte(0XFF);//��������
	NRF24L01_CSN=1;       //�ر�SPI����
	return status;        //���ض�����״ֵ̬
}
//��ָ��λ��дָ�����ȵ�����
//reg:�Ĵ���(λ��)
//*pBuf:����ָ��
//len:���ݳ���
//����ֵ,�˴ζ�����״̬�Ĵ���ֵ
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
	uint8_t status,uint8_t_ctr;	    
 	NRF24L01_CSN = 0;          //ʹ��SPI����
	status = SPI2_ReadWriteByte(reg);//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
	for(uint8_t_ctr=0; uint8_t_ctr<len; uint8_t_ctr++)SPI2_ReadWriteByte(*pBuf++); //д������	 
	NRF24L01_CSN = 1;       //�ر�SPI����
	return status;          //���ض�����״ֵ̬
}				   
//����NRF24L01����һ������
//txbuf:�����������׵�ַ
//����ֵ:�������״��
uint8_t NRF24L01_TxPacket(nrf_tx_msg msg)
{
	uint8_t *txbuf = msg.buf;
 	SPI2_SetSpeed(SPI_BaudRatePrescaler_8);//spi�ٶ�Ϊ9Mhz��24L01�����SPIʱ��Ϊ10Mhz��   
	NRF24L01_CE=0;
  NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//д���ݵ�TX BUF  32���ֽ�
 	NRF24L01_CE=1;//��������	   

	rt_sem_take(&sem_nrf_tx, RT_TICK_PER_SECOND /10);
	
	if(nrf_flag & MAX_TX)//�ﵽ����ط�����
	{
		NRF24L01_Write_Reg(FLUSH_TX,0xff);//���TX_FIFO�Ĵ���
		nrf_flag = 0;						//������
		return MAX_TX; 
	}
	if(nrf_flag&TX_OK)//�������
	{
		nrf_flag = 0;			//������
		return TX_OK;
	}
	nrf_flag=0;					//������
	return 0xff;//����ʧ��
}

//����NRF24L01����һ������
//txbuf:�����������׵�ַ
//����ֵ:0��������ɣ��������������
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
{
	uint8_t sta;		    							   
	SPI2_SetSpeed(SPI_BaudRatePrescaler_8); //spi�ٶ�Ϊ9Mhz��24L01�����SPIʱ��Ϊ10Mhz��   
	sta=NRF24L01_Read_Reg(STATUS);  //��ȡ״̬�Ĵ�����ֵ    	 
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //���TX_DS��MAX_RT�жϱ�־
	if(sta&RX_OK)//���յ�����
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//��ȡ����
		NRF24L01_Write_Reg(FLUSH_RX,0xff);//���RX FIFO�Ĵ��� 
		return 0; 
	}	   
	return 1;//û�յ��κ�����
}		

//�ú�����ʼ��NRF24L01��RXģʽ
//����RX��ַ,дRX���ݿ��,ѡ��RFƵ��,�����ʺ�LNA HCURR
//��CE��ߺ�,������RXģʽ,�����Խ���������		   
void NRF24L01_RX_Mode(void)
{
	NRF24L01_CE=0;	  
	NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t *)RX_ADDRESS,RX_ADR_WIDTH);//дRX�ڵ��ַ
	
	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);    //ʹ��ͨ��0���Զ�Ӧ��    
	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);//ʹ��ͨ��0�Ľ��յ�ַ  	 
	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,40);	     //����RFͨ��Ƶ��		
	NRF24L01_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//ѡ��ͨ��0����Ч���ݿ�� 	    
	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);//����TX�������,0db����,2Mbps,���������濪��   
	NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG, 0x0f);//���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ
	SPI2_SetSpeed(SPI_BaudRatePrescaler_8); //spi�ٶ�Ϊ9Mhz��24L01�����SPIʱ��Ϊ10Mhz�� 
	NRF24L01_Write_Reg(FLUSH_RX,0xff);//���RX FIFO�Ĵ��� 
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,0xff);	//***һ��Ҫ���״̬�Ĵ�������������**
  NRF24L01_CE = 1; //CEΪ��,�������ģʽ 
}						 
//�ú�����ʼ��NRF24L01��TXģʽ
//����TX��ַ,дTX���ݿ��,����RX�Զ�Ӧ��ĵ�ַ,���TX��������,ѡ��RFƵ��,�����ʺ�LNA HCURR
//PWR_UP,CRCʹ��
//��CE��ߺ�,������RXģʽ,�����Խ���������		   
//CEΪ�ߴ���10us,����������.	 
void NRF24L01_TX_Mode(void)
{														 
	NRF24L01_CE=0;	    
	NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH);//дTX�ڵ��ַ 
	NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK	  

	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);     //ʹ��ͨ��0���Զ�Ӧ��    
	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01); //ʹ��ͨ��0�Ľ��յ�ַ  
	NRF24L01_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x1a);//�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10��
	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,40);       //����RFͨ��Ϊ40
	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);  //����TX�������,0db����,2Mbps,���������濪��   
	NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG,0x0e);    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�
	NRF24L01_Write_Reg(FLUSH_TX,0xff);//���TX FIFO�Ĵ��� 
	NRF24L01_Write_Reg(FLUSH_RX,0xff);//���RX FIFO�Ĵ��� 
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,0xff);	//���״̬�Ĵ���	
	NRF24L01_CE=1;//CEΪ��,10us����������
}



ALIGN(RT_ALIGN_SIZE)
static char thread_nrf_rx_stack[1024];
static struct rt_thread thread_nrf_rx_handle;


void nrf_rx_handle(nrf_rx_msg rx_msg)
{
	int i = 0;
	rx_msg.len = rx_msg.buf[0];
	for(i = 1;i <= rx_msg.len;i++)
		rt_kprintf("%c",rx_msg.buf[i]);
  rt_kprintf("\n");
	
}
void entry_thread_nrf_rx(void* parameter)
{
	while(1)
	{
		//wait for a new message
		rt_sem_take(&sem_nrf_rx, RT_WAITING_FOREVER);
		nrf_rx_handle(rx_msg);
	}
}

/* ����nrf_rx���߳� */
void thread_nrf_rx_init(void)
{
	/* ��ʼ��һ���ź��� */
	rt_sem_init(&sem_nrf_rx, "nrf_rx",0 ,RT_IPC_FLAG_FIFO);
	//mpu
	rt_thread_init(&thread_nrf_rx_handle,
                   "nrf_rx",
                   entry_thread_nrf_rx,
                   RT_NULL,
                   &thread_nrf_rx_stack[0],
                   sizeof(thread_nrf_rx_stack),8,10);
  rt_thread_startup(&thread_nrf_rx_handle);
}

/* thread_can_tx */
ALIGN(RT_ALIGN_SIZE)
static char thread_nrf_tx_stack[512];
static struct rt_thread thread_nrf_tx_handle;

void entry_thread_nrf_tx(void* parameter)
{
	while(1)
	{
		//wait for new message
		rt_mq_recv(&mq, &tx_msg, sizeof(nrf_tx_msg), RT_WAITING_FOREVER);
		NRF24L01_TX_Mode();
		NRF24L01_TxPacket(tx_msg);
		NRF24L01_RX_Mode();
	}
}

//send 
int nrf_tx_msg_send(nrf_tx_msg *tx_msg)
{
	return rt_mq_send(&mq, (void *)tx_msg,sizeof(nrf_tx_msg));
}

/* ����nrf_tx���߳� */
void thread_nrf_tx_init(void)
{
		/* ��ʼ��һ���ź��� */
	rt_sem_init(&sem_nrf_tx, "nrf_tx",0 ,RT_IPC_FLAG_FIFO);
	
	rt_mq_init(&mq,
			   "nrf tx", 
				msg_pool, 
				sizeof(nrf_tx_msg), 
				sizeof(msg_pool),
				RT_IPC_FLAG_FIFO);
	
	rt_thread_init(&thread_nrf_tx_handle,
                   "nrf_tx",
                   entry_thread_nrf_tx,
                   RT_NULL,
                   &thread_nrf_tx_stack[0],
                   sizeof(thread_nrf_tx_stack),8,10);
  rt_thread_startup(&thread_nrf_tx_handle);
}

//�����ʼ��
void NRF24L01_Thread_Init(void)
{
	thread_nrf_rx_init();
	thread_nrf_tx_init();
}

/*���췢��֡,��ʽ������ת����USBģ��ƥ�� */
static void nrf_tx_frame_construct(nrf_tx_msg *tx_msg,uint8_t *buf,uint8_t len)
{
	int i = 0;
	tx_msg->buf[i] = tx_msg->len = len;
	for(i = 1;i <= tx_msg->len;i++)
	{
		tx_msg->buf[i] = buf[i-1];
	}
}

void thread_nrf(void)
{
	uint8_t tmp[32];
	int i = 0;
	NRF24L01_Init();
	NRF24L01_RX_Mode();
	while(NRF24L01_Check())
	{
		rt_kprintf("nrf not found!\n");
	}
	NRF24L01_Thread_Init();
	while(1) {
		/*ÿһ֡���ݼ��ʱ�����Ϊ10ms*/
		rt_thread_delay(10);
		for(i = 0;i < 10;i++)
			tmp[i] = '0'+i;
		
		nrf_tx_frame_construct(&tx_msg,tmp,10);
		nrf_tx_msg_send(&tx_msg);	
	}
}

//nRF24L01�жϷ������
void EXTI3_IRQHandler(void)
{
	uint8_t istatus;
	//�ж��Ƿ�����·6������ж�
	if (EXTI_GetITStatus(EXTI_Line3)!=RESET)
	{
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3)==0)
			{
				istatus = NRF24L01_Read_Reg(STATUS);            // ��ȡ״̬�Ĵ������ж����ݽ���״��
				nrf_flag= istatus; 
				if(istatus&0x40)//bit6:���ݽ����ж�
				{
					NRF24L01_Read_Buf(RD_RX_PLOAD,rx_msg.buf,RX_PLOAD_WIDTH); //��ȡ����
					NRF24L01_Write_Reg(FLUSH_RX,0xff);//���RX FIFO�Ĵ���
					rt_sem_release(&sem_nrf_rx);
				}
				else if((istatus & 0x10)>0)////�ﵽ����ʹ����ж� 
				{
					NRF24L01_Write_Reg(FLUSH_TX,0xff);//���TX FIFO�Ĵ��� 	
					rt_sem_release(&sem_nrf_tx);
				}
				else if((istatus & 0x20)>0)//TX��������ж�
				{
					NRF24L01_Write_Reg(FLUSH_TX,0xff);//���TX FIFO�Ĵ���
					rt_sem_release(&sem_nrf_tx);
				}
					NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,istatus);//���״̬�Ĵ���
			}
			EXTI_ClearITPendingBit(EXTI_Line3); //�����־
	}
}



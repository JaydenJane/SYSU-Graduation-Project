#include <rtthread.h>
#include "stm32f10x.h"
#include "24l01.h"
#include "spi.h"
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//NRF24L01驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/13
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////

const uint8_t TX_ADDRESS[TX_ADR_WIDTH]={0x11,0x11,0x11,0x11,0x11}; //发送地址
const uint8_t RX_ADDRESS[RX_ADR_WIDTH]={0x11,0x11,0x11,0x11,0x11};
uint8_t tmp_buf[33],nrf_flag;	//tmp_buf为数据存储区域，nrf_flag为nrf24L01状态寄存器的情况

/* 邮箱控制块 */

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
	  /*关闭所有中断*/
    INTX_DISABLE();
    /*获得延时经过的tick*/
    us = us * (SysTick->LOAD/(1000000/RT_TICK_PER_SECOND));
    /*获得当前时间*/
    delta = SysTick->VAL;  
    /*循环获得当前时间，直到达到时间后退出*/
    do
    {
        if ( delta > SysTick->VAL )
            current_delay = delta - SysTick->VAL;
        else
        /*延时跨越了一次os tick的边界时*/
            current_delay = SysTick->LOAD + delta - SysTick->VAL;
    } while( current_delay < us );
		/*开启所有中断*/
		INTX_DISABLE();
}

//初始化24L01的IO口
void NRF24L01_Init(void)
{ 
	GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//外部中断，需要使能AFIO时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_3;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //PA3 中断输入  
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_ResetBits(GPIOA,GPIO_Pin_1|GPIO_Pin_2);//PA1,2下拉	
	//nRF24L01 NVIC配置
	NVIC_InitStructure.NVIC_IRQChannel=EXTI3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	//将GPIO管脚与外部中断线连接
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource3);
	//nRF24L01 EXIT配置
	EXTI_InitStructure.EXTI_Line=EXTI_Line3;
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	EXTI_ClearITPendingBit(EXTI_Line3);
		 
  SPI2_Init();    		    //初始化SPI	 
	SPI_Cmd(SPI2, DISABLE); // SPI外设不使能
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//SPI主机
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//发送接收8位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		//时钟悬空低
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	//数据捕获于第1个时钟沿
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS信号由软件控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;		//定义波特率预分频的值:波特率预分频值为16
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRC值计算的多项式
	SPI_Init(SPI2, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
	
	SPI_Cmd(SPI2, ENABLE); // 使能SPI外设 
	NRF24L01_CE=0; 				 // 使能24L01
	NRF24L01_CSN=1;				 // SPI片选取消  
	 		 	 
}
//检测24L01是否存在
//返回值:0，成功;1，失败	
uint8_t NRF24L01_Check(void)
{
	uint8_t buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	uint8_t i;
	SPI2_SetSpeed(SPI_BaudRatePrescaler_8); //spi速度为9Mhz（24L01的最大SPI时钟为10Mhz）   	 
	NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,buf,5);//写入5个字节的地址.	
	NRF24L01_Read_Buf(TX_ADDR,buf,5); //读出写入的地址  
	for(i=0;i<5;i++)if(buf[i]!=0XA5)break;	 							   
	if(i!=5)return 1;//检测24L01错误	
	return 0;		 //检测到24L01
}	 	 
//SPI写寄存器
//reg:指定寄存器地址
//value:写入的值
uint8_t NRF24L01_Write_Reg(uint8_t reg,uint8_t value)
{
	uint8_t status;	
	NRF24L01_CSN=0;                 //使能SPI传输
	status =SPI2_ReadWriteByte(reg);//发送寄存器号 
	SPI2_ReadWriteByte(value);      //写入寄存器的值
	NRF24L01_CSN=1;                 //禁止SPI传输	   
	return(status);       			    //返回状态值
}
//读取SPI寄存器值
//reg:要读的寄存器
uint8_t NRF24L01_Read_Reg(uint8_t reg)
{
	uint8_t reg_val;	    
 	NRF24L01_CSN = 0;          //使能SPI传输		
	SPI2_ReadWriteByte(reg);   //发送寄存器号
	reg_val=SPI2_ReadWriteByte(0XFF);//读取寄存器内容
	NRF24L01_CSN = 1;          //禁止SPI传输		    
	return(reg_val);           //返回状态值
}	
//在指定位置读出指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值 
uint8_t NRF24L01_Read_Buf(uint8_t reg,uint8_t *pBuf,uint8_t len)
{
	uint8_t status,uint8_t_ctr;	       
	NRF24L01_CSN = 0;           //使能SPI传输
	status=SPI2_ReadWriteByte(reg);//发送寄存器值(位置),并读取状态值   	   
	for(uint8_t_ctr=0;uint8_t_ctr<len;uint8_t_ctr++)pBuf[uint8_t_ctr]=SPI2_ReadWriteByte(0XFF);//读出数据
	NRF24L01_CSN=1;       //关闭SPI传输
	return status;        //返回读到的状态值
}
//在指定位置写指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
	uint8_t status,uint8_t_ctr;	    
 	NRF24L01_CSN = 0;          //使能SPI传输
	status = SPI2_ReadWriteByte(reg);//发送寄存器值(位置),并读取状态值
	for(uint8_t_ctr=0; uint8_t_ctr<len; uint8_t_ctr++)SPI2_ReadWriteByte(*pBuf++); //写入数据	 
	NRF24L01_CSN = 1;       //关闭SPI传输
	return status;          //返回读到的状态值
}				   
//启动NRF24L01发送一次数据
//txbuf:待发送数据首地址
//返回值:发送完成状况
uint8_t NRF24L01_TxPacket(nrf_tx_msg msg)
{
	uint8_t *txbuf = msg.buf;
 	SPI2_SetSpeed(SPI_BaudRatePrescaler_8);//spi速度为9Mhz（24L01的最大SPI时钟为10Mhz）   
	NRF24L01_CE=0;
  NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//写数据到TX BUF  32个字节
 	NRF24L01_CE=1;//启动发送	   

	rt_sem_take(&sem_nrf_tx, RT_TICK_PER_SECOND /10);
	
	if(nrf_flag & MAX_TX)//达到最大重发次数
	{
		NRF24L01_Write_Reg(FLUSH_TX,0xff);//清除TX_FIFO寄存器
		nrf_flag = 0;						//清除标记
		return MAX_TX; 
	}
	if(nrf_flag&TX_OK)//发送完成
	{
		nrf_flag = 0;			//清除标记
		return TX_OK;
	}
	nrf_flag=0;					//清除标记
	return 0xff;//发送失败
}

//启动NRF24L01发送一次数据
//txbuf:待发送数据首地址
//返回值:0，接收完成；其他，错误代码
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
{
	uint8_t sta;		    							   
	SPI2_SetSpeed(SPI_BaudRatePrescaler_8); //spi速度为9Mhz（24L01的最大SPI时钟为10Mhz）   
	sta=NRF24L01_Read_Reg(STATUS);  //读取状态寄存器的值    	 
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //清除TX_DS或MAX_RT中断标志
	if(sta&RX_OK)//接收到数据
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//读取数据
		NRF24L01_Write_Reg(FLUSH_RX,0xff);//清除RX FIFO寄存器 
		return 0; 
	}	   
	return 1;//没收到任何数据
}		

//该函数初始化NRF24L01到RX模式
//设置RX地址,写RX数据宽度,选择RF频道,波特率和LNA HCURR
//当CE变高后,即进入RX模式,并可以接收数据了		   
void NRF24L01_RX_Mode(void)
{
	NRF24L01_CE=0;	  
	NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t *)RX_ADDRESS,RX_ADR_WIDTH);//写RX节点地址
	
	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);    //使能通道0的自动应答    
	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);//使能通道0的接收地址  	 
	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,40);	     //设置RF通信频率		
	NRF24L01_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//选择通道0的有效数据宽度 	    
	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);//设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
	NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG, 0x0f);//配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式
	SPI2_SetSpeed(SPI_BaudRatePrescaler_8); //spi速度为9Mhz（24L01的最大SPI时钟为10Mhz） 
	NRF24L01_Write_Reg(FLUSH_RX,0xff);//清除RX FIFO寄存器 
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,0xff);	//***一定要清空状态寄存器，否则会出错。**
  NRF24L01_CE = 1; //CE为高,进入接收模式 
}						 
//该函数初始化NRF24L01到TX模式
//设置TX地址,写TX数据宽度,设置RX自动应答的地址,填充TX发送数据,选择RF频道,波特率和LNA HCURR
//PWR_UP,CRC使能
//当CE变高后,即进入RX模式,并可以接收数据了		   
//CE为高大于10us,则启动发送.	 
void NRF24L01_TX_Mode(void)
{														 
	NRF24L01_CE=0;	    
	NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH);//写TX节点地址 
	NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH); //设置TX节点地址,主要为了使能ACK	  

	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);     //使能通道0的自动应答    
	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01); //使能通道0的接收地址  
	NRF24L01_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x1a);//设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次
	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,40);       //设置RF通道为40
	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);  //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
	NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG,0x0e);    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断
	NRF24L01_Write_Reg(FLUSH_TX,0xff);//清除TX FIFO寄存器 
	NRF24L01_Write_Reg(FLUSH_RX,0xff);//清除RX FIFO寄存器 
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,0xff);	//清除状态寄存器	
	NRF24L01_CE=1;//CE为高,10us后启动发送
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

/* 创建nrf_rx子线程 */
void thread_nrf_rx_init(void)
{
	/* 初始化一个信号量 */
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

/* 创建nrf_tx子线程 */
void thread_nrf_tx_init(void)
{
		/* 初始化一个信号量 */
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

//软件初始化
void NRF24L01_Thread_Init(void)
{
	thread_nrf_rx_init();
	thread_nrf_tx_init();
}

/*构造发送帧,格式和无线转串口USB模块匹配 */
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
		/*每一帧数据间隔时长最短为10ms*/
		rt_thread_delay(10);
		for(i = 0;i < 10;i++)
			tmp[i] = '0'+i;
		
		nrf_tx_frame_construct(&tx_msg,tmp,10);
		nrf_tx_msg_send(&tx_msg);	
	}
}

//nRF24L01中断服务程序
void EXTI3_IRQHandler(void)
{
	uint8_t istatus;
	//判断是否是线路6引起的中断
	if (EXTI_GetITStatus(EXTI_Line3)!=RESET)
	{
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3)==0)
			{
				istatus = NRF24L01_Read_Reg(STATUS);            // 读取状态寄存其来判断数据接收状况
				nrf_flag= istatus; 
				if(istatus&0x40)//bit6:数据接收中断
				{
					NRF24L01_Read_Buf(RD_RX_PLOAD,rx_msg.buf,RX_PLOAD_WIDTH); //读取数据
					NRF24L01_Write_Reg(FLUSH_RX,0xff);//清除RX FIFO寄存器
					rt_sem_release(&sem_nrf_rx);
				}
				else if((istatus & 0x10)>0)////达到最大发送次数中断 
				{
					NRF24L01_Write_Reg(FLUSH_TX,0xff);//清除TX FIFO寄存器 	
					rt_sem_release(&sem_nrf_tx);
				}
				else if((istatus & 0x20)>0)//TX发送完成中断
				{
					NRF24L01_Write_Reg(FLUSH_TX,0xff);//清除TX FIFO寄存器
					rt_sem_release(&sem_nrf_tx);
				}
					NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,istatus);//清除状态寄存器
			}
			EXTI_ClearITPendingBit(EXTI_Line3); //清除标志
	}
}



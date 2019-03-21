/*
 * File      : thread_can.c
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013.6.15    majianjia   the first version
 */
 

#include <math.h>
#include "can.h"
#include "stm32f10x.h"
#include "string.h"
#include "sys.h"
#include "db_can_message.h"
#include "mpu9250.h"
#include "pwm.h"
#include "stdio.h"  
//define which esc is it
#define MOTO_NUMBER  0xFD
#define BROADCAST 0xFF
//#define MOTO_NUM 15
#define SEND_DATA  0x00000001

#define SET_ANGLE  0x00


#define CAN_FLASH_PAGE_SIZE			((uint16_t)0x400)
// use the last KB for ESC32 storage 
// and the previous kb for CAN ID
#define CAN_FLASH_WRITE_ADDR		(0x08000000 + (uint32_t)CAN_FLASH_PAGE_SIZE * 62)    

#define CAN_CS_ENABLE     do{PAout(15) = 0;}while(0)
#define CAN_CS_DISABLE    do{PAout(15) = 1;}while(0)

// "printf": necessary code, then you can print float using such as "printf("%f\n",Local_Time);" 
int fputc(int ch, FILE *f)  
{  
    USART_SendData(USART1, (uint8_t) ch);  
  
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {}    
     
    return ch;  
}  
int GetKey (void)  {   
  
    while (!(USART1->SR & USART_FLAG_RXNE));  
  
    return ((int)(USART1->DR & 0x1FF));  
} 


int CAN_ID = MOTO_NUMBER;
int CAN_CURRENT_ID = 0;

/* tx message buffer */
static struct rt_messagequeue mq;
static char msg_pool[1024];

static struct rt_mailbox mb;
static char msg_pool_mb[1024];
//
unsigned int volatile systick = 0;
unsigned int volatile seconds = 0;
float cpu_usage;


/* 邮箱控制块 */
static struct rt_semaphore sem_rx;

//
CAN_InitTypeDef        CAN_InitStructure;
CAN_FilterInitTypeDef  CAN_FilterInitStructure;
CanTxMsg TxMessage;
CanRxMsg RxMessage;

//void can_filter_init(void)
//{
//	CAN_FilterInitTypeDef  CAN_FilterInitStructure;  
//	uint16_t std_id0 =0xaa;  
//	uint16_t std_id1 =0x22;  
//	CAN_FilterInit(&CAN_FilterInitStructure); //CAN_FilterInitStructrue  
//	CAN_FilterInitStructure.CAN_FilterNumber=0;     //设置过滤器组 0~13  
//	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdList;    //过滤器列表模式
//	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;   //过滤器带宽为32位模式  
//	
// 
//	CAN_FilterInitStructure.CAN_FilterIdHigh= (std_id0<<5) ;  //左移5位 
//	CAN_FilterInitStructure.CAN_FilterIdLow=0|CAN_ID_STD;  //
//	
//	CAN_FilterInitStructure.CAN_FilterMaskIdHigh  =(std_id1<<5); 
//	CAN_FilterInitStructure.CAN_FilterMaskIdLow  = 0|CAN_ID_STD;  

//	//CAN_FilterInitStructure.CAN_FilterMaskIdHigh=((ext_id<<3)>>16) & 0xffff; //??????????  
//	//CAN_FilterInitStructure.CAN_FilterMaskIdLow=((ext_id<<3)& 0xffff) | CAN_ID_EXT;   //??????????  
//		
//	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0; 
//	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; 
//	CAN_FilterInit(&CAN_FilterInitStructure); 
//}
//can 总线初始化
void can_init(void)
{
	uint16_t std_id0 =0xFF - joint_id;  
	uint16_t std_id1 =0xFF;  
	GPIO_InitTypeDef  GPIO_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;	
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	/* GPIO clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);
	
	/* Disable the Serial Wire Jtag Debug Port SWJ-DP */
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable ,ENABLE);
	
	/* CANx Periph clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	
	/* Configure CAN pin: RX */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure CAN pin: TX */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//GPIO_PinRemapConfig(GPIO_Remapping_CAN , ENABLE);

	/* CAN register init */
	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);

	/* CAN cell init */
	CAN_InitStructure.CAN_TTCM = DISABLE;			//时间触发模式
	CAN_InitStructure.CAN_ABOM = ENABLE;			
	CAN_InitStructure.CAN_AWUM = DISABLE;			//睡眠模式？
	CAN_InitStructure.CAN_NART = ENABLE;//DISABLE;			//报文只发送一次
	CAN_InitStructure.CAN_RFLM = DISABLE;			// 是否覆盖已满的报文
	CAN_InitStructure.CAN_TXFP = DISABLE;			//发送优先级由报文决定
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;

	/* CAN Baudrate = 0.5MBps*/
	//APB = 32M
	//Rate = RCC_APB1PeriphClock/CAN_SJW+CAN_BS1+CAN_BS2/CAN_Prescaler; 
	// 0.5m = 36m / (1 + 3 + 2)/ 12 
	//that meas sample point is about 75% from the start
	//
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
	CAN_InitStructure.CAN_Prescaler = 12;
	CAN_Init(CAN1, &CAN_InitStructure);

	/* CAN filter init */
	CAN_FilterInitStructure.CAN_FilterNumber = 0;
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdList;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	//CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
	//CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	//CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
	//CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterIdHigh= (std_id0<<5);  //左移5位 
	CAN_FilterInitStructure.CAN_FilterIdLow= 0|CAN_ID_STD;   //
	
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh  =(std_id1<<5); 
	CAN_FilterInitStructure.CAN_FilterMaskIdLow  = 0|CAN_ID_STD;  
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	/* Enable CAN1 RX0 interrupt IRQ channel */
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	/* Enable FIFO 0 message pending Interrupt */
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
	CAN_CS_ENABLE;
	
}

//interrupt handle
void canbus_rx_handle(void)
{
   if(SET == CAN_GetITStatus(CAN1, CAN_IT_FMP0))
   {
	    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
			rt_sem_release(&sem_rx);
   }
}

//can id write to flash
//int can_id_to_flash(void) 
//{
//    FLASH_Status FLASHStatus;
//	
//    int ret = 0;

//    // Startup HSI clock
//    RCC_HSICmd(ENABLE);
//    while (RCC_GetFlagStatus(RCC_FLAG_HSIRDY) != SET);

//    FLASH_Unlock();

//    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
//	
//		if ((FLASHStatus = FLASH_ErasePage(CAN_FLASH_WRITE_ADDR)) == FLASH_COMPLETE) 
//		{
//			FLASH_ProgramWord(CAN_FLASH_WRITE_ADDR, CAN_ID);
//		}
//    FLASH_Lock();

//    // Shutdown HSI clock
//    RCC_HSICmd(DISABLE);
//    while (RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == SET);

//    return ret;
//}

//int can_id_from_flash(void)
//{
//	//memcpy((char *)CAN_CURRENT_ID, (char *)CAN_FLASH_WRITE_ADDR, sizeof(CAN_CURRENT_ID));
//	
//	CAN_CURRENT_ID = *((uint32_t*)CAN_FLASH_WRITE_ADDR);
//	
//	//if there is not id in flash
//	if(CAN_CURRENT_ID == 0x00 || CAN_CURRENT_ID == 0xff)
//		CAN_CURRENT_ID = 0;
//	return CAN_CURRENT_ID;
//}

static rt_timer_t timer1;
rt_uint32_t Count=0;
static void timeout1(void* parameter)
{
	rt_kprintf("%d\n",Count);
	Count=0;
}



int Amplitude = 0;
int Frequency = 0;
int Phase = 0;
int Bias = 0;
int p_1 = 10;
int p_2 = 0;


void can_rx_handle(CanRxMsg rx)
{
	  uint32_t id = 0,i = 0;
		id = rx.StdId;
		i = 0;
	 /* rt_kprintf("CANID:%d Data:",id);
		for(i = 0;i < rx.DLC;i++)
			rt_kprintf("%d",rx.Data[i]);
		rt_kprintf("\n");*/
		
		if(id==0xFF - joint_id){
			if (rx.Data[0] > 11){
					Amplitude = rx.Data[0];
					Frequency = rx.Data[1];
					Phase = rx.Data[2];
			}
			else{
					p_1 = rx.Data[0];
					p_2 = rx.Data[1];
					Bias = rx.Data[2];
			}
					  	
			
		}	
		else if(rx.Data[0]==0x01)
		{
			rt_mb_send(&mb, SEND_DATA);
			Count++;
		}

}


/* thread_can_rx */
ALIGN(RT_ALIGN_SIZE)
static char thread_can_rx_stack[1024];
static struct rt_thread thread_can_rx_handle;

void entry_thread_can_rx(void* parameter)
{
	while(1)
	{
		//wait for a new message
		rt_sem_take(&sem_rx, RT_WAITING_FOREVER);
		can_rx_handle(RxMessage);
	}
}

/* 创建can_rx子线程 */
void thread_can_rx_init(void)
{
	/* 初始化一个信号量 */
	rt_sem_init(&sem_rx, "can rx",0 ,RT_IPC_FLAG_FIFO);

		/* 初始化一个从机内部使用的邮箱 */
	rt_mb_init(&mb,				
			   "mbt", 
				msg_pool_mb, 
				sizeof(msg_pool_mb)/4,
				RT_IPC_FLAG_FIFO);
	
//	timer1 = rt_timer_create("timer1", 
//														timeout1, 
//														RT_NULL, 
//														1000, 
//														RT_TIMER_FLAG_PERIODIC);
//	if (timer1 != RT_NULL) rt_timer_start(timer1);			
	//mpu
	rt_thread_init(&thread_can_rx_handle,
                   "can_rx",
                   entry_thread_can_rx,
                   RT_NULL,
                   &thread_can_rx_stack[0],
                   sizeof(thread_can_rx_stack),20,10);
  rt_thread_startup(&thread_can_rx_handle);
}

/* thread_can_tx */
ALIGN(RT_ALIGN_SIZE)
static char thread_can_tx_stack[512];
static struct rt_thread thread_can_tx_handle;

void entry_thread_can_tx(void* parameter)
{
	//rt_thread_delay(RT_TICK_PER_SECOND);
	while(1)
	{
		//wait for new message
		rt_mq_recv(&mq, &TxMessage, sizeof(CanTxMsg), RT_WAITING_FOREVER);
		
		//if all mailbox are no available
		if(CAN_Transmit(CAN1, &TxMessage) == CAN_TxStatus_NoMailBox)
		{
			rt_thread_delay(1);
		}
	}
}



//send 
int can_message_send(CanTxMsg *TxMessage)
{
	return rt_mq_send(&mq, (void *)TxMessage,sizeof(CanTxMsg));
}
//urgent send
int can_message_urgent(CanTxMsg *TxMessage)
{
	return rt_mq_urgent(&mq, (void *)TxMessage,sizeof(CanTxMsg));
}

/* 创建can_tx子线程 */
void thread_can_tx_init(void)
{
	rt_mq_init(&mq,
			   "can tx", 
				msg_pool, 
				sizeof(CanTxMsg), 
				sizeof(msg_pool),
				RT_IPC_FLAG_FIFO);
			

				
	rt_thread_init(&thread_can_tx_handle,
                   "can_tx",
                   entry_thread_can_tx,
                   RT_NULL,
                   &thread_can_tx_stack[0],
                   sizeof(thread_can_tx_stack),20,10);
  rt_thread_startup(&thread_can_tx_handle);
}

//硬件初始化
static void hw_init(void)
{
	can_init();
}

//软件初始化
static void sw_init(void)
{
	thread_can_rx_init();
	thread_can_tx_init();
}

void show_id_from_led(void)
{
	#define TIME_PART 100 //ms
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	
	GPIO_SetBits(GPIOB, GPIO_Pin_3);
	GPIO_SetBits(GPIOB, GPIO_Pin_4);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	if(CAN_CURRENT_ID > 0 && CAN_CURRENT_ID*TIME_PART < RT_TICK_PER_SECOND)
	{
		//off
		GPIO_SetBits(GPIOB, GPIO_Pin_3);
		//on
		while(rt_tick_get() < TIME_PART * CAN_CURRENT_ID+1) ;
		GPIO_ResetBits(GPIOB, GPIO_Pin_3);
		//off
		while(rt_tick_get() < TIME_PART * (CAN_CURRENT_ID+2));
		GPIO_SetBits(GPIOB, GPIO_Pin_3);
	}
	//wait till 1s
	while(rt_tick_get() < RT_TICK_PER_SECOND);
}

/*
CanTxMsg: 需要初始化的CAN message结构体
id:       需要发送的id号 (1 ~13位大小)
len:      数据长度(0~8 字节)
data:     数据内容(8字节)
*/
void tx_message_init(CanTxMsg *message,rt_uint16_t id,rt_uint8_t len,rt_uint8_t *data)
{
		int i = 0;
		message->StdId = id & 0x1FFF;
		message->ExtId = 0x00;
		message->RTR = CAN_RTR_DATA;
		message->IDE = CAN_ID_STD;
		message->DLC = len;
		for (i = 0;i < len; i++)
				message->Data[i] = data[i];
}


int can_send_message(rt_uint16_t can_id, rt_uint8_t dlc, uint8_t *  data){
	CanTxMsg message;
	tx_message_init(&message, (rt_uint16_t)can_id, (rt_uint8_t) dlc, data);
	can_message_send(&message);
	
	return 1;
}

extern int16_t gyro[3],accel[3];
void thread_can(void)
{

	//Write can ID to flash 1 time programed
	//can_id_to_flash();
	//read can id from flash, every time to setup
	//can_id_from_flash();
	//software init
	sw_init();	
	//hardware init
	hw_init();
//	uint8_t data[2][8];

	rt_uint32_t value[1];
	value[0]=0;
	uint8_t d[1] = {7};
		
	while(1)
	{
//		rt_uint8_t data[8] = {1,2,3,4,5,6,7,8};
		rt_thread_delay(RT_TICK_PER_SECOND/10 );
//		CanTxMsg TxMessage;
//		tx_message_init(&TxMessage,MOTO_NUMBER,8,data);
//		can_message_send(&TxMessage);	
//		if (rt_mb_recv(&mb, value, RT_WAITING_FOREVER)== RT_EOK)
//		{
//			//value[0] = SEND_DATA;
//			if(value[0]==SEND_DATA){
//				get_Gyro(data[0]);
//				get_Accel(data[1]);

//		
//				
//							
//				tx_message_init(&TxMessage,MOTO_NUMBER,8,data[1]);
//				can_message_send(&TxMessage);	
//				can_send_message(0xDD, 1, d);

//		}
//		
//		}
	}
}
